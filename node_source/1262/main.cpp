#include <Arduino.h>
#include <RadioLib.h>
#include "SerialDecoder.h"
#include "Packets.h"
#include <queue>

// 固件版本定义
#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 2
#define FIRMWARE_VERSION_PATCH 0

// 硬件版本
#define HARDWARE_REVISION 1

// 设备型号
#define MODEL_NAME "AeroLink-Node-S"

// 调试宏定义
#define DEBUG_MODE 0  // 设置为0关闭调试输出,1开启调试输出
#if DEBUG_MODE
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(x, ...)
#endif

// SX1262 引脚定义 (根据您的硬件连接调整)
#define NSS_PIN   8
#define DIO1_PIN  14
#define NRST_PIN  12
#define BUSY_PIN  13

// 最大接收队列大小
#define MAX_RX_QUEUE_SIZE 100

// TX缓冲区设置
#define TX_BUFFER_SIZE 250       // TX缓冲区最大大小
#define TX_BUFFER_THRESHOLD 230   // 触发发送的阈值
#define TX_TIMEOUT_MS 50          // 超时时间（毫秒）

// 创建SX1262实例
SX1262 radio = new Module(NSS_PIN, DIO1_PIN, NRST_PIN, BUSY_PIN);

// 接收数据结构
struct ReceivedPacket {
    float rssi;
    float snr;
    int crc_status;  // 添加 CRC 状态字段 (1=正确, 0=错误)
    std::vector<uint8_t> data;
};

// TX缓冲区结构
struct TxBuffer {
    uint8_t data[TX_BUFFER_SIZE];
    size_t length;
    unsigned long lastDataTime;  // 最后一次添加数据的时间
    SemaphoreHandle_t mutex;
    
    TxBuffer() : length(0), lastDataTime(0) {
        mutex = xSemaphoreCreateMutex();
    }
    
    // 添加数据到缓冲区
    bool addData(const uint8_t* newData, size_t newLen) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            bool result = false;
            if (length + newLen <= TX_BUFFER_SIZE) {
                memcpy(data + length, newData, newLen);
                length += newLen;
                lastDataTime = millis();
                result = true;
            }
            xSemaphoreGive(mutex);
            return result;
        }
        return false;
    }
    
    // 清空缓冲区
    void clear() {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            length = 0;
            lastDataTime = 0;
            xSemaphoreGive(mutex);
        }
    }
    
    // 获取缓冲区数据（用于发送）
    size_t getData(uint8_t* buffer, size_t maxLen) {
        size_t copiedLen = 0;
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            copiedLen = (length < maxLen) ? length : maxLen;
            if (copiedLen > 0) {
                memcpy(buffer, data, copiedLen);
            }
            xSemaphoreGive(mutex);
        }
        return copiedLen;
    }
    
    // 检查是否需要发送
    bool shouldSend() {
        bool result = false;
        if (xSemaphoreTake(mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            if (length > 0) {
                // 检查是否超过阈值或超时
                result = (length >= TX_BUFFER_THRESHOLD) || 
                        ((millis() - lastDataTime) >= TX_TIMEOUT_MS);
            }
            xSemaphoreGive(mutex);
        }
        return result;
    }
    
    // 获取当前缓冲区长度
    size_t getLength() {
        size_t len = 0;
        if (xSemaphoreTake(mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            len = length;
            xSemaphoreGive(mutex);
        }
        return len;
    }
};

// 全局变量
PacketDecoder decoder;
NodeWorkingMode currentMode = NodeWorkingMode::NOT_SET;
LoRaParam currentLoRaParams;
volatile bool receivedFlag = false;
volatile bool rxTaskRunning = false;  // 添加任务运行状态标志
TxBuffer txBuffer;  // TX缓冲区

// 使用队列存储接收到的数据包(环形缓冲区)
std::queue<ReceivedPacket> rxQueue;
SemaphoreHandle_t rxQueueMutex;
TaskHandle_t rxTaskHandle = NULL;

// 函数声明
void setFlag(void);
void configureLoRa(const LoRaParam& params);
void sendToHost(PacketType type, const uint8_t* data, size_t len);
void getESP32ChipID(uint8_t* id);
void rxTask(void* parameter);
void processRxQueue();
void processTxBuffer();  // 新增：处理TX缓冲区
void handleQueryInfo();
void handleQueryCaps();
void handleSetLoRaParam(const PacketDecoder::DecodedPacket& packet);
void handleTxData(const PacketDecoder::DecodedPacket& packet);
uint16_t getDeviceCapabilities();

void setup() {
    Serial.begin(921600);
    // set serial buffer size to maximum for fast transfers
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(4096);
    delay(100);
    
    // Turn on the LED to indicate startup
    pinMode(LED_BUILTIN, OUTPUT);
    DEBUG_PRINTLN(F("[SX1262] Initializing ... "));
    
    // 创建互斥锁
    rxQueueMutex = xSemaphoreCreateMutex();
    
    // 设置默认LoRa参数
    currentLoRaParams.frequency = 434.0;
    currentLoRaParams.bandwidth = 125.0;
    currentLoRaParams.spreading_factor = 9;
    currentLoRaParams.coding_rate = 5;
    currentLoRaParams.sync_word = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
    currentLoRaParams.tx_power = 10;
    currentLoRaParams.preamble_length = 8;
    
    // 初始化SX1262,使用默认参数
    int state = radio.begin(currentLoRaParams.frequency,
                            currentLoRaParams.bandwidth,
                            currentLoRaParams.spreading_factor,
                            currentLoRaParams.coding_rate,
                            currentLoRaParams.sync_word,
                            currentLoRaParams.tx_power,
                            currentLoRaParams.preamble_length,
                            1.6,  // TCXO电压
                            false // 使用DC-DC调节器
    );
    
    if (state == RADIOLIB_ERR_NONE) {
        DEBUG_PRINTLN(F("success!"));
    } else {
        DEBUG_PRINT(F("failed, code "));
        DEBUG_PRINTLN(state);
        while (true);
    }
    
    DEBUG_PRINTLN(F("Node ready to receive commands from host..."));
}

void loop() {
    // 处理串口数据
    if (Serial.available()) {
        static uint8_t inputBuffer[512];
        auto data_available = Serial.available();
        size_t bytesRead = Serial.readBytes(inputBuffer, data_available);
        
        auto packets = decoder.decode(inputBuffer, bytesRead);
        
        for (const auto& packet : packets) {
            switch (packet.type) {
                case PacketType::CMD_QUERY_INFO:
                    handleQueryInfo();
                    break;

                case PacketType::CMD_QUERY_CAPS:
                    handleQueryCaps();
                    break;

                case PacketType::CMD_SET_LORA_PARAM:
                    handleSetLoRaParam(packet);
                    break;
                    
                case PacketType::TX_DATA:
                    handleTxData(packet);
                    break;
                    
                default:
                    DEBUG_PRINTLN(F("Unknown packet type received"));
                    break;
            }
        }
    }
    
    // 处理接收队列中的数据(发送到上位机)
    processRxQueue();
    
    // 处理TX缓冲区（检查是否需要发送）
    if (currentMode == NodeWorkingMode::TX) {
        processTxBuffer();
    }
    
    // 小延时避免看门狗
    delay(1);
}

// 处理TX缓冲区
void processTxBuffer() {
    if (txBuffer.shouldSend()) {
        static uint8_t sendBuffer[TX_BUFFER_SIZE];
        size_t dataLen = txBuffer.getData(sendBuffer, TX_BUFFER_SIZE);
        
        if (dataLen > 0) {
            // 发送数据
            int state = radio.transmit(sendBuffer, dataLen);
            
            if (state == RADIOLIB_ERR_NONE) {
                DEBUG_PRINT(F("Buffer transmitted: "));
                DEBUG_PRINT(dataLen);
                DEBUG_PRINTLN(F(" bytes successfully"));
                
                // 清空缓冲区
                txBuffer.clear();
                
                // 获取发送后的信息(可选)
                DEBUG_PRINT(F("Datarate: "));
                DEBUG_PRINT(radio.getDataRate());
                DEBUG_PRINTLN(F(" bps"));
            } else {
                DEBUG_PRINT(F("Buffer transmission failed, code "));
                DEBUG_PRINTLN(state);
                
                // 发送失败也清空缓冲区，避免重复发送
                txBuffer.clear();
            }
        }
    }
}

// 异步接收任务(在独立的核心上运行)
void rxTask(void* parameter) {
    DEBUG_PRINTLN(F("RX Task started on core 0"));
    rxTaskRunning = true;
    
    while (rxTaskRunning) {
        if (receivedFlag) {
            receivedFlag = false;
            
            // 读取数据
            static uint8_t data[256];
            auto len = radio.getPacketLength();
            int state = radio.readData(data, len);
            
            // 创建接收数据包
            ReceivedPacket rxPacket;
            rxPacket.rssi = radio.getRSSI();
            rxPacket.snr = radio.getSNR();
            
            if (state == RADIOLIB_ERR_NONE) {
                // CRC 正确
                rxPacket.crc_status = 1;
                rxPacket.data.assign(data, data + len);
                
                DEBUG_PRINT(F("[RX] Packet received: "));
                DEBUG_PRINT(len);
                DEBUG_PRINT(F(" bytes, RSSI: "));
                DEBUG_PRINT(rxPacket.rssi);
                DEBUG_PRINT(F(" dBm, SNR: "));
                DEBUG_PRINT(rxPacket.snr);
                DEBUG_PRINTLN(F(" dB, CRC: OK"));
                
            } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                // CRC 错误,但仍然保存数据
                rxPacket.crc_status = 0;
                rxPacket.data.assign(data, data + len);
                
                DEBUG_PRINT(F("[RX] Packet with CRC error: "));
                DEBUG_PRINT(len);
                DEBUG_PRINT(F(" bytes, RSSI: "));
                DEBUG_PRINT(rxPacket.rssi);
                DEBUG_PRINT(F(" dBm, SNR: "));
                DEBUG_PRINT(rxPacket.snr);
                DEBUG_PRINTLN(F(" dB, CRC: ERROR"));
                
            } else {
                // 其他错误,不保存数据
                DEBUG_PRINT(F("[RX] Failed to read data, code "));
                DEBUG_PRINTLN(state);
                
                // 立即重新开始接收
                if (rxTaskRunning) {
                    radio.startReceive();
                }
                continue;  // 跳过保存到队列的步骤
            }
            
            // 添加到队列(线程安全) - CRC正确或CRC错误的数据都会被添加
            if (xSemaphoreTake(rxQueueMutex, portMAX_DELAY) == pdTRUE) {
                // 限制队列大小,避免内存溢出
                if (rxQueue.size() >= MAX_RX_QUEUE_SIZE) {
                    rxQueue.pop();  // 移除最旧的数据包
                }
                rxQueue.push(rxPacket);
                xSemaphoreGive(rxQueueMutex);
            }
            
            // 立即重新开始接收
            if (rxTaskRunning) {
                radio.startReceive();
            }
        }
        
        // 小延时,让出CPU
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    DEBUG_PRINTLN(F("RX Task stopped"));
    rxTaskHandle = NULL;
    vTaskDelete(NULL);  // 删除自己
}

// 处理接收队列,将数据发送到上位机
void processRxQueue() {
    if (xSemaphoreTake(rxQueueMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
        while (!rxQueue.empty()) {
            ReceivedPacket rxPacket = rxQueue.front();
            rxQueue.pop();
            
            // 创建RxDataPacket
            size_t totalSize = sizeof(RxDataPacket) + rxPacket.data.size();
            uint8_t* buffer = new uint8_t[totalSize];
            
            RxDataPacket* packet = reinterpret_cast<RxDataPacket*>(buffer);
            packet->rssi = rxPacket.rssi;
            packet->snr = rxPacket.snr;
            packet->len = rxPacket.data.size();
            packet->crc_status = rxPacket.crc_status;  // 添加 CRC 状态
            memcpy(packet->data, rxPacket.data.data(), rxPacket.data.size());
            
            // 发送到上位机
            sendToHost(PacketType::RX_DATA, buffer, totalSize);
            
            delete[] buffer;
        }
        xSemaphoreGive(rxQueueMutex);
    }
}

// 处理查询信息命令
void handleQueryInfo() {
    CMDQueryInfoPayload payload;

    // 获取ESP32芯片ID
    getESP32ChipID(payload.node_id);

    // 填充当前工作模式和LoRa参数
    payload.node_working_mode = currentMode;
    payload.node_info = currentLoRaParams;

    // 发送到上位机
    sendToHost(PacketType::RET_NODE_INFO, (uint8_t*)&payload, sizeof(payload));

    DEBUG_PRINTLN(F("Node info sent to host"));
}

// 处理查询能力命令
void handleQueryCaps() {
    NodeCapsPayload payload;

    // 获取设备 ID
    getESP32ChipID(payload.node_id);

    // 获取设备能力
    payload.capabilities = getDeviceCapabilities();

    // 固件版本
    payload.firmware_major = FIRMWARE_VERSION_MAJOR;
    payload.firmware_minor = FIRMWARE_VERSION_MINOR;
    payload.firmware_patch = FIRMWARE_VERSION_PATCH;

    // 硬件版本
    payload.hardware_revision = HARDWARE_REVISION;

    // 设备型号
    strncpy(payload.model_name, MODEL_NAME, sizeof(payload.model_name));
    payload.model_name[sizeof(payload.model_name) - 1] = '\0';

    // 发送响应
    sendToHost(PacketType::RET_NODE_CAPS, (uint8_t*)&payload, sizeof(payload));

    DEBUG_PRINTLN(F("Node capabilities sent to host"));
}

// 获取设备支持的能力
uint16_t getDeviceCapabilities() {
    // 默认支持收发功能
    return static_cast<uint16_t>(NodeCapability::CAP_TX) |
           static_cast<uint16_t>(NodeCapability::CAP_RX);
}

// 处理设置LoRa参数命令
void handleSetLoRaParam(const PacketDecoder::DecodedPacket& packet) {
    if (packet.data.size() >= sizeof(SetLoRaParamPayload)) {
        const SetLoRaParamPayload* setParam = 
            reinterpret_cast<const SetLoRaParamPayload*>(packet.data.data());
        
        // 获取新模式
        NodeWorkingMode newMode = setParam->node_working_mode;
        
        // 更新LoRa参数
        currentLoRaParams = setParam->params;
        
        // 配置LoRa参数（这会让radio进入standby模式）
        configureLoRa(currentLoRaParams);
        
        // 检查是否需要切换模式
        bool modeChanged = (newMode != currentMode);
        
        // 如果模式改变，需要先清理当前模式
        if (modeChanged) {
            // 第一步:清理当前模式
            if (currentMode == NodeWorkingMode::RX) {
                // 停止接收模式
                radio.clearDio1Action();
                
                // 安全删除接收任务
                if (rxTaskHandle != NULL) {
                    rxTaskRunning = false;  // 通知任务停止
                    
                    // 等待任务完全停止(最多等待1秒)
                    int waitCount = 0;
                    while (rxTaskHandle != NULL && waitCount < 1000) {
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                        waitCount++;
                    }
                    
                    if (rxTaskHandle != NULL) {
                        // 如果任务仍未停止,强制删除(不推荐,但作为保护措施)
                        DEBUG_PRINTLN(F("Warning: Force deleting RX task"));
                        vTaskDelete(rxTaskHandle);
                        rxTaskHandle = NULL;
                    }
                    
                    DEBUG_PRINTLN(F("RX mode stopped"));
                }
                
                // 清空接收队列
                if (xSemaphoreTake(rxQueueMutex, portMAX_DELAY) == pdTRUE) {
                    while (!rxQueue.empty()) {
                        rxQueue.pop();
                    }
                    xSemaphoreGive(rxQueueMutex);
                }
            } else if (currentMode == NodeWorkingMode::TX) {
                // 清空TX缓冲区
                txBuffer.clear();
                DEBUG_PRINTLN(F("TX mode stopped, buffer cleared"));
            }
            // 如果是 NOT_SET 模式,不需要清理
        }
        
        // 第二步:进入目标模式（无论模式是否改变都需要重新初始化）
        if (newMode == NodeWorkingMode::RX) {
            // 进入接收模式
            digitalWrite(LED_BUILTIN, LOW); // 熄灭LED表示进入RX模式
            // 如果模式没有改变但任务已经在运行，需要重新设置中断
            if (!modeChanged && rxTaskHandle != NULL) {
                // 清除旧的中断设置
                radio.clearDio1Action();
            }
            
            // 设置中断
            radio.setDio1Action(setFlag);
            
            // 启动接收
            int state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE) {
                DEBUG_PRINTLN(F("Starting RX mode"));
                
                // 创建接收任务(如果还没有创建)
                if (rxTaskHandle == NULL) {
                    receivedFlag = false;  // 清除标志
                    xTaskCreatePinnedToCore(
                        rxTask,           // 任务函数
                        "RX Task",        // 任务名称
                        4096,             // 栈大小
                        NULL,             // 参数
                        1,                // 优先级
                        &rxTaskHandle,    // 任务句柄
                        0                 // 核心0
                    );
                    DEBUG_PRINTLN(F("RX task created"));
                } else {
                    DEBUG_PRINTLN(F("RX mode restarted with new parameters"));
                }
                
                DEBUG_PRINTLN(F("Started listening for LoRa packets"));
            } else {
                DEBUG_PRINT(F("Failed to start receive mode, code "));
                DEBUG_PRINTLN(state);
                // 如果启动失败,设置为 NOT_SET 模式
                newMode = NodeWorkingMode::NOT_SET;
            }
        } else if (newMode == NodeWorkingMode::TX) {
            // 进入发送模式
            // 清空TX缓冲区，准备新的发送
            txBuffer.clear();
            DEBUG_PRINTLN(F("Entered TX mode with buffer"));
            digitalWrite(LED_BUILTIN, HIGH); // 点亮LED表示进入TX模式
        } else if (newMode == NodeWorkingMode::NOT_SET) {
            // 进入未设置模式
            // radio已经在standby状态
            DEBUG_PRINTLN(F("Entered NOT_SET mode"));
        }
        
        // 更新当前模式
        currentMode = newMode;
        
        // 打印模式信息
        if (modeChanged) {
            DEBUG_PRINT(F("Mode changed to: "));
        } else {
            DEBUG_PRINT(F("Mode remains: "));
        }
        
        switch(currentMode) {
            case NodeWorkingMode::TX:
                DEBUG_PRINTLN(F("TX"));
                break;
            case NodeWorkingMode::RX:
                DEBUG_PRINTLN(F("RX"));
                break;
            case NodeWorkingMode::NOT_SET:
                DEBUG_PRINTLN(F("NOT_SET"));
                break;
            default:
                DEBUG_PRINTLN(F("UNKNOWN"));
                break;
        }
    }
}

// 处理发送数据命令（修改：添加到缓冲区而不是立即发送）
void handleTxData(const PacketDecoder::DecodedPacket& packet) {
    if (currentMode != NodeWorkingMode::TX) {
        DEBUG_PRINTLN(F("Error: Not in TX mode"));
        return;
    }
    
    if (packet.data.size() >= sizeof(int)) {
        const TxDataPacket* txData = 
            reinterpret_cast<const TxDataPacket*>(packet.data.data());
        
        size_t actualDataSize = packet.data.size() - sizeof(int);
        
        if (actualDataSize > 0 && txData->len > 0) {
            const uint8_t* dataPtr = packet.data.data() + sizeof(int);
            size_t dataToAdd = (actualDataSize < txData->len) ? actualDataSize : txData->len;
            
            // 检查缓冲区空间
            if (txBuffer.getLength() + dataToAdd > TX_BUFFER_SIZE) {
                // 如果添加新数据会超出缓冲区，先发送当前缓冲区
                DEBUG_PRINTLN(F("Buffer full, sending current buffer first"));
                
                static uint8_t sendBuffer[TX_BUFFER_SIZE];
                size_t dataLen = txBuffer.getData(sendBuffer, TX_BUFFER_SIZE);
                
                if (dataLen > 0) {
                    int state = radio.transmit(sendBuffer, dataLen);
                    
                    if (state == RADIOLIB_ERR_NONE) {
                        DEBUG_PRINT(F("Force transmitted buffer: "));
                        DEBUG_PRINT(dataLen);
                        DEBUG_PRINTLN(F(" bytes"));
                    } else {
                        DEBUG_PRINT(F("Force transmission failed, code "));
                        DEBUG_PRINTLN(state);
                    }
                    
                    txBuffer.clear();
                }
            }
            
            // 添加数据到缓冲区
            if (txBuffer.addData(dataPtr, dataToAdd)) {
                DEBUG_PRINT(F("Added "));
                DEBUG_PRINT(dataToAdd);
                DEBUG_PRINT(F(" bytes to TX buffer. Buffer size: "));
                DEBUG_PRINTLN(txBuffer.getLength());
            } else {
                DEBUG_PRINTLN(F("Failed to add data to TX buffer"));
            }
        }
    }
}

// 配置LoRa参数
void configureLoRa(const LoRaParam& params) {
    DEBUG_PRINTLN(F("Configuring LoRa parameters..."));
    
    // 先进入待机模式
    radio.standby();
    
    // 设置频率
    if (radio.setFrequency(params.frequency) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        DEBUG_PRINTLN(F("Selected frequency is invalid for this module!"));
    }
    
    // 设置带宽
    if (radio.setBandwidth(params.bandwidth) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        DEBUG_PRINTLN(F("Selected bandwidth is invalid for this module!"));
    }
    
    // 设置扩频因子
    if (radio.setSpreadingFactor(params.spreading_factor) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        DEBUG_PRINTLN(F("Selected spreading factor is invalid for this module!"));
    }
    
    // 设置编码率
    if (radio.setCodingRate(params.coding_rate) == RADIOLIB_ERR_INVALID_CODING_RATE) {
        DEBUG_PRINTLN(F("Selected coding rate is invalid for this module!"));
    }
    
    // 设置同步字
    if (radio.setSyncWord(params.sync_word) != RADIOLIB_ERR_NONE) {
        DEBUG_PRINTLN(F("Unable to set sync word!"));
    }
    
    // 设置输出功率
    if (radio.setOutputPower(params.tx_power) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        DEBUG_PRINTLN(F("Selected output power is invalid for this module!"));
    }
    
    // 设置前导码长度
    if (radio.setPreambleLength(params.preamble_length) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
        DEBUG_PRINTLN(F("Selected preamble length is invalid for this module!"));
    }
    
    // 设置CRC
    radio.setCRC(true);
    
    DEBUG_PRINTLN(F("LoRa parameters configured"));
}

// 中断服务程序
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
    receivedFlag = true;
}

// 发送数据到上位机
void sendToHost(PacketType type, const uint8_t* data, size_t len) {
    // 帧格式: 0x7E + PacketType + Length(4 bytes, little-endian) + Payload + 0x7E
    
    Serial.write(0x7E);  // 起始标志
    Serial.write(static_cast<uint8_t>(type));  // 数据类型
    
    // 发送长度 (4字节,小端)
    uint32_t length = len;
    Serial.write((uint8_t)(length & 0xFF));
    Serial.write((uint8_t)((length >> 8) & 0xFF));
    Serial.write((uint8_t)((length >> 16) & 0xFF));
    Serial.write((uint8_t)((length >> 24) & 0xFF));
    
    // 发送数据
    Serial.write(data, len);
    
    Serial.write(0x7E);  // 结束标志
    Serial.flush();
}

// 获取ESP32芯片ID
void getESP32ChipID(uint8_t* id) {
    uint64_t chipid = ESP.getEfuseMac();
    id[0] = (chipid >> 40) & 0xFF;
    id[1] = (chipid >> 32) & 0xFF;
    id[2] = (chipid >> 24) & 0xFF;
    id[3] = (chipid >> 16) & 0xFF;
    id[4] = (chipid >> 8) & 0xFF;
    id[5] = chipid & 0xFF;
}