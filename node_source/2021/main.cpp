#include <Arduino.h>
#include <SPI.h>
#include "SerialDecoder.h"
#include "Packets.h"
#include "LR2021.h"
#include <queue>

// 固件版本定义
#define FIRMWARE_VERSION_MAJOR 2
#define FIRMWARE_VERSION_MINOR 0
#define FIRMWARE_VERSION_PATCH 0

// 硬件版本
#define HARDWARE_REVISION 1

// 设备型号
#define MODEL_NAME "RFUnit(TCXO)"

// 调试宏定义
#define DEBUG_MODE 0  // 设置为0关闭调试输出,1开启调试输出
#if DEBUG_MODE
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x, ...)
    #define DEBUG_PRINTF(x, ...)
#endif

// LR2021 引脚定义 (STICK2021)
#define PIN_NSS     8
#define PIN_BUSY    13
#define PIN_RST     12
#define PIN_DIO9    14
#define PIN_MISO    11
#define PIN_MOSI    10
#define PIN_SCK     9

// 最大接收队列大小
#define MAX_RX_QUEUE_SIZE 100

// TX缓冲区设置
#define TX_BUFFER_SIZE 250       // TX缓冲区最大大小

// 最大 SF 值
#define MAX_SF 12
#define TX_LED 35
#define TX_INDICATOR_PIN 17
// 创建 SPI 实例
SPIClass spi2(HSPI);

// 创建 LR2021 实例
LR2021 radio(PIN_NSS, PIN_BUSY, PIN_RST, PIN_DIO9, &spi2);

// 接收数据结构
struct ReceivedPacket {
    float rssi;
    float snr;
    int crc_status;      // CRC 状态字段 (1=正确, 0=错误)
    uint8_t sf;          // 接收时的 SF
    uint8_t detector_id; // 检测器 ID
    std::vector<uint8_t> data;
};


// 全局变量
PacketDecoder decoder;
NodeWorkingMode currentMode = NodeWorkingMode::NOT_SET;
LoRaParam currentLoRaParams;
volatile bool receivedFlag = false;
volatile bool rxTaskRunning = false;  // 添加任务运行状态标志

// 当前配置的 SF 列表 (用于多 SF 接收)
uint8_t configuredMainSf = 0;
uint8_t configuredSideSf[3] = {0, 0, 0};

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
void handleQueryInfo();
void handleQueryCaps();
void handleSetLoRaParam(const PacketDecoder::DecodedPacket& packet);
void handleTxData(const PacketDecoder::DecodedPacket& packet);
uint16_t getDeviceCapabilities();
void setupMultiSfRx(uint8_t mainSf);
uint8_t getSfFromDetector(uint8_t detector);
uint8_t getBwCode(float bandwidth);
uint8_t getCrCode(uint8_t codingRate);
uint8_t getLdroSetting(uint8_t sf, float bandwidth);

void setup() {
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(4096);
    Serial.begin(115200);
    // set serial buffer size to maximum for fast transfers

    delay(100);

    // Turn on the LED to indicate startup
    pinMode(TX_LED, OUTPUT);
    pinMode(TX_INDICATOR_PIN, INPUT);
    DEBUG_PRINTLN(F("[LR2021] Initializing ... "));

    // 创建互斥锁
    rxQueueMutex = xSemaphoreCreateMutex();

    // 初始化 SPI
    spi2.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

    // 设置默认LoRa参数
    currentLoRaParams.frequency = 434.0;
    currentLoRaParams.bandwidth = 125.0;
    currentLoRaParams.spreading_factor = 9;
    currentLoRaParams.coding_rate = 5;
    currentLoRaParams.sync_word = LR2021_LORA_SYNCWORD_PRIVATE;
    currentLoRaParams.tx_power = 10;
    currentLoRaParams.preamble_length = 8;

    // 初始化 LR2021
    if (!radio.begin(LR2021_TCXO_VOLTAGE_1_8V)) {
        DEBUG_PRINTLN(F("failed!"));
        while (true);
    }

    DEBUG_PRINTLN(F("success!"));

    // 设置为 LoRa 模式
    radio.setPacketType(LR2021_PACKET_TYPE_LORA);

    // 配置默认参数
    configureLoRa(currentLoRaParams);

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

    // 小延时避免看门狗
    delay(1);
}

// 获取带宽代码
uint8_t getBwCode(float bandwidth) {
    if (bandwidth <= 31.25) return LR2021_LORA_BW_31;
    if (bandwidth <= 41.67) return LR2021_LORA_BW_41;
    if (bandwidth <= 62.50) return LR2021_LORA_BW_62;
    if (bandwidth <= 83.34) return LR2021_LORA_BW_83;
    if (bandwidth <= 101.5625) return LR2021_LORA_BW_101;
    if (bandwidth <= 125.0) return LR2021_LORA_BW_125;
    if (bandwidth <= 203.125) return LR2021_LORA_BW_203;
    if (bandwidth <= 250.0) return LR2021_LORA_BW_250;
    if (bandwidth <= 406.25) return LR2021_LORA_BW_406;
    if (bandwidth <= 500.0) return LR2021_LORA_BW_500;
    if (bandwidth <= 812.5) return LR2021_LORA_BW_812;
    return LR2021_LORA_BW_1000;
}

// 获取编码率代码
uint8_t getCrCode(uint8_t codingRate) {
    switch (codingRate) {
        case 5: return LR2021_LORA_CR_4_5;
        case 6: return LR2021_LORA_CR_4_6;
        case 7: return LR2021_LORA_CR_4_7;
        case 8: return LR2021_LORA_CR_4_8;
        default: return LR2021_LORA_CR_4_5;
    }
}

// 获取 LDRO (Low Data Rate Optimization) 设置
// LDRO = 0 (OFF): SF5-10 任意BW, SF11 + BW>=250, SF12 + BW>=406
// LDRO = 1 (ON): SF11 + BW125, SF12 + BW125/250
uint8_t getLdroSetting(uint8_t sf, float bandwidth) {
    if (sf <= 10) {
        return LR2021_LORA_LDRO_OFF;
    }
    if (sf == 11 && bandwidth >= 250.0f) {
        return LR2021_LORA_LDRO_OFF;
    }
    if (sf == 12 && bandwidth >= 400.0f) {
        return LR2021_LORA_LDRO_OFF;
    }
    return LR2021_LORA_LDRO_ON;
}

// 设置多 SF 接收
// 规则: 包含主 SF 共 4 个，如果 SF 超过 12 则不配置
void setupMultiSfRx(uint8_t mainSf) {
    configuredMainSf = mainSf;

    // 计算 side SF，往后数最多 3 个，但不超过 SF12
    uint8_t sideSf1 = 0, sideSf2 = 0, sideSf3 = 0;

    switch (mainSf)
    {
    case 5:
        sideSf1 = 6;
        sideSf2 = 7;
        sideSf3 = 8;
        break;
    case 6:
        sideSf1 = 7;
        sideSf2 = 8;
        sideSf3 = 9;
        break;
    case 7:
        sideSf1 = 8;
        sideSf2 = 9;
        sideSf3 = 10;
        break;
    case 8:
        sideSf1 = 9;
        sideSf2 = 10;
        sideSf3 = 11;
        break;
    case 9:
        sideSf1 = 10;
        sideSf2 = 11;
        sideSf3 = 12;
        break;
    case 10:
        sideSf1 = 11;
        sideSf2 = 12;
        sideSf3 = 0;  // SF13 不存在
        break;
    case 11:
        sideSf1 = 12;
        sideSf2 = 0;
        sideSf3 = 0;
        break;
    case 12:
        // SF12 是最高的，没有 side SF
        sideSf1 = 0;
        sideSf2 = 0;
        sideSf3 = 0;
        break;
    default:
        // 未知 SF，使用 SF9 作为默认
        mainSf = 9;
        sideSf1 = 10;
        sideSf2 = 11;
        sideSf3 = 12;
        break;
    }

    configuredSideSf[0] = sideSf1;
    configuredSideSf[1] = sideSf2;
    configuredSideSf[2] = sideSf3;

    // 使用 LR2021 的 enableMultiSfRx 函数
    radio.enableMultiSfRx(
        mainSf,
        sideSf1,
        sideSf2,
        sideSf3,
        getBwCode(currentLoRaParams.bandwidth),
        getCrCode(currentLoRaParams.coding_rate)
    );

    DEBUG_PRINTF("Multi-SF RX enabled: Main=SF%d, Side1=SF%d, Side2=SF%d, Side3=SF%d\n",
                 mainSf,
                 sideSf1 ? sideSf1 : 0,
                 sideSf2 ? sideSf2 : 0,
                 sideSf3 ? sideSf3 : 0);
}

// 根据 detector 标志获取实际的 SF 值
uint8_t getSfFromDetector(uint8_t detector) {
    if (detector & LR2021_DETECTOR_MAIN) {
        return configuredMainSf;
    } else if (detector & LR2021_DETECTOR_SIDE1) {
        return configuredSideSf[0];
    } else if (detector & LR2021_DETECTOR_SIDE2) {
        return configuredSideSf[1];
    } else if (detector & LR2021_DETECTOR_SIDE3) {
        return configuredSideSf[2];
    }
    return configuredMainSf;  // 默认返回主 SF
}

// 异步接收任务(在独立的核心上运行)
void rxTask(void* parameter) {
    DEBUG_PRINTLN(F("RX Task started on core 0"));
    rxTaskRunning = true;

    while (rxTaskRunning) {
        // 使用非阻塞方式检查 IRQ
        uint32_t irq = radio.getAndClearIrq();

        if (irq & LR2021_IRQ_RX_DONE) {
            // 获取包长度
            uint16_t len = radio.getRxPacketLength();

            // 读取数据
            static uint8_t data[256];
            if (len > sizeof(data)) {
                len = sizeof(data);
            }
            radio.readFifo(data, len);

            // 获取包状态
            lr2021_lora_packet_status_t status;
            radio.getLoraPacketStatus(&status);

            // 创建接收数据包
            ReceivedPacket rxPacket;
            rxPacket.rssi = status.rssiPkt;
            rxPacket.snr = status.snrPkt;
            rxPacket.detector_id = status.detector;
            rxPacket.sf = getSfFromDetector(status.detector);

            // 检查 CRC
            if (irq & LR2021_IRQ_CRC_ERROR) {
                rxPacket.crc_status = 0;  // CRC 错误
                DEBUG_PRINT(F("[RX] Packet with CRC error: "));
            } else {
                rxPacket.crc_status = 1;  // CRC 正确
                DEBUG_PRINT(F("[RX] Packet received: "));
            }

            rxPacket.data.assign(data, data + len);

            DEBUG_PRINT(len);
            DEBUG_PRINT(F(" bytes, RSSI: "));
            DEBUG_PRINT(rxPacket.rssi);
            DEBUG_PRINT(F(" dBm, SNR: "));
            DEBUG_PRINT(rxPacket.snr);
            DEBUG_PRINT(F(" dB, SF: "));
            DEBUG_PRINT(rxPacket.sf);
            DEBUG_PRINT(F(", Detector: 0x"));
            DEBUG_PRINTLN(rxPacket.detector_id, HEX);

            // 添加到队列(线程安全)
            // 如果队列满，等待直到有空间（而不是丢弃）
            bool added = false;
            while (!added && rxTaskRunning) {
                if (xSemaphoreTake(rxQueueMutex, 50 / portTICK_PERIOD_MS) == pdTRUE) {
                    if (rxQueue.size() < MAX_RX_QUEUE_SIZE) {
                        rxQueue.push(rxPacket);
                        added = true;
                    }
                    xSemaphoreGive(rxQueueMutex);
                }
                if (!added) {
                    // 队列满，等待一下让 processRxQueue 有机会消费
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                }
            }

            // 清空 FIFO 并重新开始接收
            radio.clearRxFifo();
            if (rxTaskRunning) {
                radio.receive(0xFFFFFF);  // 继续接收
            }
        }

        if (irq & LR2021_IRQ_TIMEOUT) {
            DEBUG_PRINTLN(F("[RX] Timeout, restarting receive"));
            if (rxTaskRunning) {
                radio.receive(0xFFFFFF);
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
    // 增加锁等待时间到 100ms，确保能拿到锁
    if (xSemaphoreTake(rxQueueMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
        while (!rxQueue.empty()) {
            ReceivedPacket rxPacket = rxQueue.front();
            rxQueue.pop();

            // 创建 RxDataPacket (新格式: 18 字节头 + 数据)
            // rssi(4) + snr(4) + len(4) + crc_status(4) + sf(1) + detector_id(1) + data(N)
            size_t totalSize = sizeof(RxDataPacket) + rxPacket.data.size();
            uint8_t* buffer = new uint8_t[totalSize];

            RxDataPacket* packet = reinterpret_cast<RxDataPacket*>(buffer);
            packet->rssi = rxPacket.rssi;
            packet->snr = rxPacket.snr;
            packet->len = rxPacket.data.size();
            packet->crc_status = rxPacket.crc_status;
            packet->sf = rxPacket.sf;
            packet->detector_id = rxPacket.detector_id;
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
    if (digitalRead(TX_INDICATOR_PIN) == HIGH) {
        // 如果 TX_INDICATOR_PIN 为高电平，表示设备支持 TX
        return static_cast<uint16_t>(NodeCapability::CAP_TX) |
           static_cast<uint16_t>(NodeCapability::CAP_WIDE_BAND);
    } else {
        // 否则表示设备仅支持 RX
        return static_cast<uint16_t>(NodeCapability::CAP_RX) |
               static_cast<uint16_t>(NodeCapability::CAP_MULTI_SF_RX) |
               static_cast<uint16_t>(NodeCapability::CAP_WIDE_BAND);
    }
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
                DEBUG_PRINTLN(F("TX mode stopped"));
            }
            // 如果是 NOT_SET 模式,不需要清理
        }

        // 第二步:进入目标模式（无论模式是否改变都需要重新初始化）
        if (newMode == NodeWorkingMode::RX) {
            // 进入接收模式
            digitalWrite(TX_LED, LOW); // 熄灭LED表示进入RX模式

            // 设置多 SF 接收（默认启用）
            setupMultiSfRx(currentLoRaParams.spreading_factor);

            // 设置 syncword
            radio.setLoraSyncword(currentLoRaParams.sync_word);

            // 设置包参数
            radio.setLoraPacketParams(
                currentLoRaParams.preamble_length,
                0,  // 0 表示可变长度
                LR2021_LORA_HEADER_EXPLICIT,
                LR2021_LORA_CRC_ON,
                LR2021_LORA_IQ_STANDARD
            );

            // 配置 DIO9 为 IRQ 输出
            radio.setDioIrqConfig(9, LR2021_IRQ_RX_DONE | LR2021_IRQ_CRC_ERROR | LR2021_IRQ_TIMEOUT);

            // 启动接收
            radio.clearRxFifo();
            radio.receive(0xFFFFFF);  // 持续接收

            DEBUG_PRINTLN(F("Starting RX mode with multi-SF"));

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

        } else if (newMode == NodeWorkingMode::TX) {
            // 进入发送模式
            // 配置 PA
            uint32_t freqHz = (uint32_t)(currentLoRaParams.frequency * 1000000);
            radio.configurePaForFrequency(freqHz, currentLoRaParams.tx_power);

            DEBUG_PRINTLN(F("Entered TX mode"));
            digitalWrite(TX_LED, HIGH); // 点亮LED表示进入TX模式

        } else if (newMode == NodeWorkingMode::NOT_SET) {
            // 进入未设置模式
            radio.setStandby();
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

// 处理发送数据命令（新格式: len(4) + target_sf(1) + data(N)）
// 收到数据后立即发送，不进行聚合
void handleTxData(const PacketDecoder::DecodedPacket& packet) {
    if (currentMode != NodeWorkingMode::TX) {
        DEBUG_PRINTLN(F("Error: Not in TX mode"));
        return;
    }

    // 新格式: TxDataPacket 包含 len(4) + target_sf(1) + data[]
    if (packet.data.size() >= sizeof(int) + sizeof(uint8_t)) {
        const TxDataPacket* txData =
            reinterpret_cast<const TxDataPacket*>(packet.data.data());

        size_t actualDataSize = packet.data.size() - sizeof(int) - sizeof(uint8_t);
        uint8_t targetSf = txData->target_sf;

        if (actualDataSize > 0 && txData->len > 0) {
            const uint8_t* dataPtr = packet.data.data() + sizeof(int) + sizeof(uint8_t);
            size_t dataToSend = (actualDataSize < (size_t)txData->len) ? actualDataSize : txData->len;

            // 检查数据大小
            if (dataToSend > TX_BUFFER_SIZE) {
                DEBUG_PRINTLN(F("Error: Data too large to send"));
                return;
            }

            // 如果指定了目标 SF，且与当前 SF 不同，则临时切换
            bool sfChanged = false;
            uint8_t originalSf = currentLoRaParams.spreading_factor;

            if (targetSf != 0 && targetSf != originalSf) {
                DEBUG_PRINTF("Switching SF from %d to %d for TX\n", originalSf, targetSf);
                radio.setLoraModulationParams(
                    targetSf,
                    getBwCode(currentLoRaParams.bandwidth),
                    getCrCode(currentLoRaParams.coding_rate),
                    getLdroSetting(targetSf, currentLoRaParams.bandwidth)
                );
                sfChanged = true;
            }

            // 立即发送数据
            bool success = radio.transmitBlocking(const_cast<uint8_t*>(dataPtr), dataToSend, 5000);

            if (success) {
                DEBUG_PRINT(F("Transmitted: "));
                DEBUG_PRINT(dataToSend);
                DEBUG_PRINTLN(F(" bytes successfully"));
            } else {
                DEBUG_PRINTLN(F("Transmission failed"));
            }

            // 恢复原来的 SF
            if (sfChanged) {
                DEBUG_PRINTF("Restoring SF to %d\n", originalSf);
                radio.setLoraModulationParams(
                    originalSf,
                    getBwCode(currentLoRaParams.bandwidth),
                    getCrCode(currentLoRaParams.coding_rate),
                    getLdroSetting(originalSf, currentLoRaParams.bandwidth)
                );
            }
        }
    }
}

// 配置LoRa参数
void configureLoRa(const LoRaParam& params) {
    DEBUG_PRINTLN(F("Configuring LoRa parameters..."));

    // 先进入待机模式
    radio.setStandby();

    // 设置频率 (MHz -> Hz)
    uint32_t freqHz = (uint32_t)(params.frequency * 1000000);
    radio.setRfFrequency(freqHz);

    // 根据频率配置 PA 和 RX 路径
    radio.configurePaForFrequency(freqHz, params.tx_power);

    // 设置调制参数
    radio.setLoraModulationParams(
        params.spreading_factor,
        getBwCode(params.bandwidth),
        getCrCode(params.coding_rate),
        getLdroSetting(params.spreading_factor, params.bandwidth)
    );

    // 设置同步字
    radio.setLoraSyncword(params.sync_word);

    // 设置包参数
    radio.setLoraPacketParams(
        params.preamble_length,
        0,  // 0 表示可变长度
        LR2021_LORA_HEADER_EXPLICIT,
        LR2021_LORA_CRC_ON,
        LR2021_LORA_IQ_STANDARD
    );

    DEBUG_PRINTLN(F("LoRa parameters configured"));
    DEBUG_PRINTF("Freq: %.2f MHz, BW: %.2f kHz, SF: %d, CR: 4/%d, Power: %d dBm\n",
                 params.frequency, params.bandwidth, params.spreading_factor,
                 params.coding_rate, params.tx_power);
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
