#ifndef PACKETS_H
#define PACKETS_H
#include <stdint.h>

// 使用 enum class 增强类型安全
enum class PacketType : uint8_t {
    CMD_QUERY_INFO = 0x01,
    CMD_SET_LORA_PARAM = 0x02,
    TX_DATA = 0x03,
    RX_DATA = 0x04,
    RET_NODE_INFO = 0x05,
    CMD_QUERY_CAPS = 0x06,  // 查询设备能力
    RET_NODE_CAPS = 0x07,   // 返回设备能力
};

enum class NodeWorkingMode : uint8_t {
    TX = 0x01,
    RX = 0x02,
    NOT_SET = 0xFF,
};

// 设备能力标志位 (位掩码)
enum class NodeCapability : uint16_t {
    CAP_TX           = 0x0001,  // 支持发送
    CAP_RX           = 0x0002,  // 支持接收
    CAP_FULL_DUPLEX  = 0x0004,  // 支持全双工 (同时收发)
    CAP_FREQUENCY_HOP= 0x0008,  // 支持跳频
    CAP_HIGH_POWER   = 0x0010,  // 支持高功率输出 (>20dBm)
    CAP_LBT          = 0x0020,  // 支持 Listen Before Talk
    CAP_MULTI_SF_RX  = 0x0040,  // 支持多 SF 并行接收
    CAP_WIDE_BAND    = 0x0080,  // 支持宽频段 (150MHz-2.5GHz)
};

// 使用 push 和 pop 将所有需要打包的结构体包围起来
#pragma pack(push, 1)

// LoRa参数的核心定义，可被其他结构体重用
struct LoRaParam {
    float frequency;
    float bandwidth;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    uint8_t sync_word;
    int8_t tx_power;
    uint16_t preamble_length;
};
// 验证结构体大小，确保没有意外的填充
static_assert(sizeof(LoRaParam) == 14, "LoRaParam size is not 14 bytes");

// --- Packet Payload Definitions ---

// 设置LoRa参数的命令载荷，通过组合重用LoRaParam
struct SetLoRaParamPayload {
    NodeWorkingMode node_working_mode;
    LoRaParam params; // 重用LoRaParam定义
};
static_assert(sizeof(SetLoRaParamPayload) == sizeof(NodeWorkingMode) + sizeof(LoRaParam), "SetLoRaParamPayload size is incorrect");

// 查询节点信息的命令载荷
struct CMDQueryInfoPayload {
    uint8_t node_id[6];
    NodeWorkingMode node_working_mode;
    LoRaParam node_info;
};

// 能力响应载荷 (RET_NODE_CAPS)
struct NodeCapsPayload {
    uint8_t node_id[6];         // 设备 ID
    uint16_t capabilities;       // 能力位掩码
    uint8_t firmware_major;      // 固件主版本
    uint8_t firmware_minor;      // 固件次版本
    uint8_t firmware_patch;      // 固件补丁版本
    uint8_t hardware_revision;   // 硬件版本
    char model_name[16];         // 设备型号 (空字符结尾)
};
static_assert(sizeof(NodeCapsPayload) == 28, "NodeCapsPayload size is not 28 bytes");

// TX数据包的元信息 (新格式: 带 SF)
// 格式: len(4) + target_sf(1) + data(N)
struct TxDataPacket {
    int len;
    uint8_t target_sf;  // 目标 SF, 0 表示使用默认 SF
    uint8_t data[];
};

// RX数据包的元信息 (新格式: 带 SF 和 detector_id)
// 格式: rssi(4) + snr(4) + len(4) + crc_status(4) + sf(1) + detector_id(1) + data(N)
struct RxDataPacket {
    float rssi;
    float snr;
    int len;
    int crc_status;
    uint8_t sf;           // 接收时的 SF
    uint8_t detector_id;  // 检测器 ID (main=1, side1=2, side2=4, side3=8)
    uint8_t data[];
};

#pragma pack(pop) // 恢复默认的对齐方式

#endif // PACKETS_H
