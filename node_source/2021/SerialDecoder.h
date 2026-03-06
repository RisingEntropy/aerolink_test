#ifndef SERIAL_DECODER_H
#define SERIAL_DECODER_H

#ifndef PACKET_DECODER_H
#define PACKET_DECODER_H

#include "Packets.h"
#include <cstring>
#include <vector>
#include <memory>

class PacketDecoder {
public:
    // 解码结果结构，包含DataPacket及其数据
    struct DecodedPacket {
        PacketType type;
        uint32_t length;
        std::vector<uint8_t> data;

        DecodedPacket(PacketType t, uint32_t len, const uint8_t* d, size_t size)
            : type(t), length(len), data(d, d + size) {}
    };

private:
    // 解码状态机的状态
    enum class DecodeState {
        WAIT_START,      // 等待起始标志 0x7E
        READ_TYPE,       // 读取包类型
        READ_LENGTH,     // 读取长度字段 (4字节)
        READ_DATA,       // 读取数据内容
        WAIT_END         // 等待结束标志 0x7E
    };

    static constexpr uint8_t FRAME_DELIMITER = 0x7E;
    static constexpr size_t MAX_PACKET_SIZE = 512; // 最大包大小限制

    DecodeState state_ = DecodeState::WAIT_START;
    std::vector<uint8_t> buffer_;
    PacketType currentPacketType_;
    uint32_t currentLength_ = 0;
    size_t lengthBytesRead_ = 0;

    void reset();

public:
    PacketDecoder() = default;

    // 主解码函数，输入数据流，返回所有成功解码的数据包
    std::vector<DecodedPacket> decode(const uint8_t* data, size_t length);

    // 获取当前状态（用于调试）
    const char* getCurrentStateName() const;

    // 强制重置解码器状态
    void forceReset();
};

#endif // PACKET_DECODER_H
#endif // SERIAL_DECODER_H
