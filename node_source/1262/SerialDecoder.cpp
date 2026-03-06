#include "SerialDecoder.h"

void PacketDecoder::reset() {
    state_ = DecodeState::WAIT_START;
    buffer_.clear();
    currentLength_ = 0;
    lengthBytesRead_ = 0;
}

std::vector<PacketDecoder::DecodedPacket> PacketDecoder::decode(const uint8_t* data, size_t length) {
    std::vector<DecodedPacket> decodedPackets;
    
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        
        switch (state_) {
        case DecodeState::WAIT_START:
            if (byte == FRAME_DELIMITER) {
                state_ = DecodeState::READ_TYPE;
                buffer_.clear();
                lengthBytesRead_ = 0;
            }
            break;
            
        case DecodeState::READ_TYPE:
            if (byte == FRAME_DELIMITER) {
                // 连续的起始标志,重新开始
                reset();
                state_ = DecodeState::READ_TYPE;
            } else {
                // 验证包类型是否有效
                uint8_t typeValue = byte;
                if (typeValue == static_cast<uint8_t>(PacketType::CMD_QUERY_INFO) ||
                    typeValue == static_cast<uint8_t>(PacketType::CMD_SET_LORA_PARAM) ||
                    typeValue == static_cast<uint8_t>(PacketType::TX_DATA) ||
                    typeValue == static_cast<uint8_t>(PacketType::RX_DATA) ||
                    typeValue == static_cast<uint8_t>(PacketType::CMD_QUERY_CAPS) ||
                    typeValue == static_cast<uint8_t>(PacketType::RET_NODE_CAPS)) {
                    currentPacketType_ = static_cast<PacketType>(typeValue);
                    state_ = DecodeState::READ_LENGTH;
                    currentLength_ = 0;
                    lengthBytesRead_ = 0;
                } else {
                    // 无效的包类型,重置
                    reset();
                }
            }
            break;
            
        case DecodeState::READ_LENGTH:
            if (byte == FRAME_DELIMITER) {
                // 意外的起始标志,重新开始
                reset();
                state_ = DecodeState::READ_TYPE;
            } else {
                // 小端序读取32位长度
                currentLength_ |= (static_cast<uint32_t>(byte) << (lengthBytesRead_ * 8));
                lengthBytesRead_++;
                
                if (lengthBytesRead_ == 4) {
                    if (currentLength_ > MAX_PACKET_SIZE) {
                        // 包太大,重置
                        reset();
                    } else if (currentLength_ == 0) {
                        // 空数据包,等待结束标志
                        state_ = DecodeState::WAIT_END;
                    } else {
                        state_ = DecodeState::READ_DATA;
                        buffer_.clear();
                        buffer_.reserve(currentLength_);
                    }
                }
            }
            break;
            
        case DecodeState::READ_DATA:
            // 在 READ_DATA 状态下，所有字节（包括 0x7E）都是 payload 的一部分
            // 因为我们已经知道要读取 currentLength_ 字节
            buffer_.push_back(byte);
            if (buffer_.size() == currentLength_) {
                state_ = DecodeState::WAIT_END;
            }
            break;
            
        case DecodeState::WAIT_END:
            if (byte == FRAME_DELIMITER) {
                // 成功接收到结束标志
                if (currentLength_ == 0) {
                    // 空数据包
                    decodedPackets.emplace_back(currentPacketType_, 0, nullptr, 0);
                } else {
                    // 正常数据包
                    decodedPackets.emplace_back(currentPacketType_, currentLength_,
                                               buffer_.data(), buffer_.size());
                }
                reset();
            } else {
                // 没有正确的结束标志,丢弃当前包
                reset();
                // 检查这个字节是否是新包的开始
                if (byte == FRAME_DELIMITER) {
                    state_ = DecodeState::READ_TYPE;
                }
            }
            break;
        }
    }
    
    return decodedPackets;
}

const char* PacketDecoder::getCurrentStateName() const {
    switch (state_) {
    case DecodeState::WAIT_START: return "WAIT_START";
    case DecodeState::READ_TYPE: return "READ_TYPE";
    case DecodeState::READ_LENGTH: return "READ_LENGTH";
    case DecodeState::READ_DATA: return "READ_DATA";
    case DecodeState::WAIT_END: return "WAIT_END";
    default: return "UNKNOWN";
    }
}

void PacketDecoder::forceReset() {
    reset();
}