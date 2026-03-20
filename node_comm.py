#!/usr/bin/env python3
"""
LoRa 节点通信模块 - 支持 LR1262 和 LR2021

通过串口与 LoRa 节点通信，实现帧编解码和命令收发。
自动探测波特率和型号，屏蔽两种节点的协议差异。
"""

import struct
import time
import threading
from enum import IntEnum

import serial
import serial.tools.list_ports


# ============================================================
#  协议常量（两种型号共用）
# ============================================================

FRAME_DELIMITER = 0x7E
MAX_PACKET_SIZE = 512

BAUD_RATES = {
    "LR2021": 115200,
    "LR1262": 921600,
}
BAUD_RATE_PROBE_ORDER = [115200, 921600]


class PacketType:
    CMD_QUERY_INFO = 0x01
    CMD_SET_LORA_PARAM = 0x02
    TX_DATA = 0x03
    RX_DATA = 0x04
    RET_NODE_INFO = 0x05
    CMD_QUERY_CAPS = 0x06
    RET_NODE_CAPS = 0x07

    VALID_TYPES = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07}


class NodeWorkingMode:
    TX = 0x01
    RX = 0x02
    NOT_SET = 0xFF


class NodeCapability:
    CAP_TX = 0x0001
    CAP_RX = 0x0002
    CAP_FULL_DUPLEX = 0x0004
    CAP_FREQUENCY_HOP = 0x0008
    CAP_HIGH_POWER = 0x0010
    CAP_LBT = 0x0020
    CAP_MULTI_SF_RX = 0x0040
    CAP_WIDE_BAND = 0x0080


# ============================================================
#  结构体格式（匹配 Packets.h  #pragma pack(push, 1)）
# ============================================================

# 共用
LORA_PARAM_FMT = "<ffBBBbH"  # 14 bytes
SET_LORA_PARAM_FMT = "<BffBBBbH"  # 15 bytes  (mode + LoRaParam)
NODE_CAPS_FMT = "<6sHBBBB16s"  # 28 bytes
QUERY_INFO_FMT = "<6sBffBBBbH"  # 21 bytes

# LR1262 专用
TX_DATA_HEADER_FMT_1262 = "<i"  # 4 bytes  (len)
RX_DATA_HEADER_FMT_1262 = "<ffii"  # 16 bytes (rssi+snr+len+crc)
RX_DATA_HEADER_SIZE_1262 = struct.calcsize(RX_DATA_HEADER_FMT_1262)

# LR2021 专用
TX_DATA_HEADER_FMT_2021 = "<iB"  # 5 bytes  (len+target_sf)
RX_DATA_HEADER_FMT_2021 = "<ffiiBB"  # 18 bytes (rssi+snr+len+crc+sf+detector_id)
RX_DATA_HEADER_SIZE_2021 = struct.calcsize(RX_DATA_HEADER_FMT_2021)


# ============================================================
#  帧编码器
# ============================================================

class FrameEncoder:
    """将 (packet_type, payload) 编码为串口帧字节流"""

    @staticmethod
    def encode(packet_type: int, payload: bytes = b"") -> bytes:
        frame = bytearray()
        frame.append(FRAME_DELIMITER)
        frame.append(packet_type & 0xFF)
        frame.extend(struct.pack("<I", len(payload)))
        frame.extend(payload)
        frame.append(FRAME_DELIMITER)
        return bytes(frame)


# ============================================================
#  帧解码器（状态机，复刻 SerialDecoder.cpp）
# ============================================================

class _DecodeState(IntEnum):
    WAIT_START = 0
    READ_TYPE = 1
    READ_LENGTH = 2
    READ_DATA = 3
    WAIT_END = 4


class FrameDecoder:
    """
    串口帧解码状态机。

    调用 feed(data) 喂入原始字节，返回已解码的帧列表。
    每个帧为 (packet_type: int, payload: bytes)。
    """

    def __init__(self):
        self._reset()

    def _reset(self):
        self._state = _DecodeState.WAIT_START
        self._buffer = bytearray()
        self._packet_type = 0
        self._expected_len = 0
        self._len_bytes_read = 0
        self._len_accum = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        """喂入原始字节流，返回已完成解码的 [(packet_type, payload), ...]"""
        decoded = []

        for byte in data:
            if self._state == _DecodeState.WAIT_START:
                if byte == FRAME_DELIMITER:
                    self._state = _DecodeState.READ_TYPE
                    self._buffer.clear()
                    self._len_bytes_read = 0
                    self._len_accum = 0

            elif self._state == _DecodeState.READ_TYPE:
                if byte == FRAME_DELIMITER:
                    # 连续起始标志 → 重新等待类型
                    self._reset()
                    self._state = _DecodeState.READ_TYPE
                elif byte in PacketType.VALID_TYPES:
                    self._packet_type = byte
                    self._state = _DecodeState.READ_LENGTH
                    self._len_accum = 0
                    self._len_bytes_read = 0
                else:
                    self._reset()

            elif self._state == _DecodeState.READ_LENGTH:
                if byte == FRAME_DELIMITER:
                    self._reset()
                    self._state = _DecodeState.READ_TYPE
                else:
                    self._len_accum |= byte << (self._len_bytes_read * 8)
                    self._len_bytes_read += 1
                    if self._len_bytes_read == 4:
                        self._expected_len = self._len_accum
                        if self._expected_len > MAX_PACKET_SIZE:
                            self._reset()
                        elif self._expected_len == 0:
                            self._state = _DecodeState.WAIT_END
                        else:
                            self._state = _DecodeState.READ_DATA
                            self._buffer = bytearray()

            elif self._state == _DecodeState.READ_DATA:
                # 在 READ_DATA 状态下，所有字节（包括 0x7E）都是 payload 的一部分
                # 因为我们已经知道要读取 _expected_len 字节
                self._buffer.append(byte)
                if len(self._buffer) == self._expected_len:
                    self._state = _DecodeState.WAIT_END

            elif self._state == _DecodeState.WAIT_END:
                if byte == FRAME_DELIMITER:
                    if self._expected_len == 0:
                        decoded.append((self._packet_type, b""))
                    else:
                        decoded.append((self._packet_type, bytes(self._buffer)))
                    self._reset()
                else:
                    self._reset()

        return decoded


# ============================================================
#  打包 / 解包辅助函数
# ============================================================

def pack_set_lora_param(
    mode: int,
    frequency: float,
    bandwidth: float,
    spreading_factor: int,
    coding_rate: int,
    sync_word: int,
    tx_power: int,
    preamble_length: int,
) -> bytes:
    """打包 SetLoRaParamPayload (15 bytes)"""
    return struct.pack(
        SET_LORA_PARAM_FMT,
        mode,
        frequency,
        bandwidth,
        spreading_factor,
        coding_rate,
        sync_word,
        tx_power,
        preamble_length,
    )


def pack_tx_data_1262(data: bytes) -> bytes:
    """打包 LR1262 的 TxDataPacket: len(4) + data"""
    return struct.pack(TX_DATA_HEADER_FMT_1262, len(data)) + data


def pack_tx_data_2021(data: bytes, target_sf: int = 0) -> bytes:
    """打包 LR2021 的 TxDataPacket: len(4) + target_sf(1) + data"""
    return struct.pack(TX_DATA_HEADER_FMT_2021, len(data), target_sf) + data


def parse_rx_data_1262(payload: bytes) -> dict:
    """解析 LR1262 的 RxDataPacket"""
    hdr_size = RX_DATA_HEADER_SIZE_1262
    rssi, snr, length, crc_status = struct.unpack(
        RX_DATA_HEADER_FMT_1262, payload[:hdr_size]
    )
    data = payload[hdr_size : hdr_size + length]
    return {
        "rssi": rssi,
        "snr": snr,
        "len": length,
        "crc_status": crc_status,
        "data": data,
    }


def parse_rx_data_2021(payload: bytes) -> dict:
    """解析 LR2021 的 RxDataPacket"""
    hdr_size = RX_DATA_HEADER_SIZE_2021
    rssi, snr, length, crc_status, sf, detector_id = struct.unpack(
        RX_DATA_HEADER_FMT_2021, payload[:hdr_size]
    )
    data = payload[hdr_size : hdr_size + length]
    return {
        "rssi": rssi,
        "snr": snr,
        "len": length,
        "crc_status": crc_status,
        "sf": sf,
        "detector_id": detector_id,
        "data": data,
    }


def parse_node_caps(payload: bytes) -> dict:
    """解析 NodeCapsPayload (28 bytes)"""
    node_id, caps, fw_major, fw_minor, fw_patch, hw_rev, model_raw = struct.unpack(
        NODE_CAPS_FMT, payload[:struct.calcsize(NODE_CAPS_FMT)]
    )
    model_name = model_raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
    return {
        "node_id": node_id.hex().upper(),
        "capabilities": caps,
        "firmware": f"{fw_major}.{fw_minor}.{fw_patch}",
        "hardware_revision": hw_rev,
        "model_name": model_name,
    }


def parse_node_info(payload: bytes) -> dict:
    """解析 CMDQueryInfoPayload (21 bytes)"""
    fmt = QUERY_INFO_FMT
    (
        node_id,
        mode,
        freq,
        bw,
        sf,
        cr,
        sw,
        power,
        preamble,
    ) = struct.unpack(fmt, payload[: struct.calcsize(fmt)])
    return {
        "node_id": node_id.hex().upper(),
        "mode": mode,
        "frequency": freq,
        "bandwidth": bw,
        "spreading_factor": sf,
        "coding_rate": cr,
        "sync_word": sw,
        "tx_power": power,
        "preamble_length": preamble,
    }


# ============================================================
#  LoRa 空中时间计算
# ============================================================

def calc_time_on_air(
    sf: int,
    bw_khz: float,
    payload_len: int,
    preamble: int = 8,
    cr: int = 1,
    crc: bool = True,
    explicit_header: bool = True,
) -> float:
    """
    计算 LoRa 包的空中时间 (秒)。

    参数:
        sf:           扩频因子 (5-12)
        bw_khz:       带宽 (kHz)
        payload_len:  负载字节数
        preamble:     前导码符号数
        cr:           编码率分子减 4 (1=4/5, 2=4/6, 3=4/7, 4=4/8)
        crc:          是否启用 CRC
        explicit_header: 是否使用显式头
    """
    t_sym = (2**sf) / (bw_khz * 1000)  # 符号时间 (秒)
    t_preamble = (preamble + 4.25) * t_sym

    # 低速率优化
    # LDRO ON: SF11+BW125, SF12+BW125/250
    # LDRO OFF: 其他情况
    if sf == 11 and bw_khz <= 125:
        de = 1
    elif sf == 12 and bw_khz <= 250:
        de = 1
    else:
        de = 0
    h = 0 if explicit_header else 1
    crc_bits = 16 if crc else 0

    numerator = 8 * payload_len - 4 * sf + 28 + crc_bits - 20 * h
    denominator = 4 * (sf - 2 * de)

    if denominator <= 0:
        # SF5 等低 SF 防止除零
        payload_symbols = 8
    else:
        import math

        payload_symbols = 8 + max(math.ceil(numerator / denominator) * (cr + 4), 0)

    t_payload = payload_symbols * t_sym
    return t_preamble + t_payload + 0.1


# ============================================================
#  LoRaModule 类
# ============================================================

class LoRaModule:
    """
    LoRa 节点通信封装。

    自动探测波特率，自动识别型号 (LR1262 / LR2021)，
    屏蔽两种型号在 TxDataPacket / RxDataPacket 上的差异。
    """

    MODEL_LR1262 = "LR1262"
    MODEL_LR2021 = "LR2021"

    def __init__(self, port: str, name: str = "Module"):
        self.port = port
        self.name = name
        self.model: str | None = None  # "LR1262" | "LR2021"
        self.node_id: str = ""
        self.capabilities: int = 0
        self.firmware: str = ""
        self.ser: serial.Serial | None = None
        self.decoder = FrameDecoder()

        # RX 收集器
        self._rx_lock = threading.Lock()
        self._rx_packets: list[dict] = []
        self._rx_thread: threading.Thread | None = None
        self._rx_running = False

        # 自动连接并识别
        self._auto_connect()

    # ---- 连接 & 识别 ----

    def _auto_connect(self):
        """尝试不同波特率连接，通过 CMD_QUERY_CAPS 识别型号"""
        for baud in BAUD_RATE_PROBE_ORDER:
            try:
                self.ser = serial.Serial(self.port, baud, timeout=0.5)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                time.sleep(0.1)

                # 尝试查询能力
                caps = self._try_query_caps(timeout=2.0)
                if caps is not None:
                    self._identify_model(caps)
                    print(
                        f"  [{self.name}] 连接成功: {self.port} @ {baud} baud"
                    )
                    print(
                        f"  [{self.name}] 型号={self.model}, "
                        f"ID={self.node_id}, "
                        f"固件={self.firmware}"
                    )
                    return
                else:
                    self.ser.close()
            except serial.SerialException:
                if self.ser and self.ser.is_open:
                    self.ser.close()

        raise ConnectionError(
            f"无法连接到 {self.port}，已尝试波特率 {BAUD_RATE_PROBE_ORDER}"
        )

    def _try_query_caps(self, timeout: float = 2.0) -> dict | None:
        """发送 CMD_QUERY_CAPS 并等待 RET_NODE_CAPS 响应"""
        frame = FrameEncoder.encode(PacketType.CMD_QUERY_CAPS)
        self.ser.write(frame)

        result = self._read_specific_frame(PacketType.RET_NODE_CAPS, timeout)
        if result is not None:
            return parse_node_caps(result)
        return None

    def _identify_model(self, caps: dict):
        """根据 QUERY_CAPS 响应识别型号"""
        self.node_id = caps["node_id"]
        self.capabilities = caps["capabilities"]
        self.firmware = caps["firmware"]

        model_name = caps["model_name"]
        if "RFUnit" in model_name or "2021" in model_name or "LR2021" in model_name:
            self.model = self.MODEL_LR2021
        elif "AeroLink" in model_name or "1262" in model_name or "Node-S" in model_name:
            self.model = self.MODEL_LR1262
        else:
            # 通过能力标志辅助判断
            if self.capabilities & NodeCapability.CAP_MULTI_SF_RX:
                self.model = self.MODEL_LR2021
            else:
                self.model = self.MODEL_LR1262

    # ---- 命令接口 ----

    def query_caps(self) -> dict:
        """查询设备能力"""
        frame = FrameEncoder.encode(PacketType.CMD_QUERY_CAPS)
        self.ser.write(frame)
        payload = self._read_specific_frame(PacketType.RET_NODE_CAPS, timeout=2.0)
        if payload is None:
            raise TimeoutError(f"[{self.name}] CMD_QUERY_CAPS 超时")
        return parse_node_caps(payload)

    def query_info(self) -> dict:
        """查询节点当前信息"""
        frame = FrameEncoder.encode(PacketType.CMD_QUERY_INFO)
        self.ser.write(frame)
        payload = self._read_specific_frame(PacketType.RET_NODE_INFO, timeout=2.0)
        if payload is None:
            raise TimeoutError(f"[{self.name}] CMD_QUERY_INFO 超时")
        return parse_node_info(payload)

    def set_lora_params(
        self,
        mode: int,
        frequency: float,
        bandwidth: float,
        spreading_factor: int,
        coding_rate: int = 5,
        sync_word: int = 0x12,
        tx_power: int = 10,
        preamble_length: int = 8,
    ):
        """配置 LoRa 参数并设置工作模式"""
        payload = pack_set_lora_param(
            mode, frequency, bandwidth, spreading_factor,
            coding_rate, sync_word, tx_power, preamble_length,
        )
        frame = FrameEncoder.encode(PacketType.CMD_SET_LORA_PARAM, payload)
        self.ser.write(frame)
        # 给模组一点时间切换模式
        time.sleep(0.05)

    def send_tx_data(self, data: bytes, target_sf: int = 0):
        """
        发送 TX 数据包。

        LR1262: 忽略 target_sf
        LR2021: 包含 target_sf 字段
        """
        if self.model == self.MODEL_LR1262:
            payload = pack_tx_data_1262(data)
        else:
            payload = pack_tx_data_2021(data, target_sf)
        frame = FrameEncoder.encode(PacketType.TX_DATA, payload)
        self.ser.write(frame)

    def parse_rx_data(self, payload: bytes) -> dict:
        """根据型号解析 RX_DATA 负载"""
        if self.model == self.MODEL_LR1262:
            return parse_rx_data_1262(payload)
        else:
            return parse_rx_data_2021(payload)

    # ---- 帧读取 ----

    def read_frame(self, timeout: float = 2.0) -> tuple[int, bytes] | None:
        """
        从串口读取并解码一帧。

        返回 (packet_type, payload) 或超时返回 None。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw = self.ser.read(waiting)
                frames = self.decoder.feed(raw)
                if frames:
                    return frames[0]
            else:
                time.sleep(0.005)
        return None

    def _read_specific_frame(
        self, expected_type: int, timeout: float = 2.0
    ) -> bytes | None:
        """读取指定类型的帧，丢弃其他帧"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            result = self.read_frame(timeout=remaining)
            if result is None:
                return None
            pkt_type, payload = result
            if pkt_type == expected_type:
                return payload
            # 不是目标类型 → 继续读
        return None

    # ---- RX 收集器（后台线程） ----

    def start_rx_collector(self):
        """启动后台线程持续收集 RX_DATA 包"""
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return  # 已在运行

        with self._rx_lock:
            self._rx_packets.clear()
        self._rx_running = True
        self._rx_thread = threading.Thread(
            target=self._rx_collector_loop, daemon=True
        )
        self._rx_thread.start()

    def stop_rx_collector(self) -> list[dict]:
        """停止收集器，返回收集到的所有 RX 包"""
        self._rx_running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=3.0)
            self._rx_thread = None

        with self._rx_lock:
            packets = list(self._rx_packets)
            self._rx_packets.clear()
        return packets

    def _rx_collector_loop(self):
        """后台收集线程主循环"""
        while self._rx_running:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw = self.ser.read(waiting)
                frames = self.decoder.feed(raw)
                for pkt_type, payload in frames:
                    if pkt_type == PacketType.RX_DATA:
                        try:
                            parsed = self.parse_rx_data(payload)
                            with self._rx_lock:
                                self._rx_packets.append(parsed)
                        except Exception:
                            pass  # 解析失败 → 跳过
            else:
                time.sleep(0.002)

    # ---- 生命周期 ----

    def close(self):
        """停止收集器并关闭串口"""
        self._rx_running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __repr__(self):
        return (
            f"LoRaModule(port={self.port!r}, name={self.name!r}, "
            f"model={self.model!r}, id={self.node_id})"
        )


# ============================================================
#  工具函数
# ============================================================

def list_serial_ports() -> list[dict]:
    """枚举所有可用串口，返回 [{port, description, hwid}, ...]"""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            "port": p.device,
            "description": p.description,
            "hwid": p.hwid,
        })
    return sorted(ports, key=lambda x: x["port"])


def select_port(prompt: str = "请选择串口") -> str:
    """交互式选择串口"""
    ports = list_serial_ports()
    if not ports:
        raise RuntimeError("未发现可用串口")

    print(f"\n{prompt}:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p['port']}  -  {p['description']}")

    while True:
        try:
            choice = input(f"输入编号 (0-{len(ports)-1}): ").strip()
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx]["port"]
        except (ValueError, EOFError):
            pass
        print("无效输入，请重试")
