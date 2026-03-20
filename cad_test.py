#!/usr/bin/env python3
"""
CAD (Channel Activity Detection) 测试

验证 LR2021 的 CAD 功能：信道活动检测率和误检率。
支持多组 (SF, BW) 组合。

测试结构（每组 SF/BW）:
  Phase 1 - 误检率测试：TX 静默，运行 N 次 CAD 扫描，统计误报
  Phase 2 - 符号数扫描：对 num_symbols ∈ [1,2,4,8,16]，
            各运行 N 次 "TX发包 → CAD扫描" 对，统计检测率和扫描耗时

节点:
  TX  节点: 使用现有 node_comm.py 协议（LR2021 固件）
  CAD 节点: 使用 node_source/CAD_Node/CAD_Node.ino 自定义协议

自定义协议格式:
  帧: [0xCA][type:1][len:1][payload:N][xor_checksum:1][0xCA]
  CMD_PING (0x00): 无 payload → 节点回复 RES_READY（用于连接握手）
  CMD_INIT (0x01): freq_hz(4) + sf(1) + bw_khz(2) + cr(1) + num_symbols(1) + timeout_ms(2)
  CMD_CAD  (0x02): 无 payload
  RES_READY      (0x80): 无 payload
  RES_CAD_RESULT (0x81): detected(1) + timed_out(1) + duration_us(4)
"""

import csv
import json
import signal
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import serial

from node_comm import (
    LoRaModule,
    NodeWorkingMode,
    calc_time_on_air,
    select_port,
)


# ============================================================
#  测试配置
# ============================================================

# 固定 LoRa 参数
FREQUENCY       = 1439.0    # MHz
CODING_RATE     = 5         # 4/5
SYNC_WORD       = 0x12
PREAMBLE_LENGTH = 8
PAYLOAD_SIZE    = 16        # bytes
TX_POWER        = 3         # dBm

# 多组 (SF, BW_kHz) 组合
SF_BW_COMBINATIONS = [
    (7,  500),
    (5,  125),
    (5,  500),
    (7,  125),
    (5,  250),
    (7, 250),
   
]

# 测试规模
N_FALSE_POS     = 300      # 误检率测试：CAD 扫描次数（TX 静默）
N_DETECTION     = 100      # 每个 num_symbols 配置的 TX+CAD 对数
CAD_SYMBOLS_LIST = [1, 2, 4, 8, 16]   # 扫描的符号数列表
CAD_TIMEOUT_MS  = 500      # CAD 单次超时 (ms)

# TX → CAD 间隔
TX_TO_CAD_DELAY = 0.00    # 3 ms

# 输出目录
OUTPUT_DIR  = Path("CAD_Test")
CHARTS_DIR  = OUTPUT_DIR / "charts"


# ============================================================
#  CADNode 类 —— 自定义协议
# ============================================================

class CADNode:
    """与 CAD_Node.ino 通信的客户端，使用自定义二进制协议。"""

    MAGIC          = 0xCA
    CMD_PING       = 0x00  # triggers RES_READY; used at connect time
    CMD_INIT       = 0x01
    CMD_CAD        = 0x02
    RES_READY      = 0x80
    RES_CAD_RESULT = 0x81

    def __init__(self, port: str, baud: int = 115200, name: str = "CAD"):
        self.port = port
        self.name = name
        self._buf = bytearray()

        print(f"  [{name}] 正在连接 {port} @ {baud} baud ...")
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        # 主动发 ping，让节点回复 READY（避免错过节点上电时发出的初始 READY）
        self._send_frame(self.CMD_PING, b"")
        if not self._wait_ready(timeout=5.0):
            raise ConnectionError(f"[{name}] CAD 节点未在 5 秒内响应 PING")
        print(f"  [{name}] 连接成功")

    # ---- 公开接口 ----

    def configure(
        self,
        freq_hz: int,
        sf: int,
        bw_khz: int,
        cr: int,
        num_symbols: int,
        timeout_ms: int = CAD_TIMEOUT_MS,
    ) -> bool:
        """发送 CMD_INIT，等待 RES_READY 确认。"""
        payload = struct.pack(
            "<IBHBBH",
            freq_hz,
            sf,
            bw_khz,
            cr,
            num_symbols,
            timeout_ms,
        )
        self._send_frame(self.CMD_INIT, payload)
        return self._wait_ready(timeout=5.0)

    def do_cad(self, timeout: float = 2.0) -> dict | None:
        """发送 CMD_CAD，等待 RES_CAD_RESULT。返回 None 表示超时。"""
        self._send_frame(self.CMD_CAD, b"")
        return self._wait_cad_result(timeout=timeout)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ---- 帧构造 ----

    def _send_frame(self, cmd: int, payload: bytes):
        checksum = 0
        for b in payload:
            checksum ^= b
        frame = bytearray()
        frame.append(self.MAGIC)
        frame.append(cmd)
        frame.append(len(payload))
        frame.extend(payload)
        frame.append(checksum)
        frame.append(self.MAGIC)
        self.ser.write(bytes(frame))

    # ---- 帧解析 ----

    def _feed_serial(self, timeout: float):
        """从串口读数据到内部缓冲区，最多等待 timeout 秒。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n:
                self._buf.extend(self.ser.read(n))
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.ser.timeout = min(0.02, remaining)
            chunk = self.ser.read(1)
            if chunk:
                self._buf.extend(chunk)
                return
            time.sleep(0.001)

    def _try_parse(self) -> tuple[int, bytes] | None:
        """尝试从缓冲区解析一帧，成功返回 (type, payload)，否则 None。"""
        while len(self._buf) >= 1:
            if self._buf[0] != self.MAGIC:
                self._buf = self._buf[1:]
                continue

            # 帧: MAGIC + type(1) + len(1) + payload(N) + checksum(1) + MAGIC(1)
            if len(self._buf) < 3:
                return None
            frame_len    = self._buf[2]
            total_needed = 5 + frame_len  # 1+1+1+N+1+1

            if len(self._buf) < total_needed:
                return None

            frame_type   = self._buf[1]
            payload      = bytes(self._buf[3 : 3 + frame_len])
            rx_checksum  = self._buf[3 + frame_len]
            end_magic    = self._buf[3 + frame_len + 1]

            self._buf = self._buf[total_needed:]

            expected_cs = 0
            for b in payload:
                expected_cs ^= b

            if rx_checksum != expected_cs or end_magic != self.MAGIC:
                continue  # Bad frame — discard and keep trying

            return (frame_type, payload)

        return None

    def _read_frame(self, timeout: float) -> tuple[int, bytes] | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._try_parse()
            if frame is not None:
                return frame
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._feed_serial(min(0.05, remaining))
        return None

    def _wait_ready(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame = self._read_frame(timeout=remaining)
            if frame is None:
                break
            if frame[0] == self.RES_READY:
                return True
        return False

    def _wait_cad_result(self, timeout: float) -> dict | None:
        frame = self._read_frame(timeout=timeout)
        if frame is None:
            return None
        ftype, payload = frame
        if ftype != self.RES_CAD_RESULT or len(payload) < 6:
            return None
        detected    = bool(payload[0])
        timed_out   = bool(payload[1])
        duration_us = struct.unpack("<I", payload[2:6])[0]
        return {
            "detected":    detected,
            "timed_out":   timed_out,
            "duration_us": duration_us,
        }


# ============================================================
#  CadTester 主类
# ============================================================

class CadTester:
    def __init__(self):
        self.tx_module: LoRaModule | None = None
        self.cad_node:  CADNode    | None = None
        self.start_time = 0.0
        self._interrupted = False

        # 结果存储（每条记录含 sf/bw_khz 字段）
        self.false_pos_results: list[dict] = []
        self.false_pos_summary: list[dict] = []  # 每个 (sf, bw_khz) 的汇总
        self.sweep_results:     list[dict] = []  # 每个 (sf, bw_khz, num_symbols, trial)
        self.sweep_summary:     list[dict] = []  # 每个 (sf, bw_khz, num_symbols) 的汇总

    # ---- 初始化 ----

    def select_ports(self):
        print("=" * 60)
        print("LR2021 CAD 测试（多 SF/BW 组合）")
        print("=" * 60)

        tx_port  = select_port("请选择 TX 模组串口")
        cad_port = select_port("请选择 CAD 节点串口 (运行 CAD_Node.ino)")

        if tx_port == cad_port:
            print("错误: TX 和 CAD 节点不能使用同一个串口")
            sys.exit(1)

        print(f"\n正在连接 TX 模组 ({tx_port})...")
        self.tx_module = LoRaModule(tx_port, name="TX")

        print(f"\n正在连接 CAD 节点 ({cad_port})...")
        self.cad_node = CADNode(cad_port, name="CAD")

        print("\n设备信息:")
        print(f"  TX : {self.tx_module}")
        print(f"  CAD: {cad_port}")

    def _configure_tx(self, sf: int, bw_khz: int):
        self.tx_module.set_lora_params(
            mode=NodeWorkingMode.TX,
            frequency=FREQUENCY,
            bandwidth=float(bw_khz),
            spreading_factor=sf,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=TX_POWER,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.2)

    def _configure_cad(self, sf: int, bw_khz: int, num_symbols: int):
        freq_hz = int(FREQUENCY * 1_000_000)
        ok = self.cad_node.configure(
            freq_hz=freq_hz,
            sf=sf,
            bw_khz=bw_khz,
            cr=CODING_RATE,
            num_symbols=num_symbols,
            timeout_ms=CAD_TIMEOUT_MS,
        )
        if not ok:
            raise RuntimeError(
                f"CAD 节点配置失败 (SF{sf}, BW{bw_khz}kHz, num_symbols={num_symbols})"
            )

    def _build_payload(self, seq: int) -> bytes:
        payload = bytearray(PAYLOAD_SIZE)
        payload[0] = seq & 0xFF
        payload[1] = (seq >> 8) & 0xFF
        for i in range(2, PAYLOAD_SIZE):
            payload[i] = (seq + i) & 0xFF
        return bytes(payload)

    # ---- Phase 1: 误检率 ----

    def run_false_positive_test(self, sf: int, bw_khz: int):
        """TX 静默，运行 N_FALSE_POS 次 CAD，统计误报。"""
        label = f"SF{sf}/BW{bw_khz}kHz"
        print(f"\n[{self._now()}] Phase 1 — 误检率测试 ({label}, TX静默, {N_FALSE_POS}次CAD)")
        print("-" * 60)

        self._configure_tx(sf, bw_khz)
        self._configure_cad(sf, bw_khz, 4)  # 标准 4 符号
        time.sleep(0.5)

        cad_result_timeout = CAD_TIMEOUT_MS / 1000 + 0.5
        detected_count = 0
        timeout_count  = 0

        print(f"{'#':>5}  {'Detected':>10}  {'Duration(μs)':>14}  {'FP Rate':>8}")
        print("-" * 45)

        for i in range(N_FALSE_POS):
            if self._interrupted:
                break

            result = self.cad_node.do_cad(timeout=cad_result_timeout)

            if result is None:
                timeout_count += 1
                self.false_pos_results.append(
                    {"sf": sf, "bw_khz": bw_khz, "seq": i,
                     "detected": False, "timed_out": True, "duration_us": 0}
                )
                continue

            if result["detected"]:
                detected_count += 1

            self.false_pos_results.append({"sf": sf, "bw_khz": bw_khz, "seq": i, **result})

            if (i + 1) % 50 == 0:
                fp_rate = detected_count / (i + 1) * 100
                print(
                    f"{i+1:>5}  {detected_count:>10}  "
                    f"{result['duration_us']:>14}  {fp_rate:>7.2f}%"
                )

        combo_results = [r for r in self.false_pos_results
                         if r["sf"] == sf and r["bw_khz"] == bw_khz]
        total      = len(combo_results)
        fp_detected = sum(1 for r in combo_results if r.get("detected"))
        fp_rate    = fp_detected / total * 100 if total else 0
        ok_results = [r for r in combo_results if not r.get("timed_out") and r["duration_us"] > 0]

        self.false_pos_summary.append({
            "sf":          sf,
            "bw_khz":      bw_khz,
            "total":       total,
            "fp_detected": fp_detected,
            "fp_rate":     fp_rate,
            "timeouts":    timeout_count,
            "avg_duration_us": float(np.mean([r["duration_us"] for r in ok_results])) if ok_results else 0.0,
        })

        print(f"\n  {label} 误检率: {fp_rate:.2f}% ({fp_detected}/{total})", flush=True)
        if ok_results:
            durations = [r["duration_us"] for r in ok_results]
            print(f"  CAD 耗时: 均值={np.mean(durations):.0f}μs, 最大={np.max(durations):.0f}μs")
        if timeout_count:
            print(f"  超时次数: {timeout_count}")

    # ---- Phase 2: 符号数扫描 ----

    def run_symbol_sweep(self, sf: int, bw_khz: int):
        """对每个 num_symbols，运行 N_DETECTION 次 TX+CAD，统计检测率。"""
        label = f"SF{sf}/BW{bw_khz}kHz"
        print(f"\n[{self._now()}] Phase 2 — 符号数扫描 ({label}, 每档 {N_DETECTION} 次)")
        print(f"  num_symbols: {CAD_SYMBOLS_LIST}")
        print("-" * 60)

        self._configure_tx(sf, bw_khz)

        cr_val = CODING_RATE - 4
        toa    = calc_time_on_air(sf, float(bw_khz), PAYLOAD_SIZE, PREAMBLE_LENGTH, cr_val)
        cad_result_timeout = toa + CAD_TIMEOUT_MS / 1000 + 0.5

        for num_sym in CAD_SYMBOLS_LIST:
            if self._interrupted:
                break

            print(f"\n  [{self._now()}] {label}, num_symbols={num_sym} ...", flush=True)
            self._configure_cad(sf, bw_khz, num_sym)
            time.sleep(0.1)

            detected_count = 0
            timeout_count  = 0
            durations_us   = []

            for seq in range(N_DETECTION):
                if self._interrupted:
                    break

                payload = self._build_payload(seq)
                self.tx_module.send_tx_data(payload, target_sf=sf)

                time.sleep(TX_TO_CAD_DELAY)

                result = self.cad_node.do_cad(timeout=cad_result_timeout)

                if result is None:
                    timeout_count += 1
                    self.sweep_results.append({
                        "sf": sf, "bw_khz": bw_khz,
                        "num_symbols": num_sym, "seq": seq,
                        "detected": False, "timed_out": True, "duration_us": 0,
                    })
                    continue

                if result["detected"]:
                    detected_count += 1
                if not result["timed_out"]:
                    durations_us.append(result["duration_us"])

                self.sweep_results.append({
                    "sf": sf, "bw_khz": bw_khz,
                    "num_symbols": num_sym, "seq": seq,
                    **result,
                })

                elapsed = TX_TO_CAD_DELAY + (result["duration_us"] / 1e6)
                remaining_toa = toa - elapsed
                if remaining_toa > 0:
                    time.sleep(remaining_toa)

            total    = N_DETECTION
            det_rate = detected_count / total * 100

            summary = {
                "sf":             sf,
                "bw_khz":         bw_khz,
                "num_symbols":    num_sym,
                "total":          total,
                "detected":       detected_count,
                "detection_rate": det_rate,
                "timeouts":       timeout_count,
                "avg_duration_us": float(np.mean(durations_us)) if durations_us else 0.0,
                "max_duration_us": float(np.max(durations_us)) if durations_us else 0.0,
                "min_duration_us": float(np.min(durations_us)) if durations_us else 0.0,
                "std_duration_us": float(np.std(durations_us)) if durations_us else 0.0,
            }
            self.sweep_summary.append(summary)

            print(
                f"    检测率={det_rate:.1f}%  ({detected_count}/{total})"
                f"  耗时均值={summary['avg_duration_us']:.0f}μs  超时={timeout_count}"
            )

    # ---- 结果保存 ----

    def save_results(self):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        CHARTS_DIR.mkdir(parents=True, exist_ok=True)

        self._save_csv()
        self._save_report()
        self._save_raw_data()
        self._generate_charts()

        print(f"\n[{self._now()}] 结果已保存到 {OUTPUT_DIR}/")

    def _save_csv(self):
        fp_path = OUTPUT_DIR / "false_positive.csv"
        if self.false_pos_results:
            with open(fp_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.false_pos_results[0].keys())
                writer.writeheader()
                writer.writerows(self.false_pos_results)

        fp_sum_path = OUTPUT_DIR / "false_positive_summary.csv"
        if self.false_pos_summary:
            with open(fp_sum_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.false_pos_summary[0].keys())
                writer.writeheader()
                writer.writerows(self.false_pos_summary)

        sweep_path = OUTPUT_DIR / "sweep_summary.csv"
        if self.sweep_summary:
            with open(sweep_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=self.sweep_summary[0].keys())
                writer.writeheader()
                writer.writerows(self.sweep_summary)

        print(f"[{self._now()}] CSV 已保存")

    def _save_report(self):
        report_path = OUTPUT_DIR / "report.txt"
        elapsed = time.time() - self.start_time

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("LR2021 CAD 测试报告\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"测试耗时: {elapsed / 60:.1f} 分钟\n")
            f.write("=" * 70 + "\n\n")

            f.write("【固定参数】\n")
            f.write(f"  频率: {FREQUENCY} MHz\n")
            f.write(f"  CR:   4/{CODING_RATE}\n")
            f.write(f"  TX 功率: {TX_POWER} dBm\n")
            f.write(f"  Payload: {PAYLOAD_SIZE} bytes\n")
            f.write(f"  CAD 超时: {CAD_TIMEOUT_MS} ms\n\n")

            f.write("【Phase 1 — 误检率汇总（TX 静默, 4 符号, N={}）】\n".format(N_FALSE_POS))
            f.write(f"  {'配置':>15} {'误检率':>8} {'误报/总数':>12} {'超时':>6}\n")
            f.write("  " + "-" * 46 + "\n")
            for s in self.false_pos_summary:
                label = f"SF{s['sf']}/BW{s['bw_khz']}kHz"
                f.write(
                    f"  {label:>15} {s['fp_rate']:>7.2f}%"
                    f" {s['fp_detected']:>5}/{s['total']:<5}"
                    f" {s['timeouts']:>6}\n"
                )

            f.write("\n【Phase 2 — 检测率 vs 符号数】\n")
            combos = sorted({(s["sf"], s["bw_khz"]) for s in self.sweep_summary})
            for sf, bw_khz in combos:
                label = f"SF{sf}/BW{bw_khz}kHz"
                f.write(f"\n  [{label}]\n")
                f.write(
                    f"  {'num_sym':>8} {'检测率':>8} {'检中':>6} {'总数':>6}"
                    f" {'均值(μs)':>10} {'最大(μs)':>10} {'超时':>6}\n"
                )
                f.write("  " + "-" * 62 + "\n")
                for s in self.sweep_summary:
                    if s["sf"] != sf or s["bw_khz"] != bw_khz:
                        continue
                    f.write(
                        f"  {s['num_symbols']:>8} {s['detection_rate']:>7.1f}%"
                        f" {s['detected']:>6} {s['total']:>6}"
                        f" {s['avg_duration_us']:>10.0f} {s['max_duration_us']:>10.0f}"
                        f" {s['timeouts']:>6}\n"
                    )

        print(f"[{self._now()}] 报告已保存: {report_path}")

    def _save_raw_data(self):
        raw_path = OUTPUT_DIR / "raw_data.json"
        data = {
            "test_config": {
                "frequency_mhz":    FREQUENCY,
                "coding_rate":      CODING_RATE,
                "tx_power_dbm":     TX_POWER,
                "payload_size":     PAYLOAD_SIZE,
                "cad_timeout_ms":   CAD_TIMEOUT_MS,
                "n_false_pos":      N_FALSE_POS,
                "n_detection":      N_DETECTION,
                "cad_symbols_list": CAD_SYMBOLS_LIST,
                "sf_bw_combinations": SF_BW_COMBINATIONS,
            },
            "devices": {
                "tx": {
                    "port":     self.tx_module.port,
                    "model":    self.tx_module.model,
                    "node_id":  self.tx_module.node_id,
                    "firmware": self.tx_module.firmware,
                },
                "cad": {"port": self.cad_node.port},
            },
            "false_positive_results": self.false_pos_results,
            "false_positive_summary": self.false_pos_summary,
            "sweep_results":          self.sweep_results,
            "sweep_summary":          self.sweep_summary,
        }
        with open(raw_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f"[{self._now()}] 原始数据已保存: {raw_path}")

    # ---- 图表生成 ----

    def _generate_charts(self):
        plt.rcParams.update({
            "font.family":     "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
            "font.size":       10,
            "axes.titlesize":  12,
            "axes.labelsize":  11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "figure.dpi":      300,
            "savefig.dpi":     300,
        })

        self._plot_false_positive_summary()
        self._plot_detection_rate()
        self._plot_duration_distribution()
        self._plot_combined()

        print(f"[{self._now()}] 图表已保存到 {CHARTS_DIR}/")

    def _combo_label(self, sf: int, bw_khz: int) -> str:
        return f"SF{sf}/BW{bw_khz}kHz"

    def _plot_false_positive_summary(self):
        """各 (SF, BW) 组合的误检率柱状图。"""
        if not self.false_pos_summary:
            return

        labels   = [self._combo_label(s["sf"], s["bw_khz"]) for s in self.false_pos_summary]
        fp_rates = [s["fp_rate"] for s in self.false_pos_summary]
        colors   = ["#d73027" if r > 5 else "#fee08b" if r > 1 else "#1a9850"
                    for r in fp_rates]

        fig, ax = plt.subplots(figsize=(max(6, len(labels) * 1.5), 5))
        bars = ax.bar(range(len(labels)), fp_rates, color=colors,
                      edgecolor="black", linewidth=0.8, alpha=0.85)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_xlabel("Configuration")
        ax.set_ylabel("False Positive Rate (%)")
        ax.set_title(
            f"CAD False Positive Rate by Configuration\n"
            f"{FREQUENCY} MHz, 4 symbols, TX silent, N={N_FALSE_POS}"
        )
        ax.axhline(y=1, color="orange", linestyle="--", alpha=0.7, label="1% Reference")
        ax.grid(True, alpha=0.3, axis="y")

        for bar, rate in zip(bars, fp_rates):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.05,
                    f"{rate:.2f}%", ha="center", va="bottom", fontsize=9)

        patches = [
            mpatches.Patch(color="#1a9850", label="≤1%"),
            mpatches.Patch(color="#fee08b", label="1–5%"),
            mpatches.Patch(color="#d73027", label=">5%"),
        ]
        ax.legend(handles=patches, title="FP Rate", loc="upper right")
        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "false_positive.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_detection_rate(self):
        """每组 (SF, BW) 一条线，x = num_symbols，y = 检测率。"""
        if not self.sweep_summary:
            return

        combos  = sorted({(s["sf"], s["bw_khz"]) for s in self.sweep_summary})
        symbols = sorted({s["num_symbols"] for s in self.sweep_summary})
        colors  = plt.cm.tab10(np.linspace(0, 0.9, len(combos)))

        fig, ax = plt.subplots(figsize=(9, 6))

        for (sf, bw_khz), color in zip(combos, colors):
            label = self._combo_label(sf, bw_khz)
            rows  = sorted(
                [s for s in self.sweep_summary if s["sf"] == sf and s["bw_khz"] == bw_khz],
                key=lambda x: x["num_symbols"]
            )
            xs  = [r["num_symbols"] for r in rows]
            ys  = [r["detection_rate"] for r in rows]
            ax.plot(xs, ys, "o-", color=color, linewidth=2, markersize=7, label=label)

        ax.set_xscale("log", base=2)
        ax.set_xticks(symbols)
        ax.set_xticklabels([str(s) for s in symbols])
        ax.set_xlabel("CAD Symbol Count")
        ax.set_ylabel("Detection Rate (%)")
        ax.set_ylim(-5, 110)
        ax.set_title(
            f"CAD Detection Rate vs Symbol Count\n"
            f"{FREQUENCY} MHz, TX={TX_POWER}dBm, N={N_DETECTION}/config"
        )
        ax.axhline(y=100, color="gray", linestyle="--", alpha=0.4)
        ax.grid(True, alpha=0.3)
        ax.legend(title="Config", loc="lower right")
        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "detection_rate.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_duration_distribution(self):
        """每个 (SF, BW) 组合一个子图，各子图内按 num_symbols 分组箱线图。"""
        if not self.sweep_results:
            return

        combos  = sorted({(r["sf"], r["bw_khz"]) for r in self.sweep_results})
        n_combos = len(combos)
        if n_combos == 0:
            return

        ncols = min(3, n_combos)
        nrows = (n_combos + ncols - 1) // ncols

        fig, axes = plt.subplots(nrows, ncols,
                                 figsize=(5 * ncols, 4.5 * nrows),
                                 squeeze=False)

        for idx, (sf, bw_khz) in enumerate(combos):
            ax    = axes[idx // ncols][idx % ncols]
            label = self._combo_label(sf, bw_khz)

            groups = {}
            for r in self.sweep_results:
                if r["sf"] == sf and r["bw_khz"] == bw_khz:
                    if not r.get("timed_out") and r["duration_us"] > 0:
                        groups.setdefault(r["num_symbols"], []).append(r["duration_us"])

            sym_list  = sorted(groups.keys())
            data_list = [groups[s] for s in sym_list]

            if not data_list:
                ax.text(0.5, 0.5, "No data", ha="center", va="center")
                ax.set_title(label)
                continue

            bp = ax.boxplot(data_list, labels=[str(s) for s in sym_list],
                            patch_artist=True,
                            medianprops=dict(color="black", linewidth=2))
            vir_colors = plt.cm.viridis(np.linspace(0.2, 0.8, len(sym_list)))
            for patch, c in zip(bp["boxes"], vir_colors):
                patch.set_facecolor(c)
                patch.set_alpha(0.7)

            # 理论耗时标注
            t_sym_us = (2 ** sf) / (bw_khz * 1000) * 1e6
            for i, ns in enumerate(sym_list):
                ax.axhline(y=ns * t_sym_us, color=vir_colors[i],
                           linestyle=":", alpha=0.5, linewidth=1)

            ax.set_xlabel("Symbol Count")
            ax.set_ylabel("Duration (μs)")
            ax.set_title(label)
            ax.grid(True, alpha=0.3, axis="y")

        # 隐藏多余子图
        for idx in range(n_combos, nrows * ncols):
            axes[idx // ncols][idx % ncols].set_visible(False)

        fig.suptitle(
            f"CAD Scan Duration Distribution — {FREQUENCY} MHz\n"
            "(dotted = theoretical n × t_sym)",
            fontsize=12, fontweight="bold",
        )
        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "cad_duration_distribution.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_combined(self):
        """检测率 + 均值耗时双 Y 轴，每个 (SF, BW) 一组折线。"""
        if not self.sweep_summary:
            return

        combos  = sorted({(s["sf"], s["bw_khz"]) for s in self.sweep_summary})
        symbols = sorted({s["num_symbols"] for s in self.sweep_summary})
        colors  = plt.cm.tab10(np.linspace(0, 0.9, len(combos)))

        fig, ax1 = plt.subplots(figsize=(9, 6))
        ax2 = ax1.twinx()

        for (sf, bw_khz), color in zip(combos, colors):
            label = self._combo_label(sf, bw_khz)
            rows  = sorted(
                [s for s in self.sweep_summary if s["sf"] == sf and s["bw_khz"] == bw_khz],
                key=lambda x: x["num_symbols"]
            )
            xs      = [r["num_symbols"] for r in rows]
            det     = [r["detection_rate"] for r in rows]
            avg_dur = [r["avg_duration_us"] for r in rows]

            ax1.plot(xs, det, "o-", color=color, linewidth=2,
                     markersize=7, label=f"{label} det.")
            ax2.plot(xs, avg_dur, "s--", color=color, linewidth=1.5,
                     markersize=5, alpha=0.6, label=f"{label} dur.")

        ax1.set_xscale("log", base=2)
        ax1.set_xticks(symbols)
        ax1.set_xticklabels([str(s) for s in symbols])
        ax1.set_xlabel("CAD Symbol Count")
        ax1.set_ylabel("Detection Rate (%)", color="black")
        ax1.set_ylim(-5, 115)
        ax1.axhline(y=100, color="gray", linestyle="--", alpha=0.4)
        ax1.grid(True, alpha=0.3)

        ax2.set_ylabel("Avg CAD Duration (μs)", color="gray")
        ax2.tick_params(axis="y", labelcolor="gray")

        ax1.set_title(
            f"CAD Performance vs Symbol Count — {FREQUENCY} MHz\n"
            f"(solid=detection rate, dashed=duration)"
        )

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2,
                   loc="lower right", fontsize=8, ncol=2)

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "combined.png", bbox_inches="tight")
        plt.close(fig)

    # ---- 工具 ----

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _signal_handler(self, *_):
        print(f"\n[{self._now()}] 收到中断，正在保存结果...")
        self._interrupted = True

    def cleanup(self):
        if self.tx_module:
            self.tx_module.close()
        if self.cad_node:
            self.cad_node.close()


# ============================================================
#  从 JSON 重新绘图
# ============================================================

def replot_from_json(json_path: str | Path):
    json_path = Path(json_path)
    if not json_path.exists():
        print(f"错误: 文件不存在: {json_path}")
        sys.exit(1)

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    tester = CadTester()
    tester.false_pos_results = data.get("false_positive_results", [])
    tester.false_pos_summary = data.get("false_positive_summary", [])
    tester.sweep_results     = data.get("sweep_results", [])
    tester.sweep_summary     = data.get("sweep_summary", [])

    output_dir = json_path.parent
    global OUTPUT_DIR, CHARTS_DIR
    OUTPUT_DIR = output_dir
    CHARTS_DIR = output_dir / "charts"
    CHARTS_DIR.mkdir(parents=True, exist_ok=True)

    tester._generate_charts()
    print("绘图完成！")


# ============================================================
#  主入口
# ============================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="LR2021 CAD 测试（多 SF/BW 组合）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python cad_test.py
  python cad_test.py --replot CAD_Test/raw_data.json
        """
    )
    parser.add_argument("-r", "--replot", metavar="JSON_FILE",
                        help="从 JSON 文件重新生成图表")
    args = parser.parse_args()

    if args.replot:
        replot_from_json(args.replot)
        return

    tester = CadTester()
    tester.start_time = time.time()

    signal.signal(signal.SIGINT,  tester._signal_handler)
    signal.signal(signal.SIGTERM, tester._signal_handler)

    try:
        tester.select_ports()

        print(f"\n测试配置:")
        print(f"  频率={FREQUENCY} MHz, CR=4/{CODING_RATE}, TX={TX_POWER}dBm, Payload={PAYLOAD_SIZE}B")
        print(f"  (SF, BW) 组合: {SF_BW_COMBINATIONS}")
        print(f"  误检率: {N_FALSE_POS} 次/组 × {len(SF_BW_COMBINATIONS)} 组")
        print(f"  检测率: {len(CAD_SYMBOLS_LIST)} 档符号数 × {N_DETECTION} 次/档 × {len(SF_BW_COMBINATIONS)} 组")

        total_tx_calls = len(SF_BW_COMBINATIONS) * len(CAD_SYMBOLS_LIST) * N_DETECTION
        print(f"  总 TX 发包次数 (Phase 2): {total_tx_calls}")

        input("\n按 Enter 开始 (Ctrl+C 随时中断)...")

        for sf, bw_khz in SF_BW_COMBINATIONS:
            if tester._interrupted:
                break

            print(f"\n{'='*60}")
            print(f"  配置: SF{sf}, BW{bw_khz}kHz  (t_sym={2**sf/(bw_khz*1000)*1000:.2f}ms)")
            print(f"{'='*60}")

            tester.run_false_positive_test(sf, bw_khz)

            if not tester._interrupted:
                tester.run_symbol_sweep(sf, bw_khz)

    except KeyboardInterrupt:
        print("\n测试被中断")
    except Exception as e:
        print(f"\n致命错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.save_results()
        tester.cleanup()


if __name__ == "__main__":
    main()
