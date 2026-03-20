#!/usr/bin/env python3
"""
强信号饱和测试

验证 LR2021 在强信号（RSSI ≈ 0 dBm）下的接收性能，测试是否存在饱和失真。

测试场景:
- TX 和 RX 近距离放置，通过调整天线距离使 RSSI 达到 0 dBm
- 记录 PRR、RSSI、SNR，观察是否出现解调失败

功能:
1. 实时 RSSI 监控：TX 持续发包，RX 实时显示 RSSI，方便用户调整距离
2. 饱和点扫描：TX 功率从低到高递增，记录各功率下的 PRR
3. 数据记录：保存 RSSI、SNR、PRR、CRC 状态
"""

import csv
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from node_comm import (
    FrameDecoder,
    LoRaModule,
    NodeWorkingMode,
    PacketType,
    calc_time_on_air,
    select_port,
)


# ============================================================
#  测试配置
# ============================================================

# 固定 LoRa 参数
FREQUENCY = 900.0       # MHz
BANDWIDTH = 500.0        # kHz
SPREADING_FACTOR = 7     # SF7 用于快速测试
CODING_RATE = 5          # 4/5
SYNC_WORD = 0x12
PREAMBLE_LENGTH = 8
PAYLOAD_SIZE = 32        # bytes

# 功率扫描范围
TX_POWER_MIN = 3        # dBm
TX_POWER_MAX = 3        # dBm
TX_POWER_STEP = 1        # dBm

# 每个功率点的包数
PACKETS_PER_POWER = 100

# 输出目录
OUTPUT_DIR = Path("StrongSignal")
CHARTS_DIR = OUTPUT_DIR / "charts"


# ============================================================
#  饱和测试器
# ============================================================

class SaturationTester:
    def __init__(self):
        self.tx_module: LoRaModule | None = None
        self.rx_module: LoRaModule | None = None
        self.results: list[dict] = []          # 每个功率点的汇总结果
        self.raw_rx_data: list[dict] = []      # 所有原始 RX 包
        self.start_time: float = 0
        self._interrupted = False
        self._monitor_running = False

    # ---- 初始化 ----

    def select_ports(self):
        """让用户选择 TX 和 RX 串口"""
        print("=" * 60)
        print("强信号饱和测试 (Strong Signal Saturation Test)")
        print("=" * 60)

        tx_port = select_port("请选择 TX 模组串口")
        rx_port = select_port("请选择 RX 模组串口")

        if tx_port == rx_port:
            print("错误: TX 和 RX 不能使用同一个串口")
            sys.exit(1)

        print(f"\n正在连接 TX 模组 ({tx_port})...")
        self.tx_module = LoRaModule(tx_port, name="TX")

        print(f"正在连接 RX 模组 ({rx_port})...")
        self.rx_module = LoRaModule(rx_port, name="RX")

        print("\n模组信息:")
        print(f"  TX: {self.tx_module}")
        print(f"  RX: {self.rx_module}")

    def select_mode(self) -> str:
        """让用户选择测试模式"""
        print("\n" + "=" * 60)
        print("请选择测试模式:")
        print("  [1] 实时监控模式 - 持续发包，实时显示 RSSI（用于调整天线距离）")
        print("  [2] 正式测试模式 - 功率扫描，记录 PRR 和统计数据")
        print("=" * 60)

        while True:
            try:
                choice = input("输入编号 (1 或 2): ").strip()
                if choice == "1":
                    return "monitor"
                elif choice == "2":
                    return "test"
            except (ValueError, EOFError):
                pass
            print("无效输入，请重试")

    # ---- 模式 1: 实时监控 ----

    def run_monitor_mode(self):
        """实时监控模式：持续发包，实时显示 RSSI"""
        print("\n" + "=" * 60)
        print("实时监控模式")
        print("=" * 60)
        print(f"  频率: {FREQUENCY} MHz")
        print(f"  带宽: {BANDWIDTH} kHz")
        print(f"  SF: {SPREADING_FACTOR}")
        print(f"  Payload: {PAYLOAD_SIZE} bytes")
        print("=" * 60)

        # 让用户选择 TX 功率
        while True:
            try:
                power_str = input(f"请输入 TX 功率 ({TX_POWER_MIN} ~ {TX_POWER_MAX} dBm, 默认 22): ").strip()
                if power_str == "":
                    tx_power = 22
                else:
                    tx_power = int(power_str)
                if TX_POWER_MIN <= tx_power <= TX_POWER_MAX:
                    break
                print(f"功率范围: {TX_POWER_MIN} ~ {TX_POWER_MAX} dBm")
            except ValueError:
                print("请输入有效数字")

        print(f"\n使用 TX 功率: {tx_power} dBm")
        print("按 Ctrl+C 停止监控\n")

        # 配置 LoRa 参数
        self._configure_modules(tx_power)

        # 计算空中时间和超时
        cr_val = CODING_RATE - 4
        toa = calc_time_on_air(SPREADING_FACTOR, BANDWIDTH, PAYLOAD_SIZE, PREAMBLE_LENGTH, cr_val)
        rx_timeout = max(0.1, toa + 0.1)

        # 注册信号处理
        signal.signal(signal.SIGINT, self._monitor_signal_handler)
        self._monitor_running = True

        seq = 0
        rssi_history = []
        snr_history = []

        print(f"{'Seq':>6}  {'RSSI (dBm)':>12}  {'SNR (dB)':>10}  {'CRC':>5}  {'Avg RSSI':>10}  {'Avg SNR':>10}")
        print("-" * 70)

        try:
            while self._monitor_running:
                # 构造 payload
                payload = bytearray(PAYLOAD_SIZE)
                payload[0] = seq & 0xFF
                payload[1] = (seq >> 8) & 0xFF
                for i in range(2, PAYLOAD_SIZE):
                    payload[i] = (seq + i) & 0xFF

                # 发送
                self.tx_module.send_tx_data(bytes(payload), target_sf=SPREADING_FACTOR)

                # 等待接收
                deadline = time.monotonic() + rx_timeout
                received = False
                while time.monotonic() < deadline:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    frame = self.rx_module.read_frame(timeout=remaining)
                    if frame is None:
                        break
                    pkt_type, rx_payload = frame
                    if pkt_type == PacketType.RX_DATA:
                        try:
                            parsed = self.rx_module.parse_rx_data(rx_payload)
                            rssi = parsed["rssi"]
                            snr = parsed["snr"]
                            crc_ok = parsed.get("crc_status") == 1

                            rssi_history.append(rssi)
                            snr_history.append(snr)

                            # 保持最近 50 个样本
                            if len(rssi_history) > 50:
                                rssi_history = rssi_history[-50:]
                                snr_history = snr_history[-50:]

                            avg_rssi = np.mean(rssi_history)
                            avg_snr = np.mean(snr_history)

                            # 显示警告标记
                            rssi_warn = " ⚠️ STRONG" if rssi > -10 else ""
                            crc_str = "OK" if crc_ok else "FAIL"

                            print(
                                f"{seq:>6}  {rssi:>12.1f}  {snr:>10.1f}  "
                                f"{crc_str:>5}  {avg_rssi:>10.1f}  {avg_snr:>10.1f}{rssi_warn}"
                            )
                            received = True
                        except Exception as e:
                            print(f"{seq:>6}  [解析错误: {e}]")
                        break

                if not received:
                    print(f"{seq:>6}  [超时未收到]")

                seq += 1

        except KeyboardInterrupt:
            pass

        print("\n监控结束")

    def _monitor_signal_handler(self, sig, frame):
        self._monitor_running = False

    # ---- 模式 2: 正式测试 ----

    def run_test_mode(self):
        """正式测试模式：功率扫描"""
        self.start_time = time.time()

        # 注册信号处理
        signal.signal(signal.SIGINT, self._test_signal_handler)
        signal.signal(signal.SIGTERM, self._test_signal_handler)

        # 打印测试计划
        power_levels = list(range(TX_POWER_MIN, TX_POWER_MAX + 1, TX_POWER_STEP))
        total_packets = len(power_levels) * PACKETS_PER_POWER

        print("\n" + "=" * 60)
        print("正式测试模式 - 功率扫描")
        print("=" * 60)
        print(f"  频率: {FREQUENCY} MHz")
        print(f"  带宽: {BANDWIDTH} kHz")
        print(f"  SF: {SPREADING_FACTOR}")
        print(f"  Payload: {PAYLOAD_SIZE} bytes")
        print(f"  功率范围: {TX_POWER_MIN} ~ {TX_POWER_MAX} dBm (步进 {TX_POWER_STEP})")
        print(f"  功率点数: {len(power_levels)}")
        print(f"  每点包数: {PACKETS_PER_POWER}")
        print(f"  总包数: {total_packets}")
        print("=" * 60)

        input("\n按 Enter 开始测试 (Ctrl+C 随时中断)...")

        print(f"\n[{self._now()}] 测试开始\n")

        # 逐功率点测试
        for idx, tx_power in enumerate(power_levels):
            if self._interrupted:
                break

            print(
                f"[{self._now()}] "
                f"({idx + 1}/{len(power_levels)}) "
                f"TX Power = {tx_power:+d} dBm ... ",
                end="",
                flush=True,
            )

            result = self._test_power_level(tx_power)
            self.results.append(result)

            # 打印结果摘要
            if result["status"] == "OK":
                print(
                    f"PRR={result['prr']:.1f}% "
                    f"(CRC OK={result['crc_ok']}, Fail={result['crc_fail']}) "
                    f"RSSI={result['avg_rssi']:.1f}dBm, "
                    f"SNR={result['avg_snr']:.1f}dB"
                )
            elif result["status"] == "NO_RESPONSE":
                print("PRR=0% (无包接收)")
            else:
                print(f"[{result['status']}]")

        # 完成
        elapsed = time.time() - self.start_time
        print(f"\n[{self._now()}] 测试完成，耗时 {elapsed / 60:.1f} 分钟")

        if self._interrupted:
            print(f"[{self._now()}] (测试被中断，保存已完成的结果)")

        self.save_results()

    def _test_power_level(self, tx_power: int) -> dict:
        """测试单个功率点"""
        cr_val = CODING_RATE - 4
        toa = calc_time_on_air(SPREADING_FACTOR, BANDWIDTH, PAYLOAD_SIZE, PREAMBLE_LENGTH, cr_val)
        rx_timeout = max(0.1, toa + 0.3)

        # 配置模块
        self._configure_modules(tx_power)
        time.sleep(0.3)

        # 清空缓冲区
        self.rx_module.ser.reset_input_buffer()
        self.rx_module.decoder = FrameDecoder()

        # 逐包发送
        rx_packets = []
        sent_count = 0

        for seq in range(PACKETS_PER_POWER):
            if self._interrupted:
                break

            payload = bytearray(PAYLOAD_SIZE)
            payload[0] = seq & 0xFF
            payload[1] = (seq >> 8) & 0xFF
            for i in range(2, PAYLOAD_SIZE):
                payload[i] = (seq + i) & 0xFF

            self.tx_module.send_tx_data(bytes(payload), target_sf=SPREADING_FACTOR)
            sent_count += 1

            # 等待接收
            deadline = time.monotonic() + rx_timeout
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = self.rx_module.read_frame(timeout=remaining)
                if frame is None:
                    break
                pkt_type, rx_payload = frame
                if pkt_type == PacketType.RX_DATA:
                    try:
                        parsed = self.rx_module.parse_rx_data(rx_payload)
                        parsed["seq"] = seq
                        parsed["tx_power"] = tx_power
                        rx_packets.append(parsed)
                    except Exception:
                        pass
                    break

        # 保存原始数据
        for pkt in rx_packets:
            pkt_record = dict(pkt)
            if "data" in pkt_record and isinstance(pkt_record["data"], bytes):
                pkt_record["data"] = pkt_record["data"].hex()
            self.raw_rx_data.append(pkt_record)

        # 统计
        return self._compute_result(tx_power, sent_count, rx_packets)

    def _compute_result(self, tx_power: int, sent: int, rx_packets: list[dict]) -> dict:
        """统计接收结果"""
        if not rx_packets:
            return {
                "tx_power": tx_power,
                "sent": sent,
                "received": 0,
                "crc_ok": 0,
                "crc_fail": 0,
                "prr": 0.0,
                "avg_rssi": 0.0,
                "min_rssi": 0.0,
                "max_rssi": 0.0,
                "std_rssi": 0.0,
                "avg_snr": 0.0,
                "min_snr": 0.0,
                "max_snr": 0.0,
                "std_snr": 0.0,
                "status": "NO_RESPONSE",
            }

        crc_ok = sum(1 for p in rx_packets if p.get("crc_status") == 1)
        crc_fail = sum(1 for p in rx_packets if p.get("crc_status") == 0)
        total_received = len(rx_packets)

        prr = (crc_ok / sent * 100) if sent > 0 else 0.0

        rssi_list = [p["rssi"] for p in rx_packets]
        snr_list = [p["snr"] for p in rx_packets]

        return {
            "tx_power": tx_power,
            "sent": sent,
            "received": total_received,
            "crc_ok": crc_ok,
            "crc_fail": crc_fail,
            "prr": prr,
            "avg_rssi": float(np.mean(rssi_list)),
            "min_rssi": float(np.min(rssi_list)),
            "max_rssi": float(np.max(rssi_list)),
            "std_rssi": float(np.std(rssi_list)),
            "avg_snr": float(np.mean(snr_list)),
            "min_snr": float(np.min(snr_list)),
            "max_snr": float(np.max(snr_list)),
            "std_snr": float(np.std(snr_list)),
            "status": "OK",
        }

    def _configure_modules(self, tx_power: int):
        """配置 TX 和 RX 模块"""
        # 配置 RX
        self.rx_module.set_lora_params(
            mode=NodeWorkingMode.RX,
            frequency=FREQUENCY,
            bandwidth=BANDWIDTH,
            spreading_factor=SPREADING_FACTOR,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=tx_power,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.1)

        # 配置 TX
        self.tx_module.set_lora_params(
            mode=NodeWorkingMode.TX,
            frequency=FREQUENCY,
            bandwidth=BANDWIDTH,
            spreading_factor=SPREADING_FACTOR,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=tx_power,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.1)

    def _test_signal_handler(self, sig, frame):
        print(f"\n[{self._now()}] 收到中断信号，正在保存结果...")
        self._interrupted = True

    # ---- 结果保存 ----

    def save_results(self):
        """保存测试结果"""
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        CHARTS_DIR.mkdir(parents=True, exist_ok=True)

        self._save_csv()
        self._save_report()
        self._save_raw_data()
        self._generate_charts()

        print(f"[{self._now()}] 结果已保存到 {OUTPUT_DIR}/")

    def _save_csv(self):
        """保存 CSV 结果"""
        csv_path = OUTPUT_DIR / "results.csv"
        fieldnames = [
            "tx_power",
            "sent",
            "received",
            "crc_ok",
            "crc_fail",
            "prr",
            "avg_rssi",
            "min_rssi",
            "max_rssi",
            "std_rssi",
            "avg_snr",
            "min_snr",
            "max_snr",
            "std_snr",
            "status",
        ]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in self.results:
                row = {k: r.get(k, "") for k in fieldnames}
                if isinstance(row["prr"], float):
                    row["prr"] = f"{row['prr']:.2f}"
                writer.writerow(row)

        print(f"[{self._now()}] CSV 已保存: {csv_path}")

    def _save_report(self):
        """保存文本报告"""
        report_path = OUTPUT_DIR / "report.txt"
        elapsed = time.time() - self.start_time

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 60 + "\n")
            f.write("强信号饱和测试报告\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"测试耗时: {elapsed / 60:.1f} 分钟\n")
            f.write("=" * 60 + "\n\n")

            f.write("【设备信息】\n")
            f.write(f"  TX: {self.tx_module}\n")
            f.write(f"  RX: {self.rx_module}\n\n")

            f.write("【测试参数】\n")
            f.write(f"  频率: {FREQUENCY} MHz\n")
            f.write(f"  带宽: {BANDWIDTH} kHz\n")
            f.write(f"  扩频因子: SF{SPREADING_FACTOR}\n")
            f.write(f"  编码率: 4/{CODING_RATE}\n")
            f.write(f"  负载大小: {PAYLOAD_SIZE} bytes\n")
            f.write(f"  功率范围: {TX_POWER_MIN} ~ {TX_POWER_MAX} dBm\n")
            f.write(f"  每点包数: {PACKETS_PER_POWER}\n\n")

            f.write("【测试结果】\n")
            f.write(
                f"{'TX Power':>10} {'PRR%':>8} {'Sent':>6} {'RX':>6} "
                f"{'CRC_OK':>7} {'CRC_FAIL':>9} {'RSSI':>8} {'SNR':>8} {'Status':>12}\n"
            )
            f.write("-" * 85 + "\n")

            for r in self.results:
                prr_str = f"{r['prr']:.1f}" if r["status"] == "OK" else "-"
                rssi_str = f"{r['avg_rssi']:.1f}" if r["status"] == "OK" else "-"
                snr_str = f"{r['avg_snr']:.1f}" if r["status"] == "OK" else "-"
                f.write(
                    f"{r['tx_power']:>+10d} {prr_str:>8} {r['sent']:>6} {r['received']:>6} "
                    f"{r['crc_ok']:>7} {r['crc_fail']:>9} {rssi_str:>8} {snr_str:>8} {r['status']:>12}\n"
                )

            # 饱和分析
            f.write("\n【饱和分析】\n")
            ok_results = [r for r in self.results if r["status"] == "OK"]
            if ok_results:
                # 找到 RSSI > -10 dBm 的结果
                strong_signal = [r for r in ok_results if r["avg_rssi"] > -10]
                if strong_signal:
                    f.write(f"  检测到 {len(strong_signal)} 个强信号点 (RSSI > -10 dBm)\n")
                    for r in strong_signal:
                        f.write(
                            f"    TX={r['tx_power']:+d}dBm: "
                            f"RSSI={r['avg_rssi']:.1f}dBm, "
                            f"PRR={r['prr']:.1f}%, "
                            f"CRC_FAIL={r['crc_fail']}\n"
                        )

                    # 检查是否有 PRR 下降
                    max_prr = max(r["prr"] for r in ok_results)
                    low_prr_strong = [r for r in strong_signal if r["prr"] < max_prr * 0.9]
                    if low_prr_strong:
                        f.write("\n  ⚠️ 可能存在饱和现象:\n")
                        for r in low_prr_strong:
                            f.write(
                                f"    TX={r['tx_power']:+d}dBm: "
                                f"PRR 下降至 {r['prr']:.1f}% (最高 {max_prr:.1f}%)\n"
                            )
                    else:
                        f.write("\n  ✓ 未检测到明显饱和现象\n")
                else:
                    f.write("  未检测到强信号点 (RSSI > -10 dBm)，建议缩短天线距离\n")

        print(f"[{self._now()}] 报告已保存: {report_path}")

    def _save_raw_data(self):
        """保存原始数据"""
        raw_path = OUTPUT_DIR / "raw_data.json"

        output = {
            "test_config": {
                "frequency": FREQUENCY,
                "bandwidth": BANDWIDTH,
                "spreading_factor": SPREADING_FACTOR,
                "coding_rate": CODING_RATE,
                "sync_word": SYNC_WORD,
                "preamble_length": PREAMBLE_LENGTH,
                "payload_size": PAYLOAD_SIZE,
                "tx_power_min": TX_POWER_MIN,
                "tx_power_max": TX_POWER_MAX,
                "tx_power_step": TX_POWER_STEP,
                "packets_per_power": PACKETS_PER_POWER,
            },
            "devices": {
                "tx": {
                    "port": self.tx_module.port,
                    "model": self.tx_module.model,
                    "node_id": self.tx_module.node_id,
                    "firmware": self.tx_module.firmware,
                },
                "rx": {
                    "port": self.rx_module.port,
                    "model": self.rx_module.model,
                    "node_id": self.rx_module.node_id,
                    "firmware": self.rx_module.firmware,
                },
            },
            "results": self.results,
            "raw_rx_packets": self.raw_rx_data,
        }

        with open(raw_path, "w", encoding="utf-8") as f:
            json.dump(output, f, indent=2, ensure_ascii=False)

        print(f"[{self._now()}] 原始数据已保存: {raw_path}")

    # ---- 图表生成 ----

    def _generate_charts(self):
        """生成图表"""
        if not self.results:
            return

        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "figure.dpi": 300,
            "savefig.dpi": 300,
            "axes.grid": True,
        })

        ok_results = [r for r in self.results if r["status"] == "OK"]

        if ok_results:
            self._plot_rssi_distribution()
            self._plot_snr_distribution()
            self._plot_rssi_timeline()
            self._plot_prr_vs_rssi()
            self._plot_prr_vs_power()
            self._plot_combined_analysis()

        print(f"[{self._now()}] 图表已保存到 {CHARTS_DIR}/")

    def _plot_rssi_distribution(self):
        """RSSI 分布直方图"""
        rssi_values = [p["rssi"] for p in self.raw_rx_data if "rssi" in p]
        if not rssi_values:
            return

        fig, ax = plt.subplots(figsize=(8, 5))

        # 直方图
        n, bins, patches = ax.hist(rssi_values, bins=50, edgecolor="black", alpha=0.7)

        # 根据 RSSI 值着色
        for i, (patch, left, right) in enumerate(zip(patches, bins[:-1], bins[1:])):
            mid = (left + right) / 2
            if mid > -10:
                patch.set_facecolor("#d73027")  # 红色 - 强信号
            elif mid > -30:
                patch.set_facecolor("#fc8d59")  # 橙色
            elif mid > -60:
                patch.set_facecolor("#fee08b")  # 黄色
            else:
                patch.set_facecolor("#91cf60")  # 绿色

        ax.axvline(x=-10, color="red", linestyle="--", label="Strong Signal (-10 dBm)")
        ax.axvline(x=0, color="darkred", linestyle="-", linewidth=2, label="Saturation Risk (0 dBm)")

        ax.set_xlabel("RSSI (dBm)")
        ax.set_ylabel("Count")
        ax.set_title("RSSI Distribution")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 添加统计信息
        textstr = f"Mean: {np.mean(rssi_values):.1f} dBm\nStd: {np.std(rssi_values):.1f} dB\nMax: {np.max(rssi_values):.1f} dBm"
        ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "rssi_distribution.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_snr_distribution(self):
        """SNR 分布直方图"""
        snr_values = [p["snr"] for p in self.raw_rx_data if "snr" in p]
        if not snr_values:
            return

        fig, ax = plt.subplots(figsize=(8, 5))

        ax.hist(snr_values, bins=50, edgecolor="black", alpha=0.7, color="#377eb8")

        ax.set_xlabel("SNR (dB)")
        ax.set_ylabel("Count")
        ax.set_title("SNR Distribution")
        ax.grid(True, alpha=0.3)

        textstr = f"Mean: {np.mean(snr_values):.1f} dB\nStd: {np.std(snr_values):.1f} dB"
        ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment="top", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "snr_distribution.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_rssi_timeline(self):
        """RSSI 时间序列图"""
        if not self.raw_rx_data:
            return

        fig, ax = plt.subplots(figsize=(12, 5))

        rssi_values = [p["rssi"] for p in self.raw_rx_data if "rssi" in p]
        tx_powers = [p["tx_power"] for p in self.raw_rx_data if "tx_power" in p]

        x = range(len(rssi_values))

        ax.plot(x, rssi_values, "b-", linewidth=0.5, alpha=0.7, label="RSSI")

        # 标记强信号区域
        strong_mask = [v > -10 for v in rssi_values]
        strong_indices = [i for i, m in enumerate(strong_mask) if m]
        strong_rssi = [rssi_values[i] for i in strong_indices]
        ax.scatter(strong_indices, strong_rssi, c="red", s=10, label="Strong Signal (>-10 dBm)", zorder=5)

        ax.axhline(y=-10, color="red", linestyle="--", alpha=0.5)
        ax.axhline(y=0, color="darkred", linestyle="-", alpha=0.5)

        ax.set_xlabel("Packet Index")
        ax.set_ylabel("RSSI (dBm)")
        ax.set_title("RSSI Timeline")
        ax.legend()
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "rssi_timeline.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_vs_rssi(self):
        """PRR vs RSSI 曲线"""
        ok_results = [r for r in self.results if r["status"] == "OK"]
        if not ok_results:
            return

        fig, ax = plt.subplots(figsize=(8, 6))

        rssi_values = [r["avg_rssi"] for r in ok_results]
        prr_values = [r["prr"] for r in ok_results]

        # 散点图
        scatter = ax.scatter(rssi_values, prr_values, c=prr_values, cmap="RdYlGn",
                            s=80, edgecolor="black", linewidth=0.5, vmin=0, vmax=100)

        # 连接线
        sorted_data = sorted(zip(rssi_values, prr_values))
        sorted_rssi, sorted_prr = zip(*sorted_data)
        ax.plot(sorted_rssi, sorted_prr, "k--", alpha=0.5, linewidth=1)

        ax.axvline(x=-10, color="red", linestyle="--", alpha=0.7, label="Strong Signal (-10 dBm)")
        ax.axvline(x=0, color="darkred", linestyle="-", linewidth=2, alpha=0.7, label="Saturation Risk (0 dBm)")

        ax.set_xlabel("Average RSSI (dBm)")
        ax.set_ylabel("PRR (%)")
        ax.set_title("Packet Reception Rate vs RSSI")
        ax.set_ylim(-5, 105)
        ax.legend()
        ax.grid(True, alpha=0.3)

        cbar = fig.colorbar(scatter, ax=ax)
        cbar.set_label("PRR (%)")

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_vs_rssi.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_vs_power(self):
        """PRR vs TX Power 曲线"""
        ok_results = [r for r in self.results if r["status"] == "OK"]
        if not ok_results:
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        tx_powers = [r["tx_power"] for r in ok_results]
        prr_values = [r["prr"] for r in ok_results]
        rssi_values = [r["avg_rssi"] for r in ok_results]
        crc_fail = [r["crc_fail"] for r in ok_results]

        # 上图: PRR vs TX Power
        ax1.plot(tx_powers, prr_values, "b-o", linewidth=2, markersize=6, label="PRR")
        ax1.fill_between(tx_powers, prr_values, alpha=0.3)
        ax1.set_ylabel("PRR (%)")
        ax1.set_ylim(-5, 105)
        ax1.set_title("PRR and RSSI vs TX Power")
        ax1.legend(loc="lower right")
        ax1.grid(True, alpha=0.3)

        # 双 Y 轴显示 RSSI
        ax1b = ax1.twinx()
        ax1b.plot(tx_powers, rssi_values, "r-s", linewidth=2, markersize=6, label="RSSI")
        ax1b.set_ylabel("RSSI (dBm)", color="red")
        ax1b.tick_params(axis="y", labelcolor="red")
        ax1b.legend(loc="upper left")

        # 下图: CRC Failures
        colors = ["green" if c == 0 else "red" for c in crc_fail]
        ax2.bar(tx_powers, crc_fail, color=colors, edgecolor="black", alpha=0.7)
        ax2.set_xlabel("TX Power (dBm)")
        ax2.set_ylabel("CRC Failures")
        ax2.set_title("CRC Failures vs TX Power")
        ax2.grid(True, alpha=0.3, axis="y")

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_vs_power.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_combined_analysis(self):
        """综合分析图"""
        ok_results = [r for r in self.results if r["status"] == "OK"]
        if not ok_results:
            return

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        tx_powers = [r["tx_power"] for r in ok_results]
        prr_values = [r["prr"] for r in ok_results]
        rssi_values = [r["avg_rssi"] for r in ok_results]
        snr_values = [r["avg_snr"] for r in ok_results]
        crc_fail = [r["crc_fail"] for r in ok_results]

        # 1. PRR vs TX Power
        ax = axes[0, 0]
        ax.plot(tx_powers, prr_values, "b-o", linewidth=2, markersize=6)
        ax.fill_between(tx_powers, prr_values, alpha=0.3)
        ax.set_xlabel("TX Power (dBm)")
        ax.set_ylabel("PRR (%)")
        ax.set_ylim(-5, 105)
        ax.set_title("PRR vs TX Power")
        ax.grid(True, alpha=0.3)

        # 2. RSSI vs TX Power
        ax = axes[0, 1]
        ax.plot(tx_powers, rssi_values, "r-s", linewidth=2, markersize=6)
        ax.axhline(y=-10, color="orange", linestyle="--", label="-10 dBm")
        ax.axhline(y=0, color="red", linestyle="-", label="0 dBm")
        ax.set_xlabel("TX Power (dBm)")
        ax.set_ylabel("RSSI (dBm)")
        ax.set_title("RSSI vs TX Power")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3. SNR vs TX Power
        ax = axes[1, 0]
        ax.plot(tx_powers, snr_values, "g-^", linewidth=2, markersize=6)
        ax.set_xlabel("TX Power (dBm)")
        ax.set_ylabel("SNR (dB)")
        ax.set_title("SNR vs TX Power")
        ax.grid(True, alpha=0.3)

        # 4. PRR vs RSSI with saturation analysis
        ax = axes[1, 1]
        scatter = ax.scatter(rssi_values, prr_values, c=tx_powers, cmap="viridis",
                            s=80, edgecolor="black", linewidth=0.5)
        ax.axvline(x=-10, color="orange", linestyle="--", alpha=0.7)
        ax.axvline(x=0, color="red", linestyle="-", alpha=0.7)
        ax.set_xlabel("RSSI (dBm)")
        ax.set_ylabel("PRR (%)")
        ax.set_ylim(-5, 105)
        ax.set_title("PRR vs RSSI (color = TX Power)")
        ax.grid(True, alpha=0.3)
        cbar = fig.colorbar(scatter, ax=ax)
        cbar.set_label("TX Power (dBm)")

        fig.suptitle("Strong Signal Saturation Test Analysis", fontsize=14, fontweight="bold")
        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "combined_analysis.png", bbox_inches="tight")
        plt.close(fig)

    # ---- 工具方法 ----

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def cleanup(self):
        """清理资源"""
        if self.tx_module:
            self.tx_module.close()
        if self.rx_module:
            self.rx_module.close()


# ============================================================
#  从 JSON 重新绘图
# ============================================================

def replot_from_json(json_path: str | Path):
    """从 JSON 文件重新生成图表"""
    json_path = Path(json_path)

    if not json_path.exists():
        print(f"错误: 文件不存在: {json_path}")
        sys.exit(1)

    print(f"正在读取数据: {json_path}")

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    results = data.get("results", [])
    raw_rx_packets = data.get("raw_rx_packets", [])

    if not results:
        print("错误: JSON 文件中没有结果数据")
        sys.exit(1)

    tester = SaturationTester()
    tester.results = results
    tester.raw_rx_data = raw_rx_packets

    # 设置输出目录
    output_dir = json_path.parent
    charts_dir = output_dir / "charts"
    charts_dir.mkdir(parents=True, exist_ok=True)

    global OUTPUT_DIR, CHARTS_DIR
    OUTPUT_DIR = output_dir
    CHARTS_DIR = charts_dir

    print(f"结果数量: {len(results)}")
    print(f"图表将保存到: {charts_dir}")

    tester._generate_charts()
    print("绘图完成！")


# ============================================================
#  主入口
# ============================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="强信号饱和测试 - 测试 LoRa 模组在强信号下的接收性能",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 运行测试
  python saturation_test.py

  # 从 JSON 文件重新绘图
  python saturation_test.py --replot StrongSignal/raw_data.json
        """
    )

    parser.add_argument(
        "-r", "--replot",
        metavar="JSON_FILE",
        help="从指定的 JSON 文件重新生成图表"
    )

    args = parser.parse_args()

    if args.replot:
        replot_from_json(args.replot)
        return

    tester = SaturationTester()

    try:
        tester.select_ports()
        mode = tester.select_mode()

        if mode == "monitor":
            tester.run_monitor_mode()
        else:
            tester.run_test_mode()

    except KeyboardInterrupt:
        print("\n测试被中断")
        if tester.results:
            tester.save_results()
    except Exception as e:
        print(f"\n致命错误: {e}")
        import traceback
        traceback.print_exc()
        if tester.results:
            tester.save_results()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
