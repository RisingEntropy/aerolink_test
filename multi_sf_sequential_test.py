#!/usr/bin/env python3
"""
Multi-SF Sequential Test (多 SF 顺序测试)

测试目的：
- RX 设置一个主 SF 后，自动带有 3 个次 SF（相邻的 SF）
- 四个 TX 分别发送相邻四个 SF 的信号，采用轮流发送方式
- 对比每个 SF 作为主 SF vs 作为次 SF 时的解调性能差异

测试流程：
1. 对于每个可能的主 SF 配置：
   - RX 设置为该主 SF（次 SF 自动为后续 3 个）
   - 四个 TX 轮流发送各自 SF 的信号
   - 记录每个 SF 的 PRR
2. 分析每个 SF 在不同角色（主/次）时的解调性能差异
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

# 要测试的所有 SF（会以滑动窗口方式，每次取连续 4 个进行测试）
TEST_SF_LIST = [5, 6, 7, 8, 9, 10, 11]

# TX 数量（固定 4 个）
NUM_TX = 4

# 每个 SF 发送的包数（每轮测试中每个 TX 发送的次数）
PACKETS_PER_SF = 100

# 固定参数
TEST_FREQUENCY = 1439  # MHz
TEST_BANDWIDTH = 500.0  # kHz
CODING_RATE = 5  # 4/5
SYNC_WORD = 0x12
PREAMBLE_LENGTH = 8
PAYLOAD_SIZE = 32  # 测试包负载大小 (bytes)

# TX 功率配置
TX_POWER = 10  # dBm

# 输出目录
OUTPUT_DIR = Path("Multi-SF Sequential Test")
CHARTS_DIR = OUTPUT_DIR / "charts"


# ============================================================
#  测试器
# ============================================================

class MultiSFSequentialTester:
    def __init__(self):
        self.tx_modules: list[LoRaModule] = []  # 4 个 TX 模组
        self.rx_module: LoRaModule | None = None  # 1 个 RX 模组 (LR2021)
        self.results: dict[int, dict] = {}  # 主 SF -> 测试结果
        self.raw_rx_data: list[dict] = []
        self.start_time: float = 0
        self._interrupted = False

        # 计算所有要测试的主 SF 配置（滑动窗口）
        # 每个窗口包含 4 个连续的 SF，主 SF 是最小的
        self.test_windows: list[list[int]] = []
        for i in range(len(TEST_SF_LIST) - NUM_TX + 1):
            window = TEST_SF_LIST[i:i + NUM_TX]
            self.test_windows.append(window)

    # ---- 初始化 ----

    def select_ports(self):
        """让用户选择 TX 和 RX 串口"""
        print("=" * 60)
        print("Multi-SF Sequential Test (多 SF 顺序测试)")
        print("=" * 60)
        print("\n测试目的: 验证每个 SF 作为主 SF 和次 SF 时的解调性能差异")
        print()
        print(f"所有测试 SF: {TEST_SF_LIST}")
        print(f"TX 数量: {NUM_TX}")
        print(f"\n测试窗口 (每次取连续 {NUM_TX} 个 SF，主 SF 为最小的):")
        for window in self.test_windows:
            primary = min(window)
            secondary = [sf for sf in window if sf != primary]
            print(f"  主 SF={primary}, 次 SF={secondary}")
        print()

        # 选择 RX 模组 (必须是 LR2021)
        rx_port = select_port("请选择 RX 模组串口 (必须是 LR2021)")
        print(f"\n正在连接 RX 模组 ({rx_port})...")
        self.rx_module = LoRaModule(rx_port, name="RX")

        if self.rx_module.model != LoRaModule.MODEL_LR2021:
            print(f"警告: RX 模组不是 LR2021 (检测到 {self.rx_module.model})")
            confirm = input("是否继续? (y/N): ").strip().lower()
            if confirm != 'y':
                sys.exit(1)

        # 选择 4 个 TX 模组（SF 会在每轮测试时动态配置）
        used_ports = {rx_port}
        for i in range(NUM_TX):
            tx_port = select_port(f"请选择 TX{i+1} 模组串口")
            if tx_port in used_ports:
                print(f"错误: 端口 {tx_port} 已被使用")
                sys.exit(1)
            used_ports.add(tx_port)

            print(f"正在连接 TX{i+1} 模组 ({tx_port})...")
            tx_module = LoRaModule(tx_port, name=f"TX{i+1}")
            self.tx_modules.append(tx_module)

        print("\n模组信息:")
        print(f"  RX: {self.rx_module}")
        for i, tx in enumerate(self.tx_modules):
            print(f"  TX{i+1}: {tx}")

    def print_test_plan(self):
        """打印测试计划"""
        print(f"\n{'=' * 60}")
        print("测试计划:")
        print(f"  测试频率:     {TEST_FREQUENCY} MHz")
        print(f"  测试带宽:     {TEST_BANDWIDTH} kHz")
        print(f"  所有 SF:      {TEST_SF_LIST}")
        print(f"  TX 数量:      {NUM_TX}")
        print(f"  TX 功率:      {TX_POWER} dBm")
        print(f"  每 SF 发包数: {PACKETS_PER_SF}")
        print(f"  测试窗口数:   {len(self.test_windows)}")
        print()
        print("测试流程 (滑动窗口，每次 4 个连续 SF):")
        for window in self.test_windows:
            primary_sf = min(window)
            secondary_sfs = [sf for sf in window if sf != primary_sf]
            print(f"  主 SF={primary_sf}, 次 SF={secondary_sfs}")
            for sf in window:
                role = "主 SF" if sf == primary_sf else "次 SF"
                print(f"    -> TX SF{sf} ({role}) 发送 {PACKETS_PER_SF} 包")
        print(f"{'=' * 60}")

        input("\n按 Enter 开始测试 (Ctrl+C 随时中断)...")

    # ---- 主测试 ----

    def run(self):
        """执行完整测试流程"""
        self.start_time = time.time()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        print(f"\n[{self._now()}] 开始测试...")
        print(f"共 {len(self.test_windows)} 个测试窗口")

        # 对每个窗口进行测试
        for window_idx, window_sfs in enumerate(self.test_windows):
            if self._interrupted:
                break

            primary_sf = min(window_sfs)
            secondary_sfs = [sf for sf in window_sfs if sf != primary_sf]

            print(f"\n{'='*60}")
            print(f"[{self._now()}] 窗口 {window_idx + 1}/{len(self.test_windows)}")
            print(f"  主 SF = {primary_sf}, 次 SF = {secondary_sfs}")
            print(f"{'='*60}")

            # 配置 TX 为当前窗口的 SF
            print(f"[{self._now()}] 配置 TX...")
            for tx, sf in zip(self.tx_modules, window_sfs):
                tx.name = f"TX-SF{sf}"
                print(f"  {tx.name}: SF{sf}, Power={TX_POWER} dBm")
                tx.set_lora_params(
                    mode=NodeWorkingMode.TX,
                    frequency=TEST_FREQUENCY,
                    bandwidth=TEST_BANDWIDTH,
                    spreading_factor=sf,
                    coding_rate=CODING_RATE,
                    sync_word=SYNC_WORD,
                    tx_power=TX_POWER,
                    preamble_length=PREAMBLE_LENGTH,
                )
            time.sleep(0.2)

            result = self._run_window_test(window_sfs, primary_sf)
            self.results[primary_sf] = result

            # 打印结果
            self._print_window_result(primary_sf, window_sfs, result)

        # 完成
        elapsed = time.time() - self.start_time
        print(f"\n[{self._now()}] 测试完成，耗时 {elapsed / 60:.1f} 分钟")

        # 打印总结
        self._print_summary()

        # 保存结果
        self.save_results()

    def _run_window_test(self, window_sfs: list[int], primary_sf: int) -> dict:
        """运行单个窗口的测试"""
        # 配置 RX 为主 SF
        print(f"[{self._now()}] 配置 RX 为主 SF{primary_sf}...")
        self.rx_module.set_lora_params(
            mode=NodeWorkingMode.RX,
            frequency=TEST_FREQUENCY,
            bandwidth=TEST_BANDWIDTH,
            spreading_factor=primary_sf,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=0,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.3)  # 等待 RX 配置稳定

        # 清空 RX 缓冲区
        self.rx_module.ser.reset_input_buffer()
        self.rx_module.decoder = FrameDecoder()

        # 统计结果（只统计当前窗口的 SF）
        sf_stats = {sf: {"tx_count": 0, "rx_count": 0, "rssi_list": [], "snr_list": []}
                    for sf in window_sfs}
        rx_packets = []

        # 轮流测试每个 TX
        for tx, sf in zip(self.tx_modules, window_sfs):
            if self._interrupted:
                break

            role = "主 SF" if sf == primary_sf else "次 SF"
            print(f"\n[{self._now()}] 测试 SF{sf} ({role}), 发送 {PACKETS_PER_SF} 包...")

            # 计算空中时间
            toa = calc_time_on_air(sf, TEST_BANDWIDTH, PAYLOAD_SIZE, PREAMBLE_LENGTH, CODING_RATE - 4)
            rx_timeout = max(0.5, toa + 0.3)

            for seq in range(PACKETS_PER_SF):
                if self._interrupted:
                    break

                # 准备 payload
                payload = bytearray(PAYLOAD_SIZE)
                payload[0] = seq & 0xFF
                payload[1] = (seq >> 8) & 0xFF
                payload[2] = sf & 0xFF  # SF 标记
                payload[3] = primary_sf & 0xFF  # 当前主 SF 标记
                for j in range(4, PAYLOAD_SIZE):
                    payload[j] = (seq + j + sf) & 0xFF

                # 清空 RX 缓冲区
                self.rx_module.ser.reset_input_buffer()
                self.rx_module.decoder = FrameDecoder()

                # 发送
                tx.send_tx_data(bytes(payload), target_sf=sf)
                sf_stats[sf]["tx_count"] += 1

                # 等待 RX 解调
                decoded = False
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
                            detected_sf = parsed.get("sf", 0)
                            if detected_sf == sf:  # 确认是我们发的包
                                sf_stats[sf]["rx_count"] += 1
                                sf_stats[sf]["rssi_list"].append(parsed.get("rssi", 0))
                                sf_stats[sf]["snr_list"].append(parsed.get("snr", 0))
                                decoded = True

                                # 记录详细信息
                                pkt_record = {
                                    "primary_sf": primary_sf,
                                    "tx_sf": sf,
                                    "is_primary": sf == primary_sf,
                                    "seq": seq,
                                    "rssi": parsed.get("rssi", 0),
                                    "snr": parsed.get("snr", 0),
                                    "crc_status": parsed.get("crc_status", 0),
                                }
                                rx_packets.append(pkt_record)
                                self.raw_rx_data.append(pkt_record)
                        except Exception:
                            pass
                        break

                # 等待发送完成再发下一包
                time.sleep(toa + 0.05)

                # 进度显示
                if (seq + 1) % 25 == 0:
                    prr = sf_stats[sf]["rx_count"] / sf_stats[sf]["tx_count"] * 100
                    print(f"  SF{sf} 进度: {seq + 1}/{PACKETS_PER_SF}, PRR: {prr:.1f}%")

        # 计算各 SF 的 PRR
        result = {
            "primary_sf": primary_sf,
            "window_sfs": window_sfs,
            "secondary_sfs": [sf for sf in window_sfs if sf != primary_sf],
            "sf_stats": {},
            "rx_packets": rx_packets,
        }

        for sf in window_sfs:
            stats = sf_stats[sf]
            tx_count = stats["tx_count"]
            rx_count = stats["rx_count"]
            prr = (rx_count / tx_count * 100) if tx_count > 0 else 0

            result["sf_stats"][sf] = {
                "tx_count": tx_count,
                "rx_count": rx_count,
                "prr": prr,
                "is_primary": sf == primary_sf,
                "avg_rssi": np.mean(stats["rssi_list"]) if stats["rssi_list"] else 0,
                "avg_snr": np.mean(stats["snr_list"]) if stats["snr_list"] else 0,
            }

        return result

    def _print_window_result(self, primary_sf: int, window_sfs: list[int], result: dict):
        """打印单个窗口的结果"""
        print(f"\n主 SF = {primary_sf} 测试结果:")
        print("-" * 50)
        print(f"{'SF':>4} {'Role':>8} {'TX':>6} {'RX':>6} {'PRR':>8} {'RSSI':>8} {'SNR':>8}")
        print("-" * 50)

        for sf in window_sfs:
            stats = result["sf_stats"][sf]
            role = "Primary" if stats["is_primary"] else "Second"
            print(f"{sf:>4} {role:>8} {stats['tx_count']:>6} {stats['rx_count']:>6} "
                  f"{stats['prr']:>7.1f}% {stats['avg_rssi']:>7.1f} {stats['avg_snr']:>7.1f}")

    def _print_summary(self):
        """打印总结"""
        print("\n" + "=" * 70)
        print("测试总结: 各 SF 在不同角色时的 PRR")
        print("=" * 70)

        # 获取所有测试过的主 SF
        tested_primary_sfs = sorted(self.results.keys())

        # 构建对比矩阵
        # 行: TX 的 SF
        # 列: 主 SF 配置
        print(f"\n{'':>6}", end="")
        for primary_sf in tested_primary_sfs:
            print(f"{'主SF='+str(primary_sf):>12}", end="")
        print()
        print("-" * (6 + 12 * len(tested_primary_sfs)))

        for tx_sf in TEST_SF_LIST:
            print(f"SF{tx_sf:>3}:", end="")
            for primary_sf in tested_primary_sfs:
                result = self.results.get(primary_sf, {})
                stats = result.get("sf_stats", {}).get(tx_sf, {})
                if stats:
                    prr = stats.get("prr", 0)
                    role = "P" if tx_sf == primary_sf else "S"
                    print(f" {prr:>6.1f}%({role})", end="")
                else:
                    print(f"{'---':>12}", end="")
            print()

        print("-" * (6 + 12 * len(tested_primary_sfs)))

        # 分析每个 SF 作为主 SF vs 次 SF 时的性能差异
        print("\n各 SF 性能对比（主 SF vs 次 SF）:")
        print("-" * 60)
        print(f"{'SF':>4} {'作为主SF时PRR':>15} {'作为次SF时PRR':>15} {'差异':>10}")
        print("-" * 60)

        for sf in TEST_SF_LIST:
            # 作为主 SF 时的 PRR
            primary_prr = None
            if sf in self.results:
                stats = self.results[sf]["sf_stats"].get(sf, {})
                if stats:
                    primary_prr = stats["prr"]

            # 作为次 SF 时的 PRR（取所有作为次 SF 情况的平均）
            secondary_prrs = []
            for primary_sf, result in self.results.items():
                if primary_sf != sf:
                    stats = result.get("sf_stats", {}).get(sf, {})
                    if stats:
                        secondary_prrs.append(stats.get("prr", 0))

            secondary_prr = np.mean(secondary_prrs) if secondary_prrs else None

            if primary_prr is not None and secondary_prr is not None:
                diff = primary_prr - secondary_prr
                print(f"SF{sf:>2} {primary_prr:>14.1f}% {secondary_prr:>14.1f}% {diff:>+9.1f}%")
            elif primary_prr is not None:
                print(f"SF{sf:>2} {primary_prr:>14.1f}% {'N/A':>15} {'N/A':>10}")
            elif secondary_prr is not None:
                print(f"SF{sf:>2} {'N/A':>15} {secondary_prr:>14.1f}% {'N/A':>10}")
            else:
                print(f"SF{sf:>2} {'N/A':>15} {'N/A':>15} {'N/A':>10}")

        print("=" * 70)

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
        """保存 CSV"""
        csv_path = OUTPUT_DIR / "results.csv"
        fieldnames = ["primary_sf", "tx_sf", "role", "tx_count", "rx_count", "prr", "avg_rssi", "avg_snr"]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for primary_sf, result in self.results.items():
                for tx_sf, stats in result["sf_stats"].items():
                    row = {
                        "primary_sf": primary_sf,
                        "tx_sf": tx_sf,
                        "role": "Primary" if stats["is_primary"] else "Secondary",
                        "tx_count": stats["tx_count"],
                        "rx_count": stats["rx_count"],
                        "prr": f"{stats['prr']:.2f}",
                        "avg_rssi": f"{stats['avg_rssi']:.2f}",
                        "avg_snr": f"{stats['avg_snr']:.2f}",
                    }
                    writer.writerow(row)

        print(f"[{self._now()}] CSV 已保存: {csv_path}")

    def _save_report(self):
        """保存报告"""
        report_path = OUTPUT_DIR / "report.txt"
        elapsed = time.time() - self.start_time

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("Multi-SF Sequential Test Report (多 SF 顺序测试)\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"测试耗时: {elapsed / 60:.1f} 分钟\n")
            f.write("=" * 70 + "\n\n")

            f.write("【测试目的】\n")
            f.write("  验证每个 SF 作为主 SF 和次 SF 时的解调性能差异\n\n")

            f.write("【设备信息】\n")
            f.write(f"  RX: {self.rx_module}\n")
            for i, tx in enumerate(self.tx_modules):
                f.write(f"  TX{i+1}: {tx}\n")
            f.write("\n")

            f.write("【测试参数】\n")
            f.write(f"  频率: {TEST_FREQUENCY} MHz\n")
            f.write(f"  带宽: {TEST_BANDWIDTH} kHz\n")
            f.write(f"  测试 SF: {TEST_SF_LIST}\n")
            f.write(f"  TX 数量: {NUM_TX}\n")
            f.write(f"  TX 功率: {TX_POWER} dBm\n")
            f.write(f"  每 SF 发包数: {PACKETS_PER_SF}\n")
            f.write(f"  测试窗口: {len(self.test_windows)}\n\n")

            f.write("【测试结果】\n")
            for primary_sf, result in sorted(self.results.items()):
                window_sfs = result.get("window_sfs", [])
                f.write(f"\n主 SF = {primary_sf}, 窗口 SF = {window_sfs}:\n")
                f.write(f"  {'SF':>4} {'Role':>8} {'TX':>6} {'RX':>6} {'PRR':>8} {'RSSI':>8} {'SNR':>8}\n")
                f.write("  " + "-" * 50 + "\n")
                for tx_sf in window_sfs:
                    stats = result["sf_stats"].get(tx_sf, {})
                    if stats:
                        role = "Primary" if stats["is_primary"] else "Second"
                        f.write(f"  {tx_sf:>4} {role:>8} {stats['tx_count']:>6} {stats['rx_count']:>6} "
                                f"{stats['prr']:>7.1f}% {stats['avg_rssi']:>7.1f} {stats['avg_snr']:>7.1f}\n")

            f.write("\n" + "=" * 70 + "\n")
            f.write("【性能对比】\n\n")

            for sf in TEST_SF_LIST:
                # 作为主 SF 时的 PRR
                primary_prr = None
                if sf in self.results:
                    stats = self.results[sf]["sf_stats"].get(sf, {})
                    if stats:
                        primary_prr = stats["prr"]

                # 作为次 SF 时的 PRR
                secondary_prrs = []
                for primary_sf, result in self.results.items():
                    if primary_sf != sf:
                        stats = result.get("sf_stats", {}).get(sf, {})
                        if stats:
                            secondary_prrs.append(stats.get("prr", 0))
                secondary_prr = np.mean(secondary_prrs) if secondary_prrs else None

                if primary_prr is not None and secondary_prr is not None:
                    diff = primary_prr - secondary_prr
                    f.write(f"  SF{sf}: 主 SF PRR={primary_prr:.1f}%, 次 SF PRR={secondary_prr:.1f}%, 差异={diff:+.1f}%\n")
                elif primary_prr is not None:
                    f.write(f"  SF{sf}: 主 SF PRR={primary_prr:.1f}%, 次 SF PRR=N/A\n")
                elif secondary_prr is not None:
                    f.write(f"  SF{sf}: 主 SF PRR=N/A, 次 SF PRR={secondary_prr:.1f}%\n")

        print(f"[{self._now()}] 报告已保存: {report_path}")

    def _save_raw_data(self):
        """保存原始数据"""
        raw_path = OUTPUT_DIR / "raw_data.json"

        # 将 results 转换为可序列化格式
        serializable_results = {}
        for primary_sf, result in self.results.items():
            serializable_results[str(primary_sf)] = {
                "primary_sf": result["primary_sf"],
                "window_sfs": result.get("window_sfs", []),
                "secondary_sfs": result["secondary_sfs"],
                "sf_stats": {str(k): v for k, v in result["sf_stats"].items()},
            }

        output = {
            "test_config": {
                "frequency": TEST_FREQUENCY,
                "bandwidth": TEST_BANDWIDTH,
                "sf_list": TEST_SF_LIST,
                "num_tx": NUM_TX,
                "packets_per_sf": PACKETS_PER_SF,
                "tx_power": TX_POWER,
                "coding_rate": CODING_RATE,
                "sync_word": SYNC_WORD,
                "preamble_length": PREAMBLE_LENGTH,
                "payload_size": PAYLOAD_SIZE,
                "test_windows": self.test_windows,
            },
            "devices": {
                "rx": {
                    "port": self.rx_module.port if self.rx_module else "",
                    "model": self.rx_module.model if self.rx_module else "",
                    "node_id": self.rx_module.node_id if self.rx_module else "",
                    "firmware": self.rx_module.firmware if self.rx_module else "",
                },
                "tx_modules": [
                    {
                        "index": i,
                        "port": tx.port,
                        "model": tx.model,
                        "node_id": tx.node_id,
                        "firmware": tx.firmware,
                    }
                    for i, tx in enumerate(self.tx_modules)
                ],
            },
            "results": serializable_results,
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
        })

        self._plot_prr_heatmap()
        self._plot_prr_by_role()
        self._plot_prr_comparison()

        print(f"[{self._now()}] 图表已保存到 {CHARTS_DIR}/")

    def _plot_prr_heatmap(self):
        """PRR 热力图"""
        # 获取所有测试过的主 SF
        tested_primary_sfs = sorted(self.results.keys())

        fig, ax = plt.subplots(figsize=(max(8, len(tested_primary_sfs) * 1.5), max(6, len(TEST_SF_LIST) * 0.8)))

        # 构建 PRR 矩阵（行: TX SF, 列: 主 SF 配置）
        # 使用 -1 表示未测试的组合
        prr_matrix = np.full((len(TEST_SF_LIST), len(tested_primary_sfs)), -1.0)

        for j, primary_sf in enumerate(tested_primary_sfs):
            result = self.results[primary_sf]
            for i, tx_sf in enumerate(TEST_SF_LIST):
                stats = result["sf_stats"].get(tx_sf, {})
                if stats:
                    prr_matrix[i, j] = stats.get("prr", 0)

        # 创建带 mask 的热力图
        masked_matrix = np.ma.masked_where(prr_matrix < 0, prr_matrix)
        im = ax.imshow(masked_matrix, cmap="RdYlGn", vmin=0, vmax=100, aspect='auto')

        # 设置标签
        ax.set_xticks(range(len(tested_primary_sfs)))
        ax.set_yticks(range(len(TEST_SF_LIST)))
        ax.set_xticklabels([f"Pri={sf}" for sf in tested_primary_sfs], rotation=45, ha='right')
        ax.set_yticklabels([f"TX SF{sf}" for sf in TEST_SF_LIST])

        ax.set_xlabel("RX Primary SF Configuration")
        ax.set_ylabel("TX Spreading Factor")
        ax.set_title("PRR Heatmap: TX SF vs RX Primary SF\n(P = Primary SF, S = Secondary SF, gray = not tested)")

        # 添加数值标签
        for i in range(len(TEST_SF_LIST)):
            for j in range(len(tested_primary_sfs)):
                primary_sf = tested_primary_sfs[j]
                tx_sf = TEST_SF_LIST[i]
                if prr_matrix[i, j] >= 0:
                    text_color = "white" if prr_matrix[i, j] < 50 else "black"
                    role = "P" if tx_sf == primary_sf else "S"
                    ax.text(j, i, f"{prr_matrix[i, j]:.1f}%\n({role})",
                            ha="center", va="center", color=text_color, fontsize=8)
                else:
                    ax.text(j, i, "-", ha="center", va="center", color="gray", fontsize=10)

        # 添加颜色条
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label("PRR (%)")

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_heatmap.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_by_role(self):
        """按角色分组的 PRR 对比图"""
        fig, ax = plt.subplots(figsize=(max(10, len(TEST_SF_LIST) * 1.2), 6))

        # 收集数据
        primary_prrs = []
        secondary_prrs = []
        has_primary = []
        has_secondary = []

        for sf in TEST_SF_LIST:
            # 作为主 SF 时
            if sf in self.results:
                stats = self.results[sf]["sf_stats"].get(sf, {})
                if stats:
                    primary_prrs.append(stats["prr"])
                    has_primary.append(True)
                else:
                    primary_prrs.append(0)
                    has_primary.append(False)
            else:
                primary_prrs.append(0)
                has_primary.append(False)

            # 作为次 SF 时（平均）
            sec_prrs = []
            for primary_sf, result in self.results.items():
                if primary_sf != sf:
                    stats = result.get("sf_stats", {}).get(sf, {})
                    if stats:
                        sec_prrs.append(stats.get("prr", 0))

            if sec_prrs:
                secondary_prrs.append(np.mean(sec_prrs))
                has_secondary.append(True)
            else:
                secondary_prrs.append(0)
                has_secondary.append(False)

        x = np.arange(len(TEST_SF_LIST))
        width = 0.35

        # 使用不同颜色表示有数据和无数据的情况
        primary_colors = ['#3498db' if hp else '#cccccc' for hp in has_primary]
        secondary_colors = ['#2ecc71' if hs else '#cccccc' for hs in has_secondary]

        bars1 = ax.bar(x - width/2, primary_prrs, width, label='As Primary SF',
                       color=primary_colors, edgecolor='black')
        bars2 = ax.bar(x + width/2, secondary_prrs, width, label='As Secondary SF (avg)',
                       color=secondary_colors, edgecolor='black')

        # 添加数值标签
        for i, bar in enumerate(bars1):
            height = bar.get_height()
            if has_primary[i]:
                ax.annotate(f'{height:.1f}%',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=8)
            else:
                ax.annotate('N/A',
                            xy=(bar.get_x() + bar.get_width() / 2, 5),
                            ha='center', va='bottom', fontsize=7, color='gray')

        for i, bar in enumerate(bars2):
            height = bar.get_height()
            if has_secondary[i]:
                ax.annotate(f'{height:.1f}%',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=8)
            else:
                ax.annotate('N/A',
                            xy=(bar.get_x() + bar.get_width() / 2, 5),
                            ha='center', va='bottom', fontsize=7, color='gray')

        ax.set_xlabel("Spreading Factor")
        ax.set_ylabel("PRR (%)")
        ax.set_title("PRR Comparison: Primary SF vs Secondary SF Role\n(Gray = No data)")
        ax.set_xticks(x)
        ax.set_xticklabels([f"SF{sf}" for sf in TEST_SF_LIST])
        ax.set_ylim(0, 110)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y', linestyle='--')

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_by_role.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_comparison(self):
        """各主 SF 配置下的 PRR 对比"""
        n_configs = len(self.results)
        if n_configs == 0:
            return

        # 根据配置数量调整布局
        n_cols = min(4, n_configs)
        n_rows = (n_configs + n_cols - 1) // n_cols

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 5 * n_rows), squeeze=False)
        axes = axes.flatten()

        for idx, (primary_sf, result) in enumerate(sorted(self.results.items())):
            ax = axes[idx]

            # 使用当前窗口的 SF 列表
            window_sfs = result.get("window_sfs", [])
            if not window_sfs:
                continue

            prrs = []
            for sf in window_sfs:
                stats = result["sf_stats"].get(sf, {})
                prrs.append(stats.get("prr", 0) if stats else 0)

            colors = ["#3498db" if sf == primary_sf else "#2ecc71" for sf in window_sfs]

            bars = ax.bar([f"SF{sf}" for sf in window_sfs], prrs, color=colors, edgecolor='black')

            # 添加数值标签
            for bar, prr in zip(bars, prrs):
                height = bar.get_height()
                if height > 0:
                    ax.annotate(f'{prr:.1f}%',
                                xy=(bar.get_x() + bar.get_width() / 2, height),
                                xytext=(0, 3), textcoords="offset points",
                                ha='center', va='bottom', fontweight='bold', fontsize=9)

            ax.set_xlabel("TX Spreading Factor")
            ax.set_ylabel("PRR (%)")
            ax.set_title(f"Primary SF = {primary_sf}", fontweight='bold')
            ax.set_ylim(0, 110)
            ax.grid(True, alpha=0.3, axis='y', linestyle='--')

        # 隐藏多余的子图
        for idx in range(n_configs, len(axes)):
            axes[idx].set_visible(False)

        # 添加图例
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#3498db', edgecolor='black', label='Primary SF'),
            Patch(facecolor='#2ecc71', edgecolor='black', label='Secondary SF'),
        ]
        fig.legend(handles=legend_elements, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 0.02))

        fig.suptitle("PRR by Primary SF Configuration\n(Blue = Primary, Green = Secondary)", fontsize=12, fontweight='bold')
        fig.tight_layout(rect=[0, 0.05, 1, 0.95])
        fig.savefig(CHARTS_DIR / "prr_by_primary_sf.png", bbox_inches="tight")
        plt.close(fig)

    # ---- 工具方法 ----

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _signal_handler(self, sig, frame):
        print(f"\n[{self._now()}] 收到中断信号...")
        self._interrupted = True

    def cleanup(self):
        """清理资源"""
        for tx in self.tx_modules:
            tx.close()
        if self.rx_module:
            self.rx_module.close()


# ============================================================
#  从 JSON 文件重新绘图
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

    results_data = data.get("results", {})
    if not results_data:
        print("错误: JSON 文件中没有结果数据")
        sys.exit(1)

    tester = MultiSFSequentialTester()

    # 恢复 results（转换 key 类型）
    tester.results = {}
    for primary_sf_str, result in results_data.items():
        primary_sf = int(primary_sf_str)
        # 转换 sf_stats 的 key
        sf_stats = {}
        for sf_str, stats in result.get("sf_stats", {}).items():
            sf_stats[int(sf_str)] = stats
        tester.results[primary_sf] = {
            "primary_sf": result["primary_sf"],
            "window_sfs": result.get("window_sfs", []),
            "secondary_sfs": result["secondary_sfs"],
            "sf_stats": sf_stats,
        }

    # 从配置中恢复参数
    test_config = data.get("test_config", {})
    global TEST_SF_LIST, TEST_FREQUENCY, TEST_BANDWIDTH, PACKETS_PER_SF, NUM_TX
    TEST_SF_LIST = test_config.get("sf_list", TEST_SF_LIST)
    TEST_FREQUENCY = test_config.get("frequency", TEST_FREQUENCY)
    TEST_BANDWIDTH = test_config.get("bandwidth", TEST_BANDWIDTH)
    PACKETS_PER_SF = test_config.get("packets_per_sf", PACKETS_PER_SF)
    NUM_TX = test_config.get("num_tx", NUM_TX)

    # 恢复 test_windows
    tester.test_windows = test_config.get("test_windows", [])

    output_dir = json_path.parent
    charts_dir = output_dir / "charts"
    charts_dir.mkdir(parents=True, exist_ok=True)

    global OUTPUT_DIR, CHARTS_DIR
    OUTPUT_DIR = output_dir
    CHARTS_DIR = charts_dir

    print(f"SF 列表: {TEST_SF_LIST}")
    print(f"主 SF 配置数量: {len(tester.results)}")
    print(f"图表将保存到: {charts_dir}")

    tester._generate_charts()

    print("绘图完成！")


# ============================================================
#  主入口
# ============================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Multi-SF Sequential Test - 测试各 SF 作为主/次 SF 时的解调性能差异",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 运行完整测试
  python multi_sf_sequential_test.py

  # 从 JSON 文件重新绘图
  python multi_sf_sequential_test.py --replot "Multi-SF Sequential Test/raw_data.json"
  python multi_sf_sequential_test.py -r path/to/raw_data.json
        """
    )

    parser.add_argument(
        "-r", "--replot",
        metavar="JSON_FILE",
        help="从指定的 JSON 文件重新生成图表（不运行测试）"
    )

    args = parser.parse_args()

    if args.replot:
        replot_from_json(args.replot)
        return

    tester = MultiSFSequentialTester()

    try:
        tester.select_ports()
        tester.print_test_plan()
        tester.run()
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
