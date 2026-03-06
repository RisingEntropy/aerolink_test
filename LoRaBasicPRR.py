#!/usr/bin/env python3
"""
LoRa Basic PRR Test

验证 LR2021 模组在 Sub-GHz / 1.4GHz 频段、SF5~SF12、
BW 125k/250k/400k/500k/1MHz 各组合下的 Packet Reception Rate。

两个模组通过同轴线+衰减器连接，TX 功率由用户手动设定。
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
import matplotlib.ticker as mticker
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
#  测试配置（请根据实际情况修改）
# ============================================================

# 每个参数组合发送的测试包数量
PACKETS_PER_COMBO = 100

# TX 功率 (dBm) — 根据衰减器手动调整，使 RX 端 RSSI ≈ -100 dBm
TX_POWER = 0

# 固定参数
CODING_RATE = 5  # 4/5
SYNC_WORD = 0x12  # Private sync word
PREAMBLE_LENGTH = 8
PAYLOAD_SIZE = 32  # 测试包负载大小 (bytes)

# 频点列表 (MHz) — 覆盖 Sub-GHz (470-510 / 850-920) 和 1.4 GHz (1430-1444)
FREQUENCIES = [
    # Sub-GHz band 1: 470-510 MHz
    490.0,
    # Sub-GHz band 2: 850-920 MHz
    900.0,
    # 1.4 GHz band: 1430-1444 MHz
    1437.0,
]

# Spreading Factor 列表
SF_LIST = [5, 6, 7, 8, 9, 10, 11, 12]

# Bandwidth 列表 (kHz)
BW_LIST = [125.0, 250.0, 500.0, 1000.0]

# 输出目录
OUTPUT_DIR = Path("LoRa Basic PRR")
CHARTS_DIR = OUTPUT_DIR / "charts"


# ============================================================
#  测试器
# ============================================================

class PRRTester:
    def __init__(self):
        self.tx_module: LoRaModule | None = None
        self.rx_module: LoRaModule | None = None
        self.results: list[dict] = []
        self.raw_rx_data: list[dict] = []  # 所有原始 RX 包
        self.start_time: float = 0
        self._interrupted = False

    # ---- 初始化 ----

    def select_ports(self):
        """让用户选择 TX 和 RX 串口，并连接模组"""
        print("=" * 60)
        print("LoRa Basic PRR Test")
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

    def print_test_plan(self):
        """打印测试矩阵概要和预估耗时"""
        n_combos = len(FREQUENCIES) * len(SF_LIST) * len(BW_LIST)
        total_packets = n_combos * PACKETS_PER_COMBO

        # 粗略估算总耗时（逐包发送+等待模式）
        estimated_seconds = 0
        for freq in FREQUENCIES:
            for sf in SF_LIST:
                for bw in BW_LIST:
                    cr_val = CODING_RATE - 4  # coding_rate 5 → cr=1
                    toa = calc_time_on_air(sf, bw, PAYLOAD_SIZE, PREAMBLE_LENGTH, cr_val)
                    # 每包: 超时时间 = max(0.1, toa + 0.05)
                    per_packet = max(0.1, toa + 0.05)
                    estimated_seconds += per_packet * PACKETS_PER_COMBO
                    # 加上参数切换开销
                    estimated_seconds += 0.1

        est_minutes = estimated_seconds / 60

        print(f"\n{'=' * 60}")
        print("测试计划:")
        print(f"  频点数:    {len(FREQUENCIES)}")
        print(f"  SF 范围:   SF{min(SF_LIST)}~SF{max(SF_LIST)} ({len(SF_LIST)} 个)")
        print(f"  BW 范围:   {len(BW_LIST)} 个 ({', '.join(f'{b}k' for b in BW_LIST)})")
        print(f"  参数组合:  {n_combos}")
        print(f"  每组包数:  {PACKETS_PER_COMBO}")
        print(f"  总包数:    {total_packets}")
        print(f"  TX 功率:   {TX_POWER} dBm")
        print(f"  预估耗时:  {est_minutes:.1f} 分钟")
        print(f"{'=' * 60}")

        input("\n按 Enter 开始测试 (Ctrl+C 随时中断)...")

    # ---- 测试执行 ----

    def run(self):
        """执行完整测试流程"""
        self.start_time = time.time()

        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        total_combos = len(FREQUENCIES) * len(SF_LIST) * len(BW_LIST)
        current_combo = 0

        print(f"\n[{self._now()}] 测试开始\n")

        for freq in FREQUENCIES:
            if self._interrupted:
                break

            for bw in BW_LIST:
                if self._interrupted:
                    break

                for sf in SF_LIST:
                    if self._interrupted:
                        break

                    current_combo += 1
                    print(
                        f"[{self._now()}] "
                        f"({current_combo}/{total_combos}) "
                        f"Freq={freq}MHz, SF{sf}, BW={bw}kHz ... ",
                        end="",
                        flush=True,
                    )

                    try:
                        result = self._test_combination(freq, sf, bw)
                        self.results.append(result)

                        # 打印结果摘要
                        prr = result["prr"]
                        status = result["status"]
                        if status == "OK":
                            avg_rssi = result.get("avg_rssi", 0)
                            avg_snr = result.get("avg_snr", 0)
                            print(
                                f"PRR={prr:.1f}% "
                                f"(CRC OK={result['crc_ok']}, "
                                f"Fail={result['crc_fail']}) "
                                f"RSSI={avg_rssi:.1f}dBm, "
                                f"SNR={avg_snr:.1f}dB"
                            )
                        elif status == "NO_RESPONSE":
                            print("PRR=0% (无包接收)")
                        else:
                            print(f"[{status}]")

                    except Exception as e:
                        print(f"错误: {e}")
                        self.results.append(
                            self._make_result(freq, sf, bw, status="ERROR")
                        )

        # 完成 → 保存结果
        elapsed = time.time() - self.start_time
        print(f"\n[{self._now()}] 测试完成，耗时 {elapsed / 60:.1f} 分钟")

        if self._interrupted:
            print(f"[{self._now()}] (测试被中断，保存已完成的结果)")

        self.save_results()

    def _test_combination(self, freq: float, sf: int, bw: float) -> dict:
        """测试单个参数组合: 逐包发送，每包等待接收（超时则判定丢包）"""
        cr_val = CODING_RATE - 4  # coding_rate 5 → cr=1 for ToA calc
        toa = calc_time_on_air(sf, bw, PAYLOAD_SIZE, PREAMBLE_LENGTH, cr_val)
        # 超时 = 空中时间 + 50ms 处理余量，最小 100ms
        rx_timeout = max(0.1, toa + 0.05)

        # 1. 配置 RX 模组
        self.rx_module.set_lora_params(
            mode=NodeWorkingMode.RX,
            frequency=freq,
            bandwidth=bw,
            spreading_factor=sf,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=TX_POWER,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.05)  # 等待 RX 就绪

        # 清空 RX 缓冲区和解码器状态，避免旧数据干扰
        self.rx_module.ser.reset_input_buffer()
        self.rx_module.decoder = FrameDecoder()

        # 2. 配置 TX 模组
        self.tx_module.set_lora_params(
            mode=NodeWorkingMode.TX,
            frequency=freq,
            bandwidth=bw,
            spreading_factor=sf,
            coding_rate=CODING_RATE,
            sync_word=SYNC_WORD,
            tx_power=TX_POWER,
            preamble_length=PREAMBLE_LENGTH,
        )
        time.sleep(0.02)

        # 3. 逐包发送 & 等待接收
        rx_packets = []
        sent_count = 0

        for seq in range(PACKETS_PER_COMBO):
            if self._interrupted:
                break

            # 构造负载: 2 字节序号 (LE) + 填充字节
            payload = bytearray(PAYLOAD_SIZE)
            payload[0] = seq & 0xFF
            payload[1] = (seq >> 8) & 0xFF
            for i in range(2, PAYLOAD_SIZE):
                payload[i] = (seq + i) & 0xFF

            # 发送一包
            self.tx_module.send_tx_data(bytes(payload), target_sf=sf)
            sent_count += 1

            # 等待 RX_DATA 响应（超时则判定为丢包）
            deadline = time.monotonic() + rx_timeout
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = self.rx_module.read_frame(timeout=remaining)
                if frame is None:
                    break  # 超时，未收到
                pkt_type, rx_payload = frame
                if pkt_type == PacketType.RX_DATA:
                    try:
                        parsed = self.rx_module.parse_rx_data(rx_payload)
                        rx_packets.append(parsed)
                    except Exception:
                        pass
                    break  # 收到 RX_DATA → 进入下一包

        # 4. 保存原始 RX 数据
        for pkt in rx_packets:
            pkt_record = dict(pkt)
            pkt_record["test_freq"] = freq
            pkt_record["test_sf"] = sf
            pkt_record["test_bw"] = bw
            if "data" in pkt_record and isinstance(pkt_record["data"], bytes):
                pkt_record["data"] = pkt_record["data"].hex()
            self.raw_rx_data.append(pkt_record)

        # 5. 统计
        return self._compute_result(freq, sf, bw, sent_count, rx_packets)

    def _compute_result(
        self,
        freq: float,
        sf: int,
        bw: float,
        sent: int,
        rx_packets: list[dict],
    ) -> dict:
        """统计接收结果"""
        if not rx_packets:
            return self._make_result(freq, sf, bw, sent=sent, status="NO_RESPONSE")

        crc_ok = sum(1 for p in rx_packets if p.get("crc_status") == 1)
        crc_fail = sum(1 for p in rx_packets if p.get("crc_status") == 0)
        total_received = len(rx_packets)

        # PRR 只计算 CRC 通过的包
        prr = (crc_ok / sent * 100) if sent > 0 else 0.0

        # RSSI / SNR 统计（所有包）
        rssi_list = [p["rssi"] for p in rx_packets]
        snr_list = [p["snr"] for p in rx_packets]

        return self._make_result(
            freq,
            sf,
            bw,
            sent=sent,
            received=total_received,
            crc_ok=crc_ok,
            crc_fail=crc_fail,
            prr=prr,
            avg_rssi=float(np.mean(rssi_list)),
            min_rssi=float(np.min(rssi_list)),
            max_rssi=float(np.max(rssi_list)),
            avg_snr=float(np.mean(snr_list)),
            min_snr=float(np.min(snr_list)),
            max_snr=float(np.max(snr_list)),
            status="OK",
        )

    @staticmethod
    def _make_result(freq, sf, bw, **kwargs) -> dict:
        """创建标准结果字典"""
        result = {
            "frequency": freq,
            "sf": sf,
            "bandwidth": bw,
            "tx_power": TX_POWER,
            "sent": kwargs.get("sent", 0),
            "received": kwargs.get("received", 0),
            "crc_ok": kwargs.get("crc_ok", 0),
            "crc_fail": kwargs.get("crc_fail", 0),
            "prr": kwargs.get("prr", 0.0),
            "avg_rssi": kwargs.get("avg_rssi", 0.0),
            "min_rssi": kwargs.get("min_rssi", 0.0),
            "max_rssi": kwargs.get("max_rssi", 0.0),
            "avg_snr": kwargs.get("avg_snr", 0.0),
            "min_snr": kwargs.get("min_snr", 0.0),
            "max_snr": kwargs.get("max_snr", 0.0),
            "status": kwargs.get("status", ""),
        }
        return result

    # ---- 结果保存 ----

    def save_results(self):
        """保存测试结果到输出目录"""
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
            "frequency",
            "sf",
            "bandwidth",
            "tx_power",
            "sent",
            "received",
            "crc_ok",
            "crc_fail",
            "prr",
            "avg_rssi",
            "min_rssi",
            "max_rssi",
            "avg_snr",
            "min_snr",
            "max_snr",
            "status",
        ]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in self.results:
                row = {k: r.get(k, "") for k in fieldnames}
                # PRR 格式化
                if isinstance(row["prr"], float):
                    row["prr"] = f"{row['prr']:.2f}"
                writer.writerow(row)

        print(f"[{self._now()}] CSV 已保存: {csv_path}")

    def _save_report(self):
        """保存文本汇总报告"""
        report_path = OUTPUT_DIR / "report.txt"
        elapsed = time.time() - self.start_time

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 60 + "\n")
            f.write("LoRa Basic PRR 测试报告\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"测试耗时: {elapsed / 60:.1f} 分钟\n")
            f.write("=" * 60 + "\n\n")

            # 设备信息
            f.write("【设备信息】\n")
            f.write(f"  TX: {self.tx_module}\n")
            f.write(f"  RX: {self.rx_module}\n\n")

            # 测试参数
            f.write("【测试参数】\n")
            f.write(f"  频点: {FREQUENCIES}\n")
            f.write(f"  SF: {SF_LIST}\n")
            f.write(f"  BW: {BW_LIST}\n")
            f.write(f"  每组包数: {PACKETS_PER_COMBO}\n")
            f.write(f"  TX 功率: {TX_POWER} dBm\n")
            f.write(f"  编码率: 4/{CODING_RATE}\n")
            f.write(f"  负载大小: {PAYLOAD_SIZE} bytes\n\n")

            # 汇总
            ok_results = [r for r in self.results if r["status"] == "OK"]
            no_resp = [r for r in self.results if r["status"] == "NO_RESPONSE"]
            error = [r for r in self.results if r["status"] == "ERROR"]

            f.write("【汇总统计】\n")
            f.write(f"  总测试组合: {len(self.results)}\n")
            f.write(f"  成功组合: {len(ok_results)}\n")
            f.write(f"  无响应组合: {len(no_resp)}\n")
            f.write(f"  出错组合: {len(error)}\n")

            if ok_results:
                prr_values = [r["prr"] for r in ok_results]
                f.write(f"  PRR 均值: {np.mean(prr_values):.2f}%\n")
                f.write(f"  PRR 最小: {np.min(prr_values):.2f}%\n")
                f.write(f"  PRR 最大: {np.max(prr_values):.2f}%\n\n")

            # 按频率分组的详细结果
            f.write("【按频率分组的详细结果】\n")
            for freq in FREQUENCIES:
                freq_results = [r for r in self.results if r["frequency"] == freq]
                if not freq_results:
                    continue

                f.write(f"\n  === {freq} MHz ===\n")
                f.write(
                    f"  {'SF':>4} {'BW(kHz)':>8} {'PRR%':>7} "
                    f"{'Sent':>5} {'RX':>4} {'CRC_OK':>6} "
                    f"{'RSSI':>7} {'SNR':>6} {'Status':>12}\n"
                )
                f.write("  " + "-" * 70 + "\n")

                for r in freq_results:
                    prr_str = f"{r['prr']:.1f}" if r["status"] == "OK" else "-"
                    rssi_str = (
                        f"{r['avg_rssi']:.1f}" if r["status"] == "OK" else "-"
                    )
                    snr_str = f"{r['avg_snr']:.1f}" if r["status"] == "OK" else "-"
                    f.write(
                        f"  {r['sf']:>4} {r['bandwidth']:>8.0f} {prr_str:>7} "
                        f"{r['sent']:>5} {r['received']:>4} {r['crc_ok']:>6} "
                        f"{rssi_str:>7} {snr_str:>6} {r['status']:>12}\n"
                    )

        print(f"[{self._now()}] 报告已保存: {report_path}")

    def _save_raw_data(self):
        """保存原始 RX 数据"""
        raw_path = OUTPUT_DIR / "raw_data.json"

        output = {
            "test_config": {
                "frequencies": FREQUENCIES,
                "sf_list": SF_LIST,
                "bw_list": BW_LIST,
                "packets_per_combo": PACKETS_PER_COMBO,
                "tx_power": TX_POWER,
                "coding_rate": CODING_RATE,
                "sync_word": SYNC_WORD,
                "preamble_length": PREAMBLE_LENGTH,
                "payload_size": PAYLOAD_SIZE,
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
        """生成论文级图表"""
        if not self.results:
            return

        # 统一样式
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
            "axes.grid": False,
        })

        ok_results = [r for r in self.results if r["status"] == "OK"]
        tested_freqs = sorted(set(r["frequency"] for r in self.results))
        tested_sfs = sorted(set(r["sf"] for r in self.results))
        tested_bws = sorted(set(r["bandwidth"] for r in self.results))

        # 论文常用配色
        prr_cmap = LinearSegmentedColormap.from_list(
            "prr", ["#d73027", "#fc8d59", "#fee08b", "#d9ef8b", "#91cf60", "#1a9850"]
        )
        rssi_cmap = "YlOrRd_r"
        snr_cmap = "RdYlBu"

        # ── Fig 1: PRR Heatmap (SF × BW) 每个频率一张 ──
        self._plot_metric_heatmaps(
            tested_freqs, tested_sfs, tested_bws,
            metric="prr",
            title_prefix="PRR",
            fmt=".1f",
            unit="%",
            cmap=prr_cmap,
            vmin=0,
            vmax=100,
            filename_prefix="prr_heatmap",
        )

        # ── Fig 2: PRR vs SF 折线图（每个频率一子图） ──
        self._plot_prr_vs_sf(tested_freqs, tested_sfs, tested_bws)

        # ── Fig 3: RSSI Heatmap 每个频率一张 ──
        self._plot_metric_heatmaps(
            tested_freqs, tested_sfs, tested_bws,
            metric="avg_rssi",
            title_prefix="Average RSSI",
            fmt=".1f",
            unit="dBm",
            cmap=rssi_cmap,
            vmin=None,
            vmax=None,
            filename_prefix="rssi_heatmap",
        )

        # ── Fig 4: SNR Heatmap 每个频率一张 ──
        self._plot_metric_heatmaps(
            tested_freqs, tested_sfs, tested_bws,
            metric="avg_snr",
            title_prefix="Average SNR",
            fmt=".1f",
            unit="dB",
            cmap=snr_cmap,
            vmin=None,
            vmax=None,
            filename_prefix="snr_heatmap",
        )

        # ── Fig 5: PRR 跨频段对比 ──
        if len(tested_freqs) > 1:
            self._plot_prr_band_comparison(tested_freqs, tested_sfs, tested_bws)

        # ── Fig 6: CRC 错误率 Heatmap ──
        self._plot_crc_error_heatmaps(tested_freqs, tested_sfs, tested_bws)

        print(f"[{self._now()}] 图表已保存到 {CHARTS_DIR}/")

    def _build_matrix(self, freq, tested_sfs, tested_bws, metric):
        """为指定频率构建 (SF × BW) 矩阵"""
        matrix = np.full((len(tested_sfs), len(tested_bws)), np.nan)
        for r in self.results:
            if r["frequency"] != freq:
                continue
            if r["status"] != "OK" and metric in ("prr", "avg_rssi", "avg_snr"):
                continue
            try:
                si = tested_sfs.index(r["sf"])
                bi = tested_bws.index(r["bandwidth"])
            except ValueError:
                continue
            if metric == "crc_error_rate":
                total = r["crc_ok"] + r["crc_fail"]
                matrix[si][bi] = (r["crc_fail"] / total * 100) if total > 0 else 0
            else:
                matrix[si][bi] = r.get(metric, np.nan)
        return matrix

    def _plot_metric_heatmaps(
        self, tested_freqs, tested_sfs, tested_bws,
        metric, title_prefix, fmt, unit, cmap, vmin, vmax,
        filename_prefix,
    ):
        """绘制 (SF × BW) 热力图，每个频率一张子图，拼成一张大图"""
        n_freqs = len(tested_freqs)
        fig, axes = plt.subplots(
            1, n_freqs,
            figsize=(5 * n_freqs + 1, 4.5),
            squeeze=False,
        )
        axes = axes.flatten()

        bw_labels = [f"{bw:.0f}" if bw < 1000 else f"{bw/1000:.0f}M"
                     for bw in tested_bws]
        sf_labels = [f"SF{sf}" for sf in tested_sfs]

        # 自动 vmin/vmax
        if vmin is None or vmax is None:
            all_vals = []
            for freq in tested_freqs:
                m = self._build_matrix(freq, tested_sfs, tested_bws, metric)
                all_vals.extend(m[~np.isnan(m)].tolist())
            if all_vals:
                if vmin is None:
                    vmin = min(all_vals)
                if vmax is None:
                    vmax = max(all_vals)

        for idx, freq in enumerate(tested_freqs):
            ax = axes[idx]
            matrix = self._build_matrix(freq, tested_sfs, tested_bws, metric)

            im = ax.imshow(
                matrix, cmap=cmap, aspect="auto",
                vmin=vmin, vmax=vmax,
                interpolation="nearest",
            )

            # 标注数值
            for i in range(len(tested_sfs)):
                for j in range(len(tested_bws)):
                    val = matrix[i][j]
                    if np.isnan(val):
                        text = "N/A"
                        color = "gray"
                    else:
                        text = f"{val:{fmt}}"
                        # 根据亮度选文字颜色
                        norm_val = (val - vmin) / (vmax - vmin + 1e-9)
                        color = "white" if norm_val < 0.4 else "black"
                    ax.text(j, i, text, ha="center", va="center",
                            fontsize=7, color=color, fontweight="bold")

            ax.set_xticks(range(len(tested_bws)))
            ax.set_xticklabels(bw_labels)
            ax.set_yticks(range(len(tested_sfs)))
            ax.set_yticklabels(sf_labels)
            ax.set_xlabel("Bandwidth (kHz)")
            if idx == 0:
                ax.set_ylabel("Spreading Factor")
            ax.set_title(f"{freq} MHz")

        # 共用 colorbar
        fig.subplots_adjust(right=0.88)
        cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.7])
        cbar = fig.colorbar(im, cax=cbar_ax)
        cbar.set_label(f"{title_prefix} ({unit})")

        fig.suptitle(
            f"{title_prefix} vs Spreading Factor & Bandwidth",
            fontsize=13, fontweight="bold", y=1.02,
        )

        fig.savefig(CHARTS_DIR / f"{filename_prefix}.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_vs_sf(self, tested_freqs, tested_sfs, tested_bws):
        """PRR vs SF 折线图，每个频率一子图"""
        n_freqs = len(tested_freqs)
        fig, axes = plt.subplots(
            1, n_freqs,
            figsize=(5 * n_freqs, 4),
            squeeze=False,
            sharey=True,
        )
        axes = axes.flatten()

        # 论文风格的标记和颜色
        markers = ["o", "s", "^", "D", "v", "P", "*", "X"]
        colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00"]

        for idx, freq in enumerate(tested_freqs):
            ax = axes[idx]

            for bi, bw in enumerate(tested_bws):
                prr_vals = []
                sf_vals = []
                for sf in tested_sfs:
                    matching = [
                        r for r in self.results
                        if r["frequency"] == freq
                        and r["sf"] == sf
                        and r["bandwidth"] == bw
                    ]
                    if matching:
                        r = matching[0]
                        prr_vals.append(r["prr"])
                        sf_vals.append(sf)

                if sf_vals:
                    bw_label = (f"{bw:.0f} kHz" if bw < 1000
                                else f"{bw/1000:.0f} MHz")
                    ax.plot(
                        sf_vals, prr_vals,
                        marker=markers[bi % len(markers)],
                        color=colors[bi % len(colors)],
                        linewidth=1.5,
                        markersize=5,
                        label=bw_label,
                    )

            ax.set_xlabel("Spreading Factor")
            if idx == 0:
                ax.set_ylabel("PRR (%)")
            ax.set_title(f"{freq} MHz")
            ax.set_ylim(-5, 105)
            ax.set_xticks(tested_sfs)
            ax.set_xticklabels([f"{sf}" for sf in tested_sfs])
            ax.grid(True, alpha=0.3, linestyle="--")
            ax.legend(loc="lower left", framealpha=0.9)

        fig.suptitle(
            "Packet Reception Rate vs Spreading Factor",
            fontsize=13, fontweight="bold",
        )
        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_vs_sf.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_prr_band_comparison(self, tested_freqs, tested_sfs, tested_bws):
        """跨频段 PRR 对比柱状图 — 按 SF 分组，每频段一个柱子"""
        fig, ax = plt.subplots(figsize=(max(8, len(tested_sfs) * 1.2), 5))

        n_bands = len(tested_freqs)
        bar_width = 0.8 / n_bands
        colors = ["#377eb8", "#e41a1c", "#4daf4a", "#984ea3", "#ff7f00"]

        x = np.arange(len(tested_sfs))

        for fi, freq in enumerate(tested_freqs):
            avg_prr_per_sf = []
            for sf in tested_sfs:
                matching = [
                    r["prr"] for r in self.results
                    if r["frequency"] == freq
                    and r["sf"] == sf
                    and r["status"] == "OK"
                ]
                avg_prr_per_sf.append(np.mean(matching) if matching else 0)

            offset = (fi - (n_bands - 1) / 2) * bar_width
            ax.bar(
                x + offset,
                avg_prr_per_sf,
                width=bar_width,
                color=colors[fi % len(colors)],
                edgecolor="black",
                linewidth=0.5,
                label=f"{freq} MHz",
                alpha=0.85,
            )

        ax.set_xlabel("Spreading Factor")
        ax.set_ylabel("Average PRR (%)")
        ax.set_title(
            "PRR Comparison Across Frequency Bands\n"
            "(averaged over all bandwidths)",
            fontweight="bold",
        )
        ax.set_xticks(x)
        ax.set_xticklabels([f"SF{sf}" for sf in tested_sfs])
        ax.set_ylim(0, 110)
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y", linestyle="--")

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "prr_band_comparison.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_crc_error_heatmaps(self, tested_freqs, tested_sfs, tested_bws):
        """CRC 错误率热力图"""
        # 先检查是否有 CRC 失败数据
        has_crc_fail = any(r.get("crc_fail", 0) > 0 for r in self.results)
        if not has_crc_fail:
            return

        n_freqs = len(tested_freqs)
        fig, axes = plt.subplots(
            1, n_freqs,
            figsize=(5 * n_freqs + 1, 4.5),
            squeeze=False,
        )
        axes = axes.flatten()

        bw_labels = [f"{bw:.0f}" if bw < 1000 else f"{bw/1000:.0f}M"
                     for bw in tested_bws]
        sf_labels = [f"SF{sf}" for sf in tested_sfs]

        crc_cmap = LinearSegmentedColormap.from_list(
            "crc_err", ["#1a9850", "#fee08b", "#d73027"]
        )

        for idx, freq in enumerate(tested_freqs):
            ax = axes[idx]
            matrix = self._build_matrix(freq, tested_sfs, tested_bws, "crc_error_rate")

            # 对 status != OK 的组合也计算 CRC 错误率
            for r in self.results:
                if r["frequency"] != freq:
                    continue
                if r["status"] == "NO_RESPONSE":
                    try:
                        si = tested_sfs.index(r["sf"])
                        bi = tested_bws.index(r["bandwidth"])
                        matrix[si][bi] = np.nan
                    except ValueError:
                        pass

            vmax = np.nanmax(matrix) if not np.all(np.isnan(matrix)) else 100

            im = ax.imshow(
                matrix, cmap=crc_cmap, aspect="auto",
                vmin=0, vmax=max(vmax, 1),
                interpolation="nearest",
            )

            for i in range(len(tested_sfs)):
                for j in range(len(tested_bws)):
                    val = matrix[i][j]
                    if np.isnan(val):
                        text = "N/A"
                        color = "gray"
                    else:
                        text = f"{val:.1f}"
                        color = "white" if val > vmax * 0.6 else "black"
                    ax.text(j, i, text, ha="center", va="center",
                            fontsize=7, color=color, fontweight="bold")

            ax.set_xticks(range(len(tested_bws)))
            ax.set_xticklabels(bw_labels)
            ax.set_yticks(range(len(tested_sfs)))
            ax.set_yticklabels(sf_labels)
            ax.set_xlabel("Bandwidth (kHz)")
            if idx == 0:
                ax.set_ylabel("Spreading Factor")
            ax.set_title(f"{freq} MHz")

        fig.subplots_adjust(right=0.88)
        cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.7])
        cbar = fig.colorbar(im, cax=cbar_ax)
        cbar.set_label("CRC Error Rate (%)")

        fig.suptitle(
            "CRC Error Rate vs Spreading Factor & Bandwidth",
            fontsize=13, fontweight="bold", y=1.02,
        )

        fig.savefig(CHARTS_DIR / "crc_error_heatmap.png", bbox_inches="tight")
        plt.close(fig)

    # ---- 工具方法 ----

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _signal_handler(self, sig, frame):
        print(f"\n[{self._now()}] 收到中断信号，正在保存结果...")
        self._interrupted = True

    def cleanup(self):
        """清理资源"""
        if self.tx_module:
            self.tx_module.close()
        if self.rx_module:
            self.rx_module.close()


# ============================================================
#  主入口
# ============================================================

def main():
    tester = PRRTester()

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
