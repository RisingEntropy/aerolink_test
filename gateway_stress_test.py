#!/usr/bin/env python3
"""
AeroLink Gateway 压力测试 (4TX/4RX 自发自收)

通过 MQTT 同时在 4 个 TX 通道发包，4 个 RX 通道收包（自发自收），
测量单通道和总聚合吞吐量。

测试模式:
  burst    - 4 个通道同时连续发包，统一等待收包
  rampup   - 逐步增加并发通道数 (1→2→3→4)，对比吞吐量变化

用法:
  python gateway_stress_test.py burst
  python gateway_stress_test.py rampup
  python gateway_stress_test.py burst --replot <目录>
"""

import json
import ssl
import time
import base64
import os
import struct
import signal
import argparse
import csv
import threading
from datetime import datetime
from collections import defaultdict
from pathlib import Path
from threading import Lock, Event, Barrier

import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

try:
    from node_comm import calc_time_on_air
except ImportError:
    def calc_time_on_air(sf, bw_khz, payload_len, preamble=8, cr=1):
        rs = (bw_khz * 1000) / (2**sf)
        t_sym = 1 / rs
        t_preamble = (preamble + 4.25) * t_sym
        payload_symb_nb = 8 + max(
            np.ceil((8 * payload_len - 4 * sf + 28 + 16) / (4 * sf)) * (cr + 4), 0
        )
        t_payload = payload_symb_nb * t_sym
        return t_preamble + t_payload


# ============================================================
#  MQTT 配置
# ============================================================

MQTT_BROKER = "hymqtt.hydeng.cn"
MQTT_PORT = 443
MQTT_USERNAME = "admin"
MQTT_PASSWORD = "admin"
NODE_HUB_ID = "2650C2DE7B49EDFE"
TOPIC_DOWNLINK = f"nodehub/{NODE_HUB_ID}/data/downlink"
TOPIC_UPLINK = f"nodehub/{NODE_HUB_ID}/data/uplink"

# ============================================================
#  测试参数
# ============================================================

NUM_CHANNELS = 4
# 每个通道使用不同频率，避免自干扰
CHANNEL_FREQUENCIES = [1300.0, 1310.0, 1320.0, 1330.0]

SF_LIST = [5, 6, 7, 8]
PAYLOAD_SIZES = [250]
PACKETS_PER_CHANNEL = 100         # 每通道每配置发包数
BANDWIDTH_KHZ = 125
TX_POWER_DBM = 22
CODING_RATE = 1  # 4/5

# 超时参数
TIMEOUT_SAFETY_FACTOR = 10.0
MIN_TIMEOUT_WINDOW = 60.0

# Payload 校验标记
MAGIC = b"\xAE\x70\x11\x4E"


# ============================================================
#  Payload 构造 / 校验
# ============================================================

def build_tagged_payload(channel: int, seq: int, payload_size: int) -> bytes:
    """构造带 magic + channel + seq 校验头的 payload"""
    header = MAGIC + struct.pack("<BH", channel, seq)  # 4+1+2 = 7 bytes
    if payload_size <= len(header):
        return header[:payload_size]
    padding = os.urandom(payload_size - len(header))
    return header + padding


def verify_tagged_payload(data_b64: str) -> tuple[int, int] | None:
    """校验 uplink payload，返回 (channel, seq) 或 None"""
    try:
        raw = base64.b64decode(data_b64)
    except Exception:
        return None
    if len(raw) < 7:
        return None
    if raw[:4] != MAGIC:
        return None
    channel = raw[4]
    seq = struct.unpack("<H", raw[5:7])[0]
    return (channel, seq)


# ============================================================
#  GatewayStressTester
# ============================================================

class GatewayStressTester:
    def __init__(self, mode: str = "burst", replot_dir: str = None):
        self.mode = mode
        self.replot_dir = replot_dir

        self.lock = Lock()
        self.client = None
        self.connected = False
        self.running = True

        # 收包追踪: channel -> [packet_info, ...]
        self._rx_packets: dict[int, list] = defaultdict(list)
        self._collecting = False

        # 期望集合: (channel, seq)
        self._expected: set[tuple[int, int]] = set()
        self._received: set[tuple[int, int]] = set()
        self._matched_count = 0

        self.results = []

    # ============================================================
    #  MQTT 回调
    # ============================================================

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print(f"[{self._now()}] MQTT 连接成功")
            client.subscribe(TOPIC_UPLINK)
            print(f"[{self._now()}] 已订阅: {TOPIC_UPLINK}")
            self.connected = True
        else:
            print(f"[{self._now()}] MQTT 连接失败: {reason_code}")

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[{self._now()}] MQTT 断开连接: {reason_code}")
        self.connected = False

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if msg.topic != TOPIC_UPLINK:
                return

            recv_time = time.time()
            data_b64 = payload.get("data", "")
            result = verify_tagged_payload(data_b64)

            if result is None:
                return  # 非测试包

            channel, seq = result
            key = (channel, seq)

            if key not in self._expected:
                return  # 不属于当前测试

            rssi = payload.get("rssi", 0)
            snr = payload.get("snr", 0)

            with self.lock:
                is_dup = key in self._received
                if not is_dup:
                    self._received.add(key)
                self._matched_count += 1

            if self._collecting and not is_dup:
                pkt_info = {
                    "channel": channel,
                    "seq": seq,
                    "recv_time": recv_time,
                    "rssi": rssi,
                    "snr": snr,
                }
                with self.lock:
                    self._rx_packets[channel].append(pkt_info)
                    total = sum(len(v) for v in self._rx_packets.values())

                if total % 50 == 0:
                    print(f"[{self._now()}]   收到 {total} 包 (ch{channel} seq={seq})")

        except Exception as e:
            print(f"[{self._now()}] 消息处理错误: {e}")

    # ============================================================
    #  MQTT 连接
    # ============================================================

    def connect_mqtt(self):
        self.client = mqtt.Client(
            client_id=f"stress_{self.mode}_{int(time.time())}",
            transport="websockets",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.client.tls_insecure_set(True)
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.ws_set_options(path="/mqtt")

        print(f"[{self._now()}] 连接到 MQTT Broker...")
        while self.running:
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                self.client.loop_start()
                deadline = time.time() + 10
                while not self.connected and time.time() < deadline:
                    time.sleep(0.1)
                if self.connected:
                    return True
                self.client.loop_stop()
            except Exception as e:
                print(f"[{self._now()}] 连接失败，5秒后重试: {e}")
                time.sleep(5)
        return False

    # ============================================================
    #  发包: 单通道线程
    # ============================================================

    def _send_channel_burst(
        self, channel: int, sf: int, payload_size: int,
        num_packets: int, send_times: dict, barrier: Barrier = None,
    ):
        """在单个通道上 burst 发送 num_packets 包"""
        freq = CHANNEL_FREQUENCIES[channel]
        times = []

        # 等待所有通道线程同步启动
        if barrier:
            barrier.wait()

        for seq in range(num_packets):
            if not self.running:
                break
            tagged = build_tagged_payload(channel, seq, payload_size)
            encoded = base64.b64encode(tagged).decode("ascii")
            msg = json.dumps({
                "message_id": f"stress-ch{channel}-{seq:04d}",
                "target_frequency": freq,
                "target_sf": sf,
                "tx_power": TX_POWER_DBM,
                "data": encoded,
            })
            self.client.publish(TOPIC_DOWNLINK, msg)
            times.append(time.time())

        with self.lock:
            send_times[channel] = times

    # ============================================================
    #  测试执行: burst 模式 (指定通道数)
    # ============================================================

    def run_multi_channel_burst(
        self, sf: int, payload_size: int, num_channels: int,
    ) -> dict:
        """在 num_channels 个通道上同时 burst 发包"""
        total_packets = PACKETS_PER_CHANNEL * num_channels

        print(f"\n[{self._now()}] [Burst] SF={sf}, Payload={payload_size}B, "
              f"通道数={num_channels}, 每通道 {PACKETS_PER_CHANNEL} 包, "
              f"总计 {total_packets} 包")

        # 重置状态
        with self.lock:
            self._rx_packets = defaultdict(list)
            self._received = set()
            self._matched_count = 0

        self._expected = set()
        for ch in range(num_channels):
            for seq in range(PACKETS_PER_CHANNEL):
                self._expected.add((ch, seq))
        self._collecting = True

        # 多线程并发发包
        send_times: dict[int, list] = {}
        barrier = Barrier(num_channels)
        threads = []
        for ch in range(num_channels):
            t = threading.Thread(
                target=self._send_channel_burst,
                args=(ch, sf, payload_size, PACKETS_PER_CHANNEL, send_times, barrier),
            )
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        # 等待收包
        toa = calc_time_on_air(sf=sf, bw_khz=BANDWIDTH_KHZ, payload_len=payload_size,
                               preamble=8, cr=CODING_RATE)
        wait_time = min(
            max(MIN_TIMEOUT_WINDOW, toa * PACKETS_PER_CHANNEL * TIMEOUT_SAFETY_FACTOR),
            600,
        )

        wait_start = time.time()
        last_count = 0
        no_new_time = 0

        while time.time() - wait_start < wait_time and self.running:
            time.sleep(1.0)
            with self.lock:
                current_count = sum(len(v) for v in self._rx_packets.values())
            if current_count >= total_packets:
                break
            if current_count == last_count:
                no_new_time += 1
                if current_count > 0 and no_new_time >= 15:
                    break
            else:
                no_new_time = 0
                last_count = current_count

        self._collecting = False

        # 构造结果
        with self.lock:
            rx_packets = dict(self._rx_packets)
            matched = self._matched_count

        return self._build_result(
            sf, payload_size, num_channels, send_times, rx_packets, toa,
        )

    # ============================================================
    #  结果构造
    # ============================================================

    def _build_result(
        self, sf, payload_size, num_channels, send_times, rx_packets, toa,
    ) -> dict:
        total_sent = PACKETS_PER_CHANNEL * num_channels
        total_received = sum(len(pkts) for pkts in rx_packets.values())
        prr = total_received / total_sent * 100 if total_sent > 0 else 0

        # 总吞吐量: 从最早发包到最后收包的时间
        all_send = []
        for times in send_times.values():
            all_send.extend(times)
        all_recv = []
        for pkts in rx_packets.values():
            all_recv.extend(p["recv_time"] for p in pkts)

        if all_send and all_recv:
            first_send = min(all_send)
            last_recv = max(all_recv)
            total_duration = last_recv - first_send
            total_throughput_bps = (total_received * payload_size * 8) / total_duration if total_duration > 0 else 0
        else:
            total_duration = 0
            total_throughput_bps = 0

        # 每通道吞吐量
        per_channel = {}
        for ch in range(num_channels):
            ch_send = send_times.get(ch, [])
            ch_recv = rx_packets.get(ch, [])
            ch_received = len(ch_recv)
            if ch_send and ch_recv:
                ch_first_send = ch_send[0]
                ch_last_recv = max(p["recv_time"] for p in ch_recv)
                ch_dur = ch_last_recv - ch_first_send
                ch_bps = (ch_received * payload_size * 8) / ch_dur if ch_dur > 0 else 0
            else:
                ch_dur = 0
                ch_bps = 0

            ch_rssi = [p["rssi"] for p in ch_recv] if ch_recv else []
            ch_snr = [p["snr"] for p in ch_recv] if ch_recv else []

            per_channel[ch] = {
                "sent": PACKETS_PER_CHANNEL,
                "received": ch_received,
                "prr": ch_received / PACKETS_PER_CHANNEL * 100,
                "duration": ch_dur,
                "throughput_bps": ch_bps,
                "avg_rssi": float(np.mean(ch_rssi)) if ch_rssi else 0,
                "avg_snr": float(np.mean(ch_snr)) if ch_snr else 0,
                "frequency": CHANNEL_FREQUENCIES[ch],
            }

        # 理论极限: 单通道极限 * 通道数
        single_ch_theoretical = sf * (BANDWIDTH_KHZ * 1000) / (2**sf) * (4 / (4 + CODING_RATE))
        theoretical_total = single_ch_theoretical * num_channels

        result = {
            "sf": sf,
            "payload_size": payload_size,
            "num_channels": num_channels,
            "packets_per_channel": PACKETS_PER_CHANNEL,
            "total_sent": total_sent,
            "total_received": total_received,
            "prr": prr,
            "total_duration": total_duration,
            "total_throughput_bps": total_throughput_bps,
            "theoretical_single_ch_bps": single_ch_theoretical,
            "theoretical_total_bps": theoretical_total,
            "toa_ms": toa * 1000,
            "per_channel": per_channel,
        }

        print(f"[{self._now()}] 结果: 收到={total_received}/{total_sent}, "
              f"PRR={prr:.1f}%, Total Throughput={total_throughput_bps:.1f} bps, "
              f"Theory Limit={theoretical_total:.1f} bps")

        for ch in range(num_channels):
            ch_info = per_channel[ch]
            print(f"  CH{ch} ({CHANNEL_FREQUENCIES[ch]}MHz): "
                  f"{ch_info['received']}/{ch_info['sent']} "
                  f"({ch_info['prr']:.0f}%), "
                  f"{ch_info['throughput_bps']:.0f} bps")

        return result

    # ============================================================
    #  主运行: burst 模式
    # ============================================================

    def run_burst(self):
        """所有 SF x payload_size 组合，全部 4 通道并发"""
        total_configs = len(SF_LIST) * len(PAYLOAD_SIZES)
        config_idx = 0

        for sf in SF_LIST:
            for payload_size in PAYLOAD_SIZES:
                if not self.running:
                    break
                config_idx += 1
                print(f"\n{'='*60}")
                print(f"  配置 {config_idx}/{total_configs}")
                print(f"{'='*60}")
                self.results.append(
                    self.run_multi_channel_burst(sf, payload_size, NUM_CHANNELS)
                )
                if self.running and config_idx < total_configs:
                    time.sleep(5)

    # ============================================================
    #  主运行: rampup 模式
    # ============================================================

    def run_rampup(self):
        """固定 SF 和 payload，逐步增加通道数 1→2→3→4"""
        rampup_sf = 7
        rampup_payload = 80
        channel_counts = [1, 2, 3, 4]

        for n_ch in channel_counts:
            if not self.running:
                break
            print(f"\n{'='*60}")
            print(f"  Ramp-up: {n_ch} 通道")
            print(f"{'='*60}")
            self.results.append(
                self.run_multi_channel_burst(rampup_sf, rampup_payload, n_ch)
            )
            if self.running:
                time.sleep(5)

        # 再测所有 SF（全 4 通道）
        for sf in SF_LIST:
            if not self.running:
                break
            print(f"\n{'='*60}")
            print(f"  全通道 SF={sf}, Payload={rampup_payload}B")
            print(f"{'='*60}")
            self.results.append(
                self.run_multi_channel_burst(sf, rampup_payload, NUM_CHANNELS)
            )
            if self.running:
                time.sleep(5)

    # ============================================================
    #  主入口
    # ============================================================

    def run(self):
        if self.replot_dir:
            self.run_replot()
            return

        print(f"[{self._now()}] AeroLink Gateway 压力测试 [{self.mode.upper()}]")
        print(f"  通道数: {NUM_CHANNELS}")
        print(f"  通道频率: {CHANNEL_FREQUENCIES}")
        print(f"  BW: {BANDWIDTH_KHZ} kHz, CR: 4/{4+CODING_RATE}, TX Power: {TX_POWER_DBM} dBm")

        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, 'running', False))

        if not self.connect_mqtt():
            return

        if self.mode == "burst":
            self.run_burst()
        elif self.mode == "rampup":
            self.run_rampup()

        self.client.loop_stop()
        self.client.disconnect()

        if self.results:
            self.generate_outputs()

    # ============================================================
    #  重绘
    # ============================================================

    def run_replot(self):
        print(f"\n[{self._now()}] 从目录重新加载数据: {self.replot_dir}")
        json_path = self._output_dir() / "stress_raw.json"

        if not json_path.exists():
            print(f"错误: 找不到 {json_path}")
            return

        with open(json_path, "r", encoding="utf-8") as f:
            self.results = json.load(f)
        print(f"加载 {len(self.results)} 条记录")

        self._generate_charts()
        self._generate_text_report()
        print(f"重绘完成: {self._output_dir()}")

    # ============================================================
    #  输出生成
    # ============================================================

    def _output_dir(self) -> Path:
        if self.replot_dir:
            return Path(self.replot_dir)
        return Path(f"stress_output_{self.mode}")

    def _charts_dir(self) -> Path:
        return self._output_dir() / "charts"

    def generate_outputs(self):
        self._output_dir().mkdir(exist_ok=True)
        self._charts_dir().mkdir(exist_ok=True)
        self._save_csv()
        self._save_json()
        self._generate_charts()
        self._generate_text_report()

    def _save_csv(self):
        csv_path = self._output_dir() / "stress_results.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "SF", "payload_size", "num_channels",
                "total_sent", "total_received", "PRR(%)",
                "total_throughput_bps", "theoretical_total_bps",
                "total_duration_s",
                "ch0_bps", "ch1_bps", "ch2_bps", "ch3_bps",
                "ch0_prr", "ch1_prr", "ch2_prr", "ch3_prr",
            ])
            for r in self.results:
                n_ch = r["num_channels"]
                ch_bps = [r["per_channel"].get(str(ch), r["per_channel"].get(ch, {})).get("throughput_bps", 0)
                          for ch in range(4)]
                ch_prr = [r["per_channel"].get(str(ch), r["per_channel"].get(ch, {})).get("prr", 0)
                          for ch in range(4)]
                writer.writerow([
                    r["sf"], r["payload_size"], n_ch,
                    r["total_sent"], r["total_received"],
                    f"{r['prr']:.2f}",
                    f"{r['total_throughput_bps']:.2f}",
                    f"{r['theoretical_total_bps']:.2f}",
                    f"{r['total_duration']:.2f}",
                    *[f"{b:.2f}" for b in ch_bps],
                    *[f"{p:.2f}" for p in ch_prr],
                ])

    def _save_json(self):
        json_path = self._output_dir() / "stress_raw.json"
        with open(json_path, "w") as f:
            json.dump(self.results, f, indent=2, default=str)

    def _generate_charts(self):
        plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        charts_dir = self._charts_dir()
        charts_dir.mkdir(exist_ok=True)

        self._plot_total_throughput_vs_sf()
        self._plot_actual_vs_theoretical()
        self._plot_per_channel_throughput()
        self._plot_prr_heatmap()

        # Ramp-up 模式专属图
        rampup_results = [r for r in self.results if r.get("payload_size") == 80]
        channel_counts = sorted(set(r["num_channels"] for r in rampup_results))
        if len(channel_counts) > 1:
            self._plot_rampup(rampup_results)

    def _plot_total_throughput_vs_sf(self):
        """总吞吐量 vs SF (不同 payload)"""
        charts_dir = self._charts_dir()

        # 只取全通道的数据
        full_ch = [r for r in self.results if r["num_channels"] == NUM_CHANNELS]
        if not full_ch:
            return

        payload_sizes = sorted(set(r["payload_size"] for r in full_ch))
        sf_list = sorted(set(r["sf"] for r in full_ch))

        fig, ax = plt.subplots(figsize=(10, 6))
        for ps in payload_sizes:
            sfs = [r["sf"] for r in full_ch if r["payload_size"] == ps]
            tps = [r["total_throughput_bps"] for r in full_ch if r["payload_size"] == ps]
            if sfs:
                ax.plot(sfs, tps, "o-", label=f"{ps}B payload", linewidth=2, markersize=7)

        ax.set_xlabel("Spreading Factor")
        ax.set_ylabel("Total Throughput (bps)")
        ax.set_title(f"Aggregate Throughput vs SF ({NUM_CHANNELS} Channels)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        if sf_list:
            ax.set_xticks(sf_list)
        plt.tight_layout()
        plt.savefig(charts_dir / "total_throughput_vs_sf.png", dpi=150)
        plt.close()

    def _plot_actual_vs_theoretical(self):
        """实际 vs 理论吞吐量对比柱状图"""
        charts_dir = self._charts_dir()

        full_ch = [r for r in self.results if r["num_channels"] == NUM_CHANNELS]
        if not full_ch:
            return

        sf_list = sorted(set(r["sf"] for r in full_ch))
        n_sf = len(sf_list)
        if n_sf == 0:
            return

        cols = min(n_sf, 2)
        rows = (n_sf + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5 * rows), squeeze=False)
        axes_flat = axes.flatten()

        for idx, sf in enumerate(sf_list):
            ax = axes_flat[idx]
            sf_results = sorted(
                [r for r in full_ch if r["sf"] == sf],
                key=lambda r: r["payload_size"],
            )
            if not sf_results:
                continue

            ps_labels = [str(r["payload_size"]) for r in sf_results]
            actual = [r["total_throughput_bps"] for r in sf_results]
            theoretical = [r["theoretical_total_bps"] for r in sf_results]

            x = np.arange(len(ps_labels))
            width = 0.35
            ax.bar(x - width / 2, actual, width, label="Actual Total", color="steelblue")
            ax.bar(x + width / 2, theoretical, width, label=f"Theoretical ({NUM_CHANNELS}ch)", color="coral")
            ax.set_xlabel("Payload Size (bytes)")
            ax.set_ylabel("Throughput (bps)")
            ax.set_title(f"SF{sf}: Actual vs Theoretical ({NUM_CHANNELS} Channels)")
            ax.set_xticks(x)
            ax.set_xticklabels(ps_labels)
            ax.legend()
            ax.grid(True, alpha=0.3, axis="y")

        for j in range(len(sf_list), len(axes_flat)):
            axes_flat[j].set_visible(False)

        fig.suptitle("Aggregate Throughput: Actual vs Theoretical", fontsize=14)
        plt.tight_layout()
        plt.savefig(charts_dir / "actual_vs_theoretical.png", dpi=150)
        plt.close()

    def _plot_per_channel_throughput(self):
        """每通道吞吐量 stacked bar"""
        charts_dir = self._charts_dir()

        full_ch = [r for r in self.results if r["num_channels"] == NUM_CHANNELS]
        if not full_ch:
            return

        # 选一个 payload_size 画
        ps_target = 80
        sf_results = sorted(
            [r for r in full_ch if r["payload_size"] == ps_target],
            key=lambda r: r["sf"],
        )
        if not sf_results:
            sf_results = sorted(full_ch, key=lambda r: r["sf"])

        fig, ax = plt.subplots(figsize=(10, 6))
        x = np.arange(len(sf_results))
        width = 0.6

        bottoms = np.zeros(len(sf_results))
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
        for ch in range(NUM_CHANNELS):
            ch_bps = []
            for r in sf_results:
                ch_data = r["per_channel"].get(str(ch), r["per_channel"].get(ch, {}))
                ch_bps.append(ch_data.get("throughput_bps", 0))
            ax.bar(x, ch_bps, width, bottom=bottoms,
                   label=f"CH{ch} ({CHANNEL_FREQUENCIES[ch]}MHz)",
                   color=colors[ch], edgecolor="white", linewidth=0.5)
            bottoms += np.array(ch_bps)

        labels = [f"SF{r['sf']}\n{r['payload_size']}B" for r in sf_results]
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_xlabel("Configuration")
        ax.set_ylabel("Throughput (bps)")
        ax.set_title("Per-Channel Throughput Breakdown")
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")
        plt.tight_layout()
        plt.savefig(charts_dir / "per_channel_throughput.png", dpi=150)
        plt.close()

    def _plot_prr_heatmap(self):
        """PRR 热力图: SF vs Payload"""
        charts_dir = self._charts_dir()

        full_ch = [r for r in self.results if r["num_channels"] == NUM_CHANNELS]
        if not full_ch:
            return

        sf_list = sorted(set(r["sf"] for r in full_ch))
        ps_list = sorted(set(r["payload_size"] for r in full_ch))
        if not sf_list or not ps_list:
            return

        prr_matrix = np.full((len(sf_list), len(ps_list)), np.nan)
        for r in full_ch:
            i = sf_list.index(r["sf"])
            j = ps_list.index(r["payload_size"])
            prr_matrix[i, j] = r["prr"]

        fig, ax = plt.subplots(figsize=(8, 5))
        im = ax.imshow(prr_matrix, cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")

        ax.set_xticks(range(len(ps_list)))
        ax.set_xticklabels([str(p) for p in ps_list])
        ax.set_yticks(range(len(sf_list)))
        ax.set_yticklabels([f"SF{s}" for s in sf_list])
        ax.set_xlabel("Payload Size (bytes)")
        ax.set_ylabel("Spreading Factor")
        ax.set_title(f"Packet Reception Rate (%) — {NUM_CHANNELS} Channels")

        for i in range(len(sf_list)):
            for j in range(len(ps_list)):
                val = prr_matrix[i, j]
                if not np.isnan(val):
                    ax.text(j, i, f"{val:.0f}%", ha="center", va="center",
                            color="black" if val > 50 else "white", fontsize=10)

        fig.colorbar(im, ax=ax, label="PRR (%)")
        plt.tight_layout()
        plt.savefig(charts_dir / "prr_heatmap.png", dpi=150)
        plt.close()

    def _plot_rampup(self, rampup_results):
        """Ramp-up: 吞吐量随通道数变化"""
        charts_dir = self._charts_dir()

        sorted_results = sorted(rampup_results, key=lambda r: r["num_channels"])
        # 去重，只保留每个 num_channels 的第一个结果
        seen = set()
        unique = []
        for r in sorted_results:
            n = r["num_channels"]
            if n not in seen:
                seen.add(n)
                unique.append(r)

        if len(unique) < 2:
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

        n_chs = [r["num_channels"] for r in unique]
        total_tp = [r["total_throughput_bps"] for r in unique]
        prrs = [r["prr"] for r in unique]

        # 左图: 吞吐量 vs 通道数
        ax1.plot(n_chs, total_tp, "b-o", linewidth=2, markersize=10, label="Actual")

        # 理论线性增长
        if unique[0]["total_throughput_bps"] > 0:
            base = unique[0]["total_throughput_bps"]
            linear = [base * n for n in n_chs]
            ax1.plot(n_chs, linear, "r--", linewidth=1.5, label="Linear Scaling")

        ax1.set_xlabel("Number of Channels")
        ax1.set_ylabel("Total Throughput (bps)")
        ax1.set_title("Throughput Scaling with Channel Count")
        ax1.set_xticks(n_chs)
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 右图: PRR vs 通道数
        ax2.bar(n_chs, prrs, color="steelblue", edgecolor="black", alpha=0.8)
        ax2.set_xlabel("Number of Channels")
        ax2.set_ylabel("PRR (%)")
        ax2.set_title("PRR vs Channel Count")
        ax2.set_xticks(n_chs)
        ax2.set_ylim(0, 105)
        ax2.grid(True, alpha=0.3, axis="y")

        for i, (n, p) in enumerate(zip(n_chs, prrs)):
            ax2.text(n, p + 1, f"{p:.1f}%", ha="center", va="bottom", fontsize=10)

        fig.suptitle("Gateway Ramp-Up Stress Test", fontsize=14)
        plt.tight_layout()
        plt.savefig(charts_dir / "rampup.png", dpi=150)
        plt.close()

    def _generate_text_report(self):
        report_path = self._output_dir() / "stress_report.txt"
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 80 + "\n")
            f.write("AeroLink Gateway Stress Test Report\n")
            f.write(f"生成时间: {self._now()}\n")
            f.write(f"模式: {self.mode}\n")
            f.write("=" * 80 + "\n\n")

            f.write("【测试配置】\n")
            f.write(f"  通道数: {NUM_CHANNELS}\n")
            f.write(f"  通道频率: {CHANNEL_FREQUENCIES}\n")
            f.write(f"  BW: {BANDWIDTH_KHZ} kHz\n")
            f.write(f"  CR: 4/{4+CODING_RATE}\n")
            f.write(f"  TX Power: {TX_POWER_DBM} dBm\n")
            f.write(f"  每通道包数: {PACKETS_PER_CHANNEL}\n\n")

            f.write("【聚合结果汇总】\n")
            f.write(f"{'SF':>4} {'Payload':>8} {'CH':>4} {'Sent':>6} {'Recv':>6} "
                    f"{'PRR%':>8} {'Throughput(bps)':>16} {'Theory(bps)':>14} "
                    f"{'Duration(s)':>12}\n")
            f.write("-" * 90 + "\n")

            for r in self.results:
                f.write(
                    f"{r['sf']:>4} {r['payload_size']:>8} {r['num_channels']:>4} "
                    f"{r['total_sent']:>6} {r['total_received']:>6} "
                    f"{r['prr']:>7.1f}% "
                    f"{r['total_throughput_bps']:>15.1f} "
                    f"{r['theoretical_total_bps']:>13.1f} "
                    f"{r['total_duration']:>11.2f}\n"
                )

            f.write("\n\n【每通道详情】\n")
            for r in self.results:
                f.write(f"\n  SF={r['sf']}, Payload={r['payload_size']}B, "
                        f"Channels={r['num_channels']}\n")
                for ch in range(r["num_channels"]):
                    ch_data = r["per_channel"].get(str(ch), r["per_channel"].get(ch, {}))
                    f.write(
                        f"    CH{ch} ({ch_data.get('frequency', '?')}MHz): "
                        f"recv={ch_data.get('received', 0)}/{ch_data.get('sent', 0)} "
                        f"PRR={ch_data.get('prr', 0):.1f}% "
                        f"bps={ch_data.get('throughput_bps', 0):.0f} "
                        f"RSSI={ch_data.get('avg_rssi', 0):.1f} "
                        f"SNR={ch_data.get('avg_snr', 0):.1f}\n"
                    )

            # 最佳结果
            if self.results:
                best = max(self.results, key=lambda r: r["total_throughput_bps"])
                f.write(f"\n\n【最佳吞吐量】\n")
                f.write(f"  SF={best['sf']}, Payload={best['payload_size']}B, "
                        f"{best['num_channels']}通道\n")
                f.write(f"  总吞吐量: {best['total_throughput_bps']:.1f} bps "
                        f"({best['total_throughput_bps']/1000:.2f} kbps)\n")
                f.write(f"  PRR: {best['prr']:.1f}%\n")

    @staticmethod
    def _now():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# ============================================================
#  主入口
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="AeroLink Gateway 压力测试 (4TX/4RX 自发自收)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python gateway_stress_test.py burst           # 全通道并发 burst
  python gateway_stress_test.py rampup          # 逐步增加通道数
  python gateway_stress_test.py burst --replot stress_output_burst
        """,
    )
    parser.add_argument(
        "mode", choices=["burst", "rampup"], nargs="?", default="burst",
        help="测试模式: burst (全通道并发) 或 rampup (逐步增加通道数)",
    )
    parser.add_argument(
        "--replot", type=str, metavar="DIR",
        help="离线重绘: 传入输出目录，重新生成图表",
    )
    args = parser.parse_args()

    tester = GatewayStressTester(mode=args.mode, replot_dir=args.replot)
    tester.run()
