#!/usr/bin/env python3
"""
AeroLink Gateway Throughput Test

通过 MQTT 控制 gateway 发包（downlink）并接收回包（uplink），
测试不同 SF 和不同 payload 大小下的吞吐量。

两种运行模式:
  1. 在线测试 (默认): 
     burst    - 连续发完所有包，再统一等待收包
     pingpong - 发一个包，等收到回包后再发下一个
  2. 离线重绘:
     使用 --replot <目录名> 从已有 json 数据重新生成图表，并根据当前代码重新计算理论速率。
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
from datetime import datetime
from collections import defaultdict
from pathlib import Path
from threading import Lock, Event

import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

# 简化的 ToA 计算回退，兼容没有 node_comm 模块的情况
try:
    from node_comm import calc_time_on_air
except ImportError:
    def calc_time_on_air(sf, bw_khz, payload_len, preamble=8, cr=1):
        """简易版 LoRa ToA 计算"""
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
#  测试参数 (在线测试时生效)
# ============================================================

SF_LIST = [5,6,7,8]
PAYLOAD_SIZES = [20,40,80,160, 250]  # bytes
PACKETS_PER_CONFIG = 100
FREQUENCY_MHZ = 1300.0
BANDWIDTH_KHZ = 125
TX_POWER_DBM = 22
CODING_RATE = 1  # 4/5

# ============================================================
#  理论速率计算参数
# ============================================================
# 如果你的网关实际测出很高的数据，说明具备多通道并发接收能力。
# 调整此数值（例如 8），离线重绘时理论柱状图会自动按此倍数重新计算。
GATEWAY_CHANNELS = 1

# Burst 模式超时参数
TIMEOUT_SAFETY_FACTOR = 10.0
MIN_TIMEOUT_WINDOW = 60.0

# Ping-pong 模式单包超时（秒）
PINGPONG_TIMEOUT = 10.0

# ============================================================
#  Payload 校验标记
# ============================================================
MAGIC = b"\xAE\x70\x11\x4E"  # "AEro LINk" 缩写


def build_tagged_payload(seq: int, payload_size: int) -> bytes:
    """构造带 magic + seq 校验头的 payload"""
    header = MAGIC + struct.pack("<I", seq)  # 8 bytes
    if payload_size <= len(header):
        return header[:payload_size]
    padding = os.urandom(payload_size - len(header))
    return header + padding


def verify_tagged_payload(data_b64: str) -> int | None:
    """校验 uplink 中的 base64 data 字段"""
    try:
        raw = base64.b64decode(data_b64)
    except Exception:
        return None
    if len(raw) < 8:
        return None
    if raw[:4] != MAGIC:
        return None
    seq = struct.unpack("<I", raw[4:8])[0]
    return seq


class ThroughputTester:
    def __init__(self, mode: str = "burst", replot_dir: str = None):
        self.mode = mode
        self.replot_dir = replot_dir
        
        self.lock = Lock()
        self.client = None
        self.connected = False

        self._uplink_packets = []
        self._collecting = False
        self._pingpong_event = Event()
        self._pingpong_result = None

        self._expected_seqs = set()
        self._received_seqs = set()
        self._matched_count = 0
        self._unmatched_count = 0

        self.results = []
        self.running = True

    # ============================================================
    #  核心：理论吞吐量计算公式 (在线与离线共用)
    # ============================================================
    def _calc_theoretical_bps(self, payload_size: int, toa: float) -> float:
        """
        计算理论极限吞吐量：
        单通道应用层极速 = (有效载荷比特) / (单次飞行时间)
        网关综合极速 = 单通道极速 * 网关并发通道数
        """
        single_channel_app_bps = (payload_size * 8) / toa if toa > 0 else 0
        return single_channel_app_bps * GATEWAY_CHANNELS

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
            payload["_recv_time"] = recv_time

            data_b64 = payload.get("data", "")
            seq = verify_tagged_payload(data_b64)
            payload["_verified_seq"] = seq

            rssi = payload.get("rssi", "?")
            snr = payload.get("snr", "?")
            crc = payload.get("crc_ok", "?")
            spreading_factor = payload.get("spreading_factor", "?")
            if seq is not None and seq in self._expected_seqs:
                payload["_matched"] = True
                is_dup = False
                with self.lock:
                    if seq in self._received_seqs:
                        is_dup = True
                    else:
                        self._received_seqs.add(seq)
                    self._matched_count += 1

                if is_dup:
                    print(f"[{self._now()}]   [DUP] seq={seq} 重复收到, RSSI={rssi}, SNR={snr}, CRC={crc}, Spreading Factor={spreading_factor}")
                else:
                    with self.lock:
                        count = len(self._received_seqs)
                    print(f"[{self._now()}]   [OK] seq={seq} ({count}/{PACKETS_PER_CONFIG}),  "
                          f"RSSI={rssi}, SNR={snr}, CRC={crc}, Spreading Factor={spreading_factor}")
            else:
                payload["_matched"] = False
                with self.lock:
                    self._unmatched_count += 1
                if seq is None:
                    pass # 静默忽略非测试包
                else:
                    print(f"[{self._now()}]   [UNEXPECTED] seq={seq} 不在预期集合中, RSSI={rssi}, SNR={snr}, CRC={crc}, Spreading Factor={spreading_factor}")

            if self.mode == "burst" and self._collecting:
                if payload["_matched"]:
                    with self.lock:
                        self._uplink_packets.append(payload)

            elif self.mode == "pingpong":
                if payload["_matched"]:
                    self._pingpong_result = payload
                    self._pingpong_event.set()

        except Exception as e:
            print(f"[{self._now()}] 消息处理错误: {e}")

    # ============================================================
    #  连接与发包逻辑
    # ============================================================

    def connect_mqtt(self):
        self.client = mqtt.Client(
            client_id=f"throughput_{self.mode}_{int(time.time())}",
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

    def build_downlink_message(self, payload_bytes: bytes, sf: int, seq: int) -> str:
        encoded = base64.b64encode(payload_bytes).decode("ascii")
        msg = {
            "message_id": f"msg-{seq:04d}",
            "target_frequency": FREQUENCY_MHZ,
            "target_sf": sf,
            "tx_power": TX_POWER_DBM,
            "data": encoded,
        }
        return json.dumps(msg)

    # ============================================================
    #  Burst & PingPong 测试执行
    # ============================================================

    def run_burst_test(self, sf: int, payload_size: int) -> dict:
        print(f"\n[{self._now()}] [Burst] SF={sf}, Payload={payload_size}B, 发送 {PACKETS_PER_CONFIG} 包")
        with self.lock:
            self._uplink_packets = []
            self._received_seqs = set()
            self._matched_count = 0
            self._unmatched_count = 0
        self._expected_seqs = set(range(PACKETS_PER_CONFIG))
        self._collecting = True

        send_times = []
        for i in range(PACKETS_PER_CONFIG):
            if not self.running: break
            tagged_payload = build_tagged_payload(i, payload_size)
            msg = self.build_downlink_message(tagged_payload, sf, seq=i)
            self.client.publish(TOPIC_DOWNLINK, msg)
            send_times.append(time.time())

        toa = calc_time_on_air(sf=sf, bw_khz=BANDWIDTH_KHZ, payload_len=payload_size, preamble=8, cr=CODING_RATE)
        wait_time = min(max(MIN_TIMEOUT_WINDOW, toa * PACKETS_PER_CONFIG * TIMEOUT_SAFETY_FACTOR), 600)
        
        wait_start = time.time()
        last_count = 0
        no_new_time = 0

        while time.time() - wait_start < wait_time and self.running:
            time.sleep(1.0)
            with self.lock:
                current_count = len(self._uplink_packets)
            if current_count >= PACKETS_PER_CONFIG:
                break
            if current_count == last_count:
                no_new_time += 1
                if current_count > 0 and no_new_time >= 15: break
            else:
                no_new_time = 0
                last_count = current_count

        self._collecting = False
        with self.lock:
            received_packets = list(self._uplink_packets)
            matched = self._matched_count
            unmatched = self._unmatched_count

        return self._build_result(sf, payload_size, send_times, received_packets, matched, unmatched, toa)

    def run_pingpong_test(self, sf: int, payload_size: int) -> dict:
        print(f"\n[{self._now()}] [PingPong] SF={sf}, Payload={payload_size}B, 发送 {PACKETS_PER_CONFIG} 包")
        with self.lock:
            self._received_seqs = set()
            self._matched_count = 0
            self._unmatched_count = 0

        toa = calc_time_on_air(sf=sf, bw_khz=BANDWIDTH_KHZ, payload_len=payload_size, preamble=8, cr=CODING_RATE)
        send_times = []
        received_packets = []

        for i in range(PACKETS_PER_CONFIG):
            if not self.running: break
            self._expected_seqs = {i}
            self._pingpong_event.clear()
            self._pingpong_result = None

            tagged_payload = build_tagged_payload(i, payload_size)
            msg = self.build_downlink_message(tagged_payload, sf, seq=i)
            self.client.publish(TOPIC_DOWNLINK, msg)
            send_times.append(time.time())

            if self._pingpong_event.wait(timeout=PINGPONG_TIMEOUT) and self._pingpong_result:
                received_packets.append(self._pingpong_result)

        with self.lock:
            matched = self._matched_count
            unmatched = self._unmatched_count

        return self._build_result(sf, payload_size, send_times, received_packets, matched, unmatched, toa)

    # ============================================================
    #  结果构造
    # ============================================================

    def _build_result(self, sf, payload_size, send_times, received_packets, matched, unmatched, toa) -> dict:
        packets_received = len(received_packets)
        prr = packets_received / PACKETS_PER_CONFIG * 100

        if packets_received > 0 and send_times:
            first_send = send_times[0]
            last_recv = max(p["_recv_time"] for p in received_packets)
            total_duration = last_recv - first_send
            throughput_bps = (packets_received * payload_size * 8) / total_duration if total_duration > 0 else 0
        else:
            total_duration = 0
            throughput_bps = 0

        # 调用统一的计算函数
        theoretical_throughput_bps = sf*(BANDWIDTH_KHZ*1000) / (2**sf) * (4 / (4 + CODING_RATE))
        rssi_values = [p.get("rssi", 0) for p in received_packets if "rssi" in p]
        snr_values = [p.get("snr", 0) for p in received_packets if "snr" in p]
        
        result = {
            "sf": sf,
            "payload_size": payload_size,
            "packets_sent": PACKETS_PER_CONFIG,
            "packets_received": packets_received,
            "packets_matched": matched,
            "packets_unmatched": unmatched,
            "prr": prr,
            "send_duration": send_times[-1] - send_times[0] if len(send_times) > 1 else 0,
            "total_duration": total_duration,
            "throughput_bps": throughput_bps,
            "theoretical_throughput_bps": theoretical_throughput_bps,
            "toa_ms": toa * 1000,
            "avg_rssi": float(np.mean(rssi_values)) if rssi_values else 0,
            "avg_snr": float(np.mean(snr_values)) if snr_values else 0,
            "send_times": send_times,
            "received_packets": received_packets,
        }

        print(f"[{self._now()}] 结果: 收到={packets_received}/{PACKETS_PER_CONFIG}, "
              f"PRR={prr:.1f}%, Actual Throughput={throughput_bps:.1f} bps "
              f"(Gateway App Limit: {theoretical_throughput_bps:.1f} bps)")
        return result

    # ============================================================
    #  重绘模块 (离线数据读取 + 重新计算理论速率)
    # ============================================================
    
    def run_replot(self):
        print(f"\n[{self._now()}] 正在从目录重新加载数据: {self.replot_dir}")
        json_path = self._output_dir() / "throughput_raw.json"
        
        if not json_path.exists():
            print(f"[{self._now()}] ❌ 错误: 找不到数据文件 {json_path}")
            return

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                self.results = json.load(f)
            print(f"[{self._now()}] ✅ 成功加载 {len(self.results)} 条测试记录。")
            
            global SF_LIST, PAYLOAD_SIZES
            SF_LIST = sorted(list(set(r["sf"] for r in self.results)))
            PAYLOAD_SIZES = sorted(list(set(r["payload_size"] for r in self.results)))
            
            # ---> 核心修改点：重新计算所有数据中的理论速率 <---
            print(f"[{self._now()}] 🔄 正在根据当前代码配置重新计算理论速率 (网关并发通道数={GATEWAY_CHANNELS})...")
            for r in self.results:
                sf = r["sf"]
                payload_size = r["payload_size"]
                toa_sec = r.get("toa_ms", 0) / 1000.0
                
                # 如果历史数据中由于某些原因丢失了 toa，则尝试通过公式补全
                if toa_sec == 0:
                    toa_sec = calc_time_on_air(sf=sf, bw_khz=BANDWIDTH_KHZ, payload_len=payload_size, preamble=8, cr=CODING_RATE)
                    r["toa_ms"] = toa_sec * 1000.0

                # 使用最新的公式覆盖 JSON 中的旧数据
                r["theoretical_throughput_bps"] = sf*(BANDWIDTH_KHZ*1000) / (2**sf) * (4 / (4 + CODING_RATE))

            
            # 生成图表与覆盖报告
            self._generate_charts()
            self._generate_text_report()
            print(f"[{self._now()}] 🎉 图表与报告重绘完成！文件位于: {self._output_dir()}")
        except Exception as e:
            print(f"[{self._now()}] ❌ 读取数据或重绘失败: {e}")

    # ============================================================
    #  主干运行流程
    # ============================================================

    def run(self):
        if self.replot_dir:
            self.run_replot()
            return

        print(f"[{self._now()}] AeroLink Gateway Throughput Test [{self.mode.upper()}]")
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, 'running', False))

        if not self.connect_mqtt():
            return

        total_configs = len(SF_LIST) * len(PAYLOAD_SIZES)
        config_idx = 0

        for sf in SF_LIST:
            for payload_size in PAYLOAD_SIZES:
                if not self.running: break
                config_idx += 1
                
                if self.mode == "burst":
                    self.results.append(self.run_burst_test(sf, payload_size))
                else:
                    self.results.append(self.run_pingpong_test(sf, payload_size))

                if self.running and config_idx < total_configs:
                    time.sleep(1)

        self.client.loop_stop()
        self.client.disconnect()

        if self.results:
            self.generate_outputs()

    # ============================================================
    #  输出生成
    # ============================================================

    def _output_dir(self) -> Path:
        if self.replot_dir:
            return Path(self.replot_dir)
        return Path(f"throughput_output_{self.mode}")

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
        csv_path = self._output_dir() / "throughput_results.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "SF", "payload_size", "packets_sent", "packets_received",
                "PRR(%)", "throughput_bps", "theoretical_throughput_bps",
                "avg_rssi", "avg_snr", "toa_ms", "total_duration_s"
            ])
            for r in self.results:
                writer.writerow([
                    r["sf"], r["payload_size"], r["packets_sent"], r["packets_received"],
                    f"{r['prr']:.2f}", f"{r['throughput_bps']:.2f}", f"{r['theoretical_throughput_bps']:.2f}",
                    f"{r['avg_rssi']:.2f}", f"{r['avg_snr']:.2f}", f"{r['toa_ms']:.2f}", f"{r['total_duration']:.2f}"
                ])

    def _save_json(self):
        json_path = self._output_dir() / "throughput_raw.json"
        export_data = []
        for r in self.results:
            entry = {k: v for k, v in r.items() if k not in ("send_times", "received_packets")}
            entry["send_times"] = r.get("send_times", [])
            entry["received_packets"] = [
                {k: v for k, v in p.items() if k != "data"}
                for p in r.get("received_packets", [])
            ]
            export_data.append(entry)
        with open(json_path, "w") as f:
            json.dump(export_data, f, indent=2, default=str)

    def _generate_charts(self):
        plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        charts_dir = self._charts_dir()
        charts_dir.mkdir(exist_ok=True)
        
        # 1. Throughput vs SF
        _, ax = plt.subplots(figsize=(10, 6))
        for ps in PAYLOAD_SIZES:
            sfs = [r["sf"] for r in self.results if r["payload_size"] == ps]
            tps = [r["throughput_bps"] for r in self.results if r["payload_size"] == ps]
            if sfs: ax.plot(sfs, tps, "o-", label=f"{ps} bytes")
        ax.set_xlabel("Spreading Factor")
        ax.set_ylabel("Actual Throughput (bps)")
        ax.set_title("Actual Throughput vs Spreading Factor")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xticks(SF_LIST)
        plt.tight_layout()
        plt.savefig(charts_dir / "throughput_vs_sf.png", dpi=150)
        plt.close()

        # 2. PRR vs SF
        _, ax = plt.subplots(figsize=(10, 6))
        for ps in PAYLOAD_SIZES:
            sfs = [r["sf"] for r in self.results if r["payload_size"] == ps]
            prrs = [r["prr"] for r in self.results if r["payload_size"] == ps]
            if sfs: ax.plot(sfs, prrs, "o-", label=f"{ps} bytes")
        ax.set_xlabel("Spreading Factor")
        ax.set_ylabel("PRR (%)")
        ax.set_title("Packet Reception Rate vs SF")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xticks(SF_LIST)
        ax.set_ylim(0, 105)
        plt.tight_layout()
        plt.savefig(charts_dir / "prr_vs_sf.png", dpi=150)
        plt.close()

        # 3. Actual vs Theoretical throughput (Gateway App Limit)
        n_sf = len(SF_LIST)
        cols = min(n_sf, 2)
        rows = (n_sf + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5 * rows), squeeze=False)
        axes_flat = axes.flatten()

        for idx, sf in enumerate(SF_LIST):
            ax = axes_flat[idx]
            sf_results = [r for r in self.results if r["sf"] == sf]
            if not sf_results: continue

            ps_labels = [str(r["payload_size"]) for r in sf_results]
            actual = [r["throughput_bps"] for r in sf_results]
            theoretical = [r["theoretical_throughput_bps"] for r in sf_results]

            x = np.arange(len(ps_labels))
            width = 0.35
            ax.bar(x - width / 2, actual, width, label="Actual Application", color="steelblue")
            ax.bar(x + width / 2, theoretical, width, label="Theoretical Gateway Limit", color="coral")
            ax.set_xlabel("Payload Size (bytes)")
            ax.set_ylabel("Throughput (bps)")
            ax.set_title(f"SF{sf}: Actual vs Gateway Limit")
            ax.set_xticks(x)
            ax.set_xticklabels(ps_labels)
            ax.legend()
            ax.grid(True, alpha=0.3, axis="y")

        for j in range(len(SF_LIST), len(axes_flat)):
            axes_flat[j].set_visible(False)

        fig.suptitle("Actual Goodput vs Theoretical Gateway Limit", fontsize=14)
        plt.tight_layout()
        plt.savefig(charts_dir / "actual_vs_theoretical.png", dpi=150)
        plt.close()

    def _generate_text_report(self):
        report_path = self._output_dir() / "throughput_report.txt"
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("AeroLink Gateway Throughput Test Report\n")
            f.write(f"生成时间: {self._now()}\n")
            f.write("=" * 70 + "\n\n")

            f.write("【测试结果汇总】\n")
            f.write(f"{'SF':>4} {'Payload':>8} {'Sent':>6} {'Recv':>6} "
                    f"{'PRR%':>8} {'Goodput(bps)':>14} {'Gw Limit(bps)':>16} {'RSSI':>8} {'SNR':>8}\n")
            f.write("-" * 90 + "\n")

            for r in self.results:
                f.write(
                    f"{r['sf']:>4} {r['payload_size']:>8} {r['packets_sent']:>6} "
                    f"{r['packets_received']:>6} {r['prr']:>7.1f}% "
                    f"{r['throughput_bps']:>13.1f} {r['theoretical_throughput_bps']:>15.1f} "
                    f"{r['avg_rssi']:>7.1f} {r['avg_snr']:>7.1f}\n"
                )

    @staticmethod
    def _now():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AeroLink Gateway Throughput Test")
    parser.add_argument(
        "mode", choices=["burst", "pingpong"], nargs="?", default="burst",
        help="测试模式: burst (连续发包) 或 pingpong (逐包等回包)"
    )
    parser.add_argument(
        "--replot", type=str, metavar="DIR",
        help="离线重绘模式: 传入包含 throughput_raw.json 的目录，重新生成图表，不执行真实测试"
    )
    args = parser.parse_args()

    tester = ThroughputTester(mode=args.mode, replot_dir=args.replot)
    tester.run()