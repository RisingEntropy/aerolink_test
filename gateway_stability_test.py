#!/usr/bin/env python3
"""
LoRa Gateway 性能测试工具
连接 MQTT 收集心跳包和 Uplink 数据，运行 10 小时后生成统计报告和图表
"""

import json
import ssl
import time
import signal
import sys
from datetime import datetime, timedelta
from collections import defaultdict
from pathlib import Path
from threading import Lock

import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

# MQTT 配置
MQTT_BROKER = "hymqtt.hydeng.cn"
MQTT_PORT = 443
MQTT_USERNAME = "admin"
MQTT_PASSWORD = "admin"
MQTT_USE_WSS = True

# 订阅主题
NODE_HUB_ID = "A3E69B6B3C731540"
TOPIC_HEARTBEAT = f"nodehub/{NODE_HUB_ID}/status/heartbeat"
TOPIC_UPLINK = f"nodehub/{NODE_HUB_ID}/data/uplink"

# 测试时长（秒）
TEST_DURATION = 10 * 60 * 60  # 10 小时

# 输出目录
OUTPUT_DIR = Path("output")
CHARTS_DIR = OUTPUT_DIR / "charts"


class GatewayTester:
    def __init__(self):
        self.start_time = None
        self.lock = Lock()

        # 心跳数据
        self.heartbeat_timestamps = []
        self.heartbeat_intervals = []
        self.cpu_usages = []
        self.memory_usages = []
        self.status_abnormal_count = 0
        self.node_offline_count = 0

        # Uplink 数据
        self.uplink_timestamps = []
        self.uplink_intervals = []
        # 按频率分开统计信号参数
        self.rssi_by_freq = defaultdict(list)
        self.snr_by_freq = defaultdict(list)
        self.noise_floor_by_freq = defaultdict(list)  # 只记录 SNR < 0 时的 rssi - snr
        self.crc_ok_count = 0
        self.crc_fail_count = 0
        self.frequency_counts = defaultdict(int)
        self.node_counts = defaultdict(int)
        self.sf_counts = defaultdict(int)
        self.hourly_counts = defaultdict(int)

        # 原始数据备份
        self.raw_heartbeats = []
        self.raw_uplinks = []

        self.running = True

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print(f"[{self._now()}] MQTT 连接成功")
            client.subscribe(TOPIC_HEARTBEAT)
            client.subscribe(TOPIC_UPLINK)
            print(f"[{self._now()}] 已订阅: {TOPIC_HEARTBEAT}")
            print(f"[{self._now()}] 已订阅: {TOPIC_UPLINK}")
        else:
            print(f"[{self._now()}] MQTT 连接失败: {reason_code}")

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        print(f"[{self._now()}] MQTT 断开连接: {reason_code}")
        if self.running:
            print(f"[{self._now()}] 将自动尝试重新连接...")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == TOPIC_HEARTBEAT:
                self._process_heartbeat(payload)
            elif msg.topic == TOPIC_UPLINK:
                self._process_uplink(payload)
        except Exception as e:
            print(f"[{self._now()}] 消息处理错误: {e}")

    def _process_heartbeat(self, data):
        with self.lock:
            self.raw_heartbeats.append(data)

            timestamp = data.get("timestamp", 0)

            # 计算心跳间隔
            if self.heartbeat_timestamps:
                interval = timestamp - self.heartbeat_timestamps[-1]
                self.heartbeat_intervals.append(interval)
            self.heartbeat_timestamps.append(timestamp)

            # CPU 和内存使用率 (CPU 需要乘 100 转为百分比)
            cpu = data.get("cpu_usage", 0) * 100
            mem = data.get("memory_usage", 0)
            self.cpu_usages.append(cpu)
            self.memory_usages.append(mem)

            # 状态检查
            if data.get("status") != "Normal":
                self.status_abnormal_count += 1
                print(f"[{self._now()}] 警告: 状态异常 - {data.get('status')}")

            # 节点连接状态
            nodes = data.get("nodes", [])
            for node in nodes:
                if not node.get("connected", True):
                    self.node_offline_count += 1
                    print(f"[{self._now()}] 警告: 节点离线 - {node.get('node_id')}")

    def _process_uplink(self, data):
        with self.lock:
            self.raw_uplinks.append(data)

            timestamp = data.get("timestamp", 0)

            # 计算包间隔
            if self.uplink_timestamps:
                interval = timestamp - self.uplink_timestamps[-1]
                self.uplink_intervals.append(interval)
            self.uplink_timestamps.append(timestamp)

            # RSSI 和 SNR（按频率分开统计）
            freq = data.get("frequency", 0)
            rssi = data.get("rssi", 0)
            snr = data.get("snr", 0)
            self.rssi_by_freq[freq].append(rssi)
            self.snr_by_freq[freq].append(snr)

            # Noise floor = rssi - snr，只在 SNR < 0 时记录
            if snr < 0:
                noise_floor = rssi - snr
                self.noise_floor_by_freq[freq].append(noise_floor)

            # CRC 状态
            if data.get("crc_ok", True):
                self.crc_ok_count += 1
            else:
                self.crc_fail_count += 1

            # 频率和节点统计
            node_id = data.get("node_id", "unknown")
            sf = data.get("spreading_factor", 0)

            self.frequency_counts[freq] += 1
            self.node_counts[node_id] += 1
            self.sf_counts[sf] += 1

            # 每小时统计
            hour = datetime.fromtimestamp(timestamp / 1000).hour
            self.hourly_counts[hour] += 1

            # 打印 uplink 日志
            total = len(self.uplink_timestamps)
            print(f"[{self._now()}] Uplink #{total}: freq={freq}MHz, RSSI={data.get('rssi')}dBm, SNR={data.get('snr')}dB, node={data.get('node_name')}")

    def _now(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def print_status(self):
        with self.lock:
            elapsed = time.time() - self.start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)

            hb_count = len(self.heartbeat_timestamps)
            ul_count = len(self.uplink_timestamps)

            print(f"\n{'='*60}")
            print(f"[{self._now()}] 运行状态 - 已运行 {hours}h {minutes}m")
            print(f"  心跳包: {hb_count} 个")
            print(f"  Uplink: {ul_count} 个")
            if ul_count > 0:
                crc_rate = self.crc_ok_count / (self.crc_ok_count + self.crc_fail_count) * 100
                print(f"  CRC 通过率: {crc_rate:.2f}%")
                # 汇总所有频率的 RSSI/SNR
                all_rssi = [v for vals in self.rssi_by_freq.values() for v in vals]
                all_snr = [v for vals in self.snr_by_freq.values() for v in vals]
                if all_rssi:
                    print(f"  平均 RSSI: {np.mean(all_rssi):.2f} dBm")
                    print(f"  平均 SNR: {np.mean(all_snr):.2f} dB")
            if self.cpu_usages:
                print(f"  平均 CPU: {np.mean(self.cpu_usages):.2f}%")
                print(f"  平均内存: {np.mean(self.memory_usages):.2f}%")
            print(f"  状态异常次数: {self.status_abnormal_count}")
            print(f"  节点离线次数: {self.node_offline_count}")
            print(f"{'='*60}\n")

    def generate_report(self):
        OUTPUT_DIR.mkdir(exist_ok=True)
        CHARTS_DIR.mkdir(exist_ok=True)

        # 保存原始数据
        with open(OUTPUT_DIR / "raw_data.json", "w") as f:
            json.dump({
                "heartbeats": self.raw_heartbeats,
                "uplinks": self.raw_uplinks
            }, f)
        print(f"[{self._now()}] 原始数据已保存到 {OUTPUT_DIR / 'raw_data.json'}")

        # 生成文本报告
        self._generate_text_report()

        # 生成图表
        self._generate_charts()

    def _generate_text_report(self):
        with open(OUTPUT_DIR / "report.txt", "w", encoding="utf-8") as f:
            f.write("=" * 60 + "\n")
            f.write("LoRa Gateway 性能测试报告\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 60 + "\n\n")

            # 心跳统计
            f.write("【心跳包统计】\n")
            f.write(f"  总数: {len(self.heartbeat_timestamps)}\n")
            if self.heartbeat_intervals:
                f.write(f"  平均间隔: {np.mean(self.heartbeat_intervals):.2f} ms\n")
                f.write(f"  最小间隔: {np.min(self.heartbeat_intervals):.2f} ms\n")
                f.write(f"  最大间隔: {np.max(self.heartbeat_intervals):.2f} ms\n")
                f.write(f"  标准差: {np.std(self.heartbeat_intervals):.2f} ms\n")
            if self.cpu_usages:
                f.write(f"  CPU 使用率 - 平均: {np.mean(self.cpu_usages):.2f}%, 最大: {np.max(self.cpu_usages):.2f}%\n")
                f.write(f"  内存使用率 - 平均: {np.mean(self.memory_usages):.2f}%, 最大: {np.max(self.memory_usages):.2f}%\n")
            f.write(f"  状态异常次数: {self.status_abnormal_count}\n")
            f.write(f"  节点离线次数: {self.node_offline_count}\n\n")

            # Uplink 统计
            f.write("【Uplink 统计】\n")
            f.write(f"  总数: {len(self.uplink_timestamps)}\n")
            if self.uplink_intervals:
                f.write(f"  平均间隔: {np.mean(self.uplink_intervals):.2f} ms\n")
                f.write(f"  最小间隔: {np.min(self.uplink_intervals):.2f} ms\n")
                f.write(f"  最大间隔: {np.max(self.uplink_intervals):.2f} ms\n")

            total_crc = self.crc_ok_count + self.crc_fail_count
            if total_crc > 0:
                f.write(f"  CRC 通过: {self.crc_ok_count} ({self.crc_ok_count/total_crc*100:.2f}%)\n")
                f.write(f"  CRC 失败: {self.crc_fail_count} ({self.crc_fail_count/total_crc*100:.2f}%)\n")

            # 按频率分开统计信号参数
            f.write("\n  【按频率分开的信号统计】\n")
            for freq in sorted(self.rssi_by_freq.keys()):
                rssi_vals = self.rssi_by_freq[freq]
                snr_vals = self.snr_by_freq[freq]
                nf_vals = self.noise_floor_by_freq.get(freq, [])

                f.write(f"\n  === {freq} MHz ===\n")
                f.write(f"    包数量: {len(rssi_vals)}\n")

                if rssi_vals:
                    f.write(f"    RSSI (dBm): 平均={np.mean(rssi_vals):.2f}, 最小={np.min(rssi_vals):.2f}, 最大={np.max(rssi_vals):.2f}, 标准差={np.std(rssi_vals):.2f}\n")

                if snr_vals:
                    f.write(f"    SNR (dB): 平均={np.mean(snr_vals):.2f}, 最小={np.min(snr_vals):.2f}, 最大={np.max(snr_vals):.2f}, 标准差={np.std(snr_vals):.2f}\n")

                if nf_vals:
                    f.write(f"    Noise Floor (dBm, SNR<0时): 平均={np.mean(nf_vals):.2f}, 最小={np.min(nf_vals):.2f}, 最大={np.max(nf_vals):.2f}, 标准差={np.std(nf_vals):.2f}, 样本数={len(nf_vals)}\n")
                else:
                    f.write(f"    Noise Floor: 无数据 (没有 SNR<0 的样本)\n")

            f.write("\n  频率通道分布:\n")
            for freq, count in sorted(self.frequency_counts.items()):
                f.write(f"    {freq} MHz: {count}\n")

            f.write("\n  节点接收分布:\n")
            for node_id, count in sorted(self.node_counts.items(), key=lambda x: -x[1]):
                f.write(f"    {node_id}: {count}\n")

            f.write("\n  Spreading Factor 分布:\n")
            for sf, count in sorted(self.sf_counts.items()):
                f.write(f"    SF{sf}: {count}\n")

            f.write("\n  每小时吞吐量:\n")
            for hour, count in sorted(self.hourly_counts.items()):
                f.write(f"    {hour:02d}:00 - {count} 包\n")

        print(f"[{self._now()}] 报告已保存到 {OUTPUT_DIR / 'report.txt'}")

    def _generate_charts(self):
        plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False

        # 1. 按频率分开的 RSSI 分布直方图
        if self.rssi_by_freq:
            freqs = sorted(self.rssi_by_freq.keys())
            n_freqs = len(freqs)
            cols = 2
            rows = (n_freqs + 1) // 2
            fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows))
            axes = axes.flatten() if n_freqs > 1 else [axes]

            for i, freq in enumerate(freqs):
                vals = self.rssi_by_freq[freq]
                if vals:
                    axes[i].hist(vals, bins=30, edgecolor='black', alpha=0.7)
                    axes[i].set_xlabel('RSSI (dBm)')
                    axes[i].set_ylabel('Count')
                    axes[i].set_title(f'{freq} MHz (n={len(vals)})')
                    axes[i].grid(True, alpha=0.3)

            # 隐藏多余的子图
            for j in range(i + 1, len(axes)):
                axes[j].set_visible(False)

            fig.suptitle('RSSI Distribution by Frequency', fontsize=14)
            plt.tight_layout()
            plt.savefig(CHARTS_DIR / "rssi_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 2. 按频率分开的 SNR 分布直方图
        if self.snr_by_freq:
            freqs = sorted(self.snr_by_freq.keys())
            n_freqs = len(freqs)
            cols = 2
            rows = (n_freqs + 1) // 2
            fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows))
            axes = axes.flatten() if n_freqs > 1 else [axes]

            for i, freq in enumerate(freqs):
                vals = self.snr_by_freq[freq]
                if vals:
                    axes[i].hist(vals, bins=30, edgecolor='black', alpha=0.7, color='green')
                    axes[i].set_xlabel('SNR (dB)')
                    axes[i].set_ylabel('Count')
                    axes[i].set_title(f'{freq} MHz (n={len(vals)})')
                    axes[i].grid(True, alpha=0.3)

            for j in range(i + 1, len(axes)):
                axes[j].set_visible(False)

            fig.suptitle('SNR Distribution by Frequency', fontsize=14)
            plt.tight_layout()
            plt.savefig(CHARTS_DIR / "snr_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 3. 按频率分开的 Noise Floor 分布直方图 (仅 SNR < 0 时)
        if self.noise_floor_by_freq:
            freqs = sorted([f for f in self.noise_floor_by_freq.keys() if self.noise_floor_by_freq[f]])
            if freqs:
                n_freqs = len(freqs)
                cols = 2
                rows = (n_freqs + 1) // 2
                fig, axes = plt.subplots(rows, cols, figsize=(14, 4 * rows))
                axes = axes.flatten() if n_freqs > 1 else [axes]

                for i, freq in enumerate(freqs):
                    vals = self.noise_floor_by_freq[freq]
                    if vals:
                        axes[i].hist(vals, bins=30, edgecolor='black', alpha=0.7, color='brown')
                        axes[i].set_xlabel('Noise Floor (dBm)')
                        axes[i].set_ylabel('Count')
                        axes[i].set_title(f'{freq} MHz (n={len(vals)})')
                        axes[i].grid(True, alpha=0.3)

                for j in range(i + 1, len(axes)):
                    axes[j].set_visible(False)

                fig.suptitle('Noise Floor Distribution by Frequency (SNR < 0 only)', fontsize=14)
                plt.tight_layout()
                plt.savefig(CHARTS_DIR / "noise_floor_distribution.png", dpi=150, bbox_inches='tight')
                plt.close()

        # 4. CPU 使用率直方图
        if self.cpu_usages:
            plt.figure(figsize=(10, 6))
            plt.hist(self.cpu_usages, bins=30, edgecolor='black', alpha=0.7, color='orange')
            plt.xlabel('CPU Usage (%)')
            plt.ylabel('Count')
            plt.title('CPU Usage Distribution')
            plt.grid(True, alpha=0.3)
            plt.savefig(CHARTS_DIR / "cpu_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 5. 内存使用率直方图
        if self.memory_usages:
            plt.figure(figsize=(10, 6))
            plt.hist(self.memory_usages, bins=30, edgecolor='black', alpha=0.7, color='purple')
            plt.xlabel('Memory Usage (%)')
            plt.ylabel('Count')
            plt.title('Memory Usage Distribution')
            plt.grid(True, alpha=0.3)
            plt.savefig(CHARTS_DIR / "memory_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 6. 心跳间隔直方图
        if self.heartbeat_intervals:
            plt.figure(figsize=(10, 6))
            plt.hist(self.heartbeat_intervals, bins=30, edgecolor='black', alpha=0.7, color='red')
            plt.xlabel('Heartbeat Interval (ms)')
            plt.ylabel('Count')
            plt.title('Heartbeat Interval Distribution')
            plt.grid(True, alpha=0.3)
            plt.savefig(CHARTS_DIR / "heartbeat_interval.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 7. 各频率通道接收量柱状图
        if self.frequency_counts:
            plt.figure(figsize=(12, 6))
            freqs = sorted(self.frequency_counts.keys())
            counts = [self.frequency_counts[f] for f in freqs]
            plt.bar([f"{f}" for f in freqs], counts, edgecolor='black', alpha=0.7)
            plt.xlabel('Frequency (MHz)')
            plt.ylabel('Packet Count')
            plt.title('Packets per Frequency Channel')
            plt.xticks(rotation=45)
            plt.grid(True, alpha=0.3, axis='y')
            plt.savefig(CHARTS_DIR / "frequency_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 8. 各节点接收量柱状图
        if self.node_counts:
            plt.figure(figsize=(12, 6))
            nodes = sorted(self.node_counts.keys())
            counts = [self.node_counts[n] for n in nodes]
            plt.bar(nodes, counts, edgecolor='black', alpha=0.7, color='teal')
            plt.xlabel('Node ID')
            plt.ylabel('Packet Count')
            plt.title('Packets per Node')
            plt.xticks(rotation=45, ha='right')
            plt.grid(True, alpha=0.3, axis='y')
            plt.savefig(CHARTS_DIR / "node_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 9. SF 分布饼图
        if self.sf_counts:
            plt.figure(figsize=(8, 8))
            sfs = sorted(self.sf_counts.keys())
            counts = [self.sf_counts[sf] for sf in sfs]
            labels = [f"SF{sf}" for sf in sfs]
            plt.pie(counts, labels=labels, autopct='%1.1f%%', startangle=90)
            plt.title('Spreading Factor Distribution')
            plt.savefig(CHARTS_DIR / "sf_distribution.png", dpi=150, bbox_inches='tight')
            plt.close()

        # 10. 每小时吞吐量柱状图
        if self.hourly_counts:
            plt.figure(figsize=(14, 6))
            hours = sorted(self.hourly_counts.keys())
            counts = [self.hourly_counts[h] for h in hours]
            plt.bar([f"{h:02d}:00" for h in hours], counts, edgecolor='black', alpha=0.7, color='steelblue')
            plt.xlabel('Hour')
            plt.ylabel('Packet Count')
            plt.title('Hourly Throughput')
            plt.xticks(rotation=45)
            plt.grid(True, alpha=0.3, axis='y')
            plt.savefig(CHARTS_DIR / "hourly_throughput.png", dpi=150, bbox_inches='tight')
            plt.close()

        print(f"[{self._now()}] 图表已保存到 {CHARTS_DIR}/")

    def run(self):
        self.start_time = time.time()

        print(f"[{self._now()}] LoRa Gateway 性能测试工具启动")
        print(f"[{self._now()}] 计划运行时间: 10 小时")
        print(f"[{self._now()}] 连接到 MQTT Broker: {MQTT_BROKER}")

        # 设置 MQTT 客户端
        client = mqtt.Client(
            client_id=f"gateway_tester_{int(time.time())}",
            transport="websockets",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.tls_set(cert_reqs=ssl.CERT_NONE)
        client.tls_insecure_set(True)

        # 启用自动重连，初始延迟 1 秒，最大延迟 120 秒
        client.reconnect_delay_set(min_delay=1, max_delay=120)

        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect
        client.on_message = self.on_message

        # 信号处理
        def signal_handler(sig, frame):
            print(f"\n[{self._now()}] 收到中断信号，正在生成报告...")
            self.running = False
            client.disconnect()

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # 连接（失败则重试）
        client.ws_set_options(path="/mqtt")
        while self.running:
            try:
                client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                client.loop_start()
                break
            except Exception as e:
                print(f"[{self._now()}] 连接失败: {e}")
                print(f"[{self._now()}] 5 秒后重试...")
                time.sleep(5)

        # 主循环
        last_status_time = time.time()
        status_interval = 60  # 每分钟打印状态

        while self.running:
            elapsed = time.time() - self.start_time

            # 检查是否到达测试时间
            if elapsed >= TEST_DURATION:
                print(f"\n[{self._now()}] 测试时间到达 10 小时，正在生成报告...")
                break

            # 定期打印状态
            if time.time() - last_status_time >= status_interval:
                self.print_status()
                last_status_time = time.time()

            time.sleep(1)

        # 停止并生成报告
        self.running = False
        client.loop_stop()
        client.disconnect()

        self.generate_report()
        print(f"[{self._now()}] 测试完成!")


if __name__ == "__main__":
    tester = GatewayTester()
    tester.run()
