#!/usr/bin/env python3
"""
Multi-SF Priority Test

Tests LR2021 in Multi-SF mode: when 4 signals with different SFs arrive simultaneously,
which one will be demodulated preferentially?
1. Primary SF priority?
2. Maximum power SF priority?

Test Method:
- 4 TX modules send signals with different SFs at the same time.
- RX is configured with the Primary SF (the smallest SF).
- Record which SF is demodulated by the RX each time.
- Verify demodulation priority by adjusting the power of each TX.
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
# Set to a standard sans-serif font for English compatibility
plt.rcParams['font.family'] = 'sans-serif'

from node_comm import (
    FrameDecoder,
    LoRaModule,
    NodeWorkingMode,
    PacketType,
    calc_time_on_air,
    select_port,
)


# ============================================================
#  Test Configuration
# ============================================================

# The 4 SFs to be tested (the first one is the Primary SF)
TEST_SF_LIST = [5, 6, 7, 8]

# Number of packets sent per test round
PACKETS_PER_TEST = 100

# Fixed parameters
TEST_FREQUENCY = 1439.0  # MHz
TEST_BANDWIDTH = 500.0   # kHz
CODING_RATE = 5          # 4/5
SYNC_WORD = 0x12
PREAMBLE_LENGTH = 8
PAYLOAD_SIZE = 32        # Test packet payload size (bytes)

# TX Power configurations (for different test scenarios)
# Scenario 1: All TX have equal power
# Scenario 2: Primary SF low power, one Secondary SF high power
# Scenario 3: Primary SF high power, Secondary SFs low power
TX_POWER_DEFAULT = 10  # dBm
TX_POWER_HIGH = 10     # dBm
TX_POWER_LOW = -9      # dBm

# Output directories
OUTPUT_DIR = Path("Multi-SF Priority Test")
CHARTS_DIR = OUTPUT_DIR / "charts"


# ============================================================
#  Tester Class
# ============================================================

class MultiSFPriorityTester:
    def __init__(self):
        self.tx_modules: list[LoRaModule] = []    # 4 TX modules
        self.rx_module: LoRaModule | None = None  # 1 RX module (LR2021)
        self.results: dict[str, dict] = {}        # Scenario name -> Test results
        self.raw_rx_data: list[dict] = []
        self.start_time: float = 0
        self._interrupted = False

    # ---- Initialization ----

    def select_ports(self):
        """Allow user to select TX and RX serial ports"""
        primary_sf = min(TEST_SF_LIST)
        secondary_sfs = [sf for sf in TEST_SF_LIST if sf != primary_sf]

        print("=" * 60)
        print("Multi-SF Priority Test")
        print("=" * 60)
        print("\nPurpose: Verify demodulation priority when multiple SF signals arrive simultaneously.")
        print("  - Is Primary SF prioritized? Or is the SF with the highest power prioritized?")
        print()
        print(f"SF Configuration:")
        print(f"  Primary SF:   SF{primary_sf}")
        print(f"  Secondary SF: {[f'SF{sf}' for sf in secondary_sfs]}")
        print()

        # Select RX Module (Must be LR2021)
        rx_port = select_port("Please select RX module port (Must be LR2021)")
        print(f"\nConnecting to RX module ({rx_port})...")
        self.rx_module = LoRaModule(rx_port, name="RX")

        if self.rx_module.model != LoRaModule.MODEL_LR2021:
            print(f"Warning: RX module is not LR2021 (Detected {self.rx_module.model})")
            confirm = input("Continue anyway? (y/N): ").strip().lower()
            if confirm != 'y':
                sys.exit(1)

        # Select 4 TX Modules
        used_ports = {rx_port}
        for i, sf in enumerate(TEST_SF_LIST):
            sf_type = "Primary SF" if sf == primary_sf else "Secondary SF"
            tx_port = select_port(f"Select TX{i+1} module port (SF{sf}, {sf_type})")
            if tx_port in used_ports:
                print(f"Error: Port {tx_port} is already in use.")
                sys.exit(1)
            used_ports.add(tx_port)

            print(f"Connecting to TX{i+1} module ({tx_port})...")
            tx_module = LoRaModule(tx_port, name=f"TX{i+1}-SF{sf}")
            self.tx_modules.append(tx_module)

        print("\nModule Information:")
        print(f"  RX: {self.rx_module}")
        for i, (tx, sf) in enumerate(zip(self.tx_modules, TEST_SF_LIST)):
            sf_type = "Primary SF" if sf == primary_sf else "Secondary SF"
            print(f"  TX{i+1} (SF{sf}, {sf_type}): {tx}")

    def print_test_plan(self):
        """Print test plan details"""
        primary_sf = min(TEST_SF_LIST)

        print(f"\n{'=' * 60}")
        print("Test Plan:")
        print(f"  Frequency:      {TEST_FREQUENCY} MHz")
        print(f"  Bandwidth:      {TEST_BANDWIDTH} kHz")
        print(f"  Primary SF:     SF{primary_sf}")
        print(f"  Secondary SFs:  {[sf for sf in TEST_SF_LIST if sf != primary_sf]}")
        print(f"  Packets/Scenario: {PACKETS_PER_TEST}")
        print()
        print("Test Scenarios:")
        print(f"  Scenario 1: Equal power for all TX ({TX_POWER_DEFAULT} dBm)")
        print(f"  Scenario 2: Primary SF Low ({TX_POWER_LOW} dBm), Secondary SFs High ({TX_POWER_HIGH} dBm)")
        print(f"  Scenario 3: Primary SF High ({TX_POWER_HIGH} dBm), Secondary SFs Low ({TX_POWER_LOW} dBm)")
        print(f"{'=' * 60}")

        input("\nPress Enter to start test (Ctrl+C to interrupt)...")

    # ---- Main Test ----

    def run(self):
        """Execute full test workflow"""
        self.start_time = time.time()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        primary_sf = min(TEST_SF_LIST)

        print(f"\n[{self._now()}] Starting test...")

        # Configure RX as Primary SF (remains constant throughout test)
        print(f"\n[{self._now()}] Configuring RX to Primary SF{primary_sf}...")
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
        time.sleep(0.2)

        # Define scenarios
        test_scenarios = [
            {
                "name": "equal_power",
                "description": f"All TX Equal Power ({TX_POWER_DEFAULT} dBm)",
                "powers": {sf: TX_POWER_DEFAULT for sf in TEST_SF_LIST},
            },
            {
                "name": "secondary_high_power",
                "description": f"Secondary SF High ({TX_POWER_HIGH} dBm), Primary SF Low ({TX_POWER_LOW} dBm)",
                "powers": {sf: TX_POWER_LOW if sf == primary_sf else TX_POWER_HIGH
                          for sf in TEST_SF_LIST},
            },
            {
                "name": "primary_high_power",
                "description": f"Primary SF High ({TX_POWER_HIGH} dBm), Secondary SF Low ({TX_POWER_LOW} dBm)",
                "powers": {sf: TX_POWER_HIGH if sf == primary_sf else TX_POWER_LOW
                          for sf in TEST_SF_LIST},
            },
        ]

        # Execute each scenario
        for scenario in test_scenarios:
            if self._interrupted:
                break

            print(f"\n{'='*60}")
            print(f"[{self._now()}] Scenario: {scenario['description']}")
            print(f"{'='*60}")

            result = self._run_scenario(scenario, primary_sf)
            self.results[scenario["name"]] = result

            # Print result
            self._print_scenario_result(scenario["name"], result, primary_sf)

        # Completion
        elapsed = time.time() - self.start_time
        print(f"\n[{self._now()}] Test complete. Elapsed time: {elapsed / 60:.1f} minutes")

        # Summary
        self._print_summary()

        # Save
        self.save_results()

    def _run_scenario(self, scenario: dict, primary_sf: int) -> dict:
        """Run a single test scenario"""
        powers = scenario["powers"]

        # Configure TX power and SF
        print(f"[{self._now()}] Configuring TX power...")
        for tx, sf in zip(self.tx_modules, TEST_SF_LIST):
            power = powers[sf]
            sf_type = "Primary SF" if sf == primary_sf else "Secondary SF"
            print(f"  {tx.name}: SF{sf} ({sf_type}), Power={power} dBm")
            tx.set_lora_params(
                mode=NodeWorkingMode.TX,
                frequency=TEST_FREQUENCY,
                bandwidth=TEST_BANDWIDTH,
                spreading_factor=sf,
                coding_rate=CODING_RATE,
                sync_word=SYNC_WORD,
                tx_power=power,
                preamble_length=PREAMBLE_LENGTH,
            )
        time.sleep(0.1)

        # Clear RX buffer
        self.rx_module.ser.reset_input_buffer()
        self.rx_module.decoder = FrameDecoder()

        # Calculate max Time on Air
        max_toa = max(
            calc_time_on_air(sf, TEST_BANDWIDTH, PAYLOAD_SIZE, PREAMBLE_LENGTH, CODING_RATE - 4)
            for sf in TEST_SF_LIST
        )
        rx_timeout = max(0.5, max_toa + 0.5)

        # Statistics
        tx_count = 0
        decoded_sf_counts = {sf: 0 for sf in TEST_SF_LIST}
        decoded_sf_counts["none"] = 0  # No packet demodulated
        rx_packets = []

        print(f"[{self._now()}] Starting simultaneous transmission test ({PACKETS_PER_TEST} iterations)...")

        for seq in range(PACKETS_PER_TEST):
            if self._interrupted:
                break

            # Prepare payloads for all TX
            payloads = []
            for sf in TEST_SF_LIST:
                payload = bytearray(PAYLOAD_SIZE)
                payload[0] = seq & 0xFF
                payload[1] = (seq >> 8) & 0xFF
                payload[2] = sf & 0xFF  # SF marker
                for j in range(3, PAYLOAD_SIZE):
                    payload[j] = (seq + j + sf) & 0xFF
                payloads.append(bytes(payload))

            # Trigger all TX simultaneously (as fast as possible)
            for tx, sf, payload in zip(self.tx_modules, TEST_SF_LIST, payloads):
                tx.send_tx_data(payload, target_sf=sf)
            tx_count += 1

            # Wait for RX demodulation
            decoded_this_round = False
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
                        if detected_sf in decoded_sf_counts:
                            decoded_sf_counts[detected_sf] += 1
                            decoded_this_round = True

                            # Record details
                            pkt_record = {
                                "seq": seq,
                                "scenario": scenario["name"],
                                "detected_sf": detected_sf,
                                "is_primary": detected_sf == primary_sf,
                                "rssi": parsed.get("rssi", 0),
                                "snr": parsed.get("snr", 0),
                                "crc_status": parsed.get("crc_status", 0),
                                "tx_power": powers.get(detected_sf, 0),
                            }
                            rx_packets.append(pkt_record)
                            self.raw_rx_data.append(pkt_record)
                    except Exception:
                        pass
                    break  # Record only the first demodulated packet

            if not decoded_this_round:
                decoded_sf_counts["none"] += 1

            # Progress display
            if (seq + 1) % 20 == 0:
                print(f"  Progress: {seq + 1}/{PACKETS_PER_TEST}")

        # Compute stats
        result = {
            "scenario": scenario["name"],
            "description": scenario["description"],
            "powers": powers,
            "tx_count": tx_count,
            "decoded_counts": decoded_sf_counts,
            "rx_packets": rx_packets,
            "primary_sf": primary_sf,
        }

        # Demodulation rates
        total_decoded = sum(decoded_sf_counts[sf] for sf in TEST_SF_LIST)
        result["decode_rates"] = {}
        for sf in TEST_SF_LIST:
            rate = (decoded_sf_counts[sf] / total_decoded * 100) if total_decoded > 0 else 0
            result["decode_rates"][sf] = rate

        result["total_decoded"] = total_decoded
        result["no_decode_count"] = decoded_sf_counts["none"]

        return result

    def _print_scenario_result(self, name: str, result: dict, primary_sf: int):
        """Print result of a single scenario"""
        print(f"\nScenario Result: {result['description']}")
        print("-" * 50)
        print(f"Sent:       {result['tx_count']}")
        print(f"Decoded:    {result['total_decoded']}")
        print(f"Failed:     {result['no_decode_count']}")
        print()
        print(f"{'SF':>4} {'Type':>8} {'Power':>8} {'Count':>6} {'Rate':>8}")
        print("-" * 40)

        for sf in TEST_SF_LIST:
            sf_type = "Primary" if sf == primary_sf else "Second"
            power = result["powers"][sf]
            count = result["decoded_counts"][sf]
            rate = result["decode_rates"][sf]
            print(f"{sf:>4} {sf_type:>8} {power:>7}dB {count:>6} {rate:>7.1f}%")

        # Judge priority
        if result["total_decoded"] > 0:
            max_sf = max(TEST_SF_LIST, key=lambda sf: result["decoded_counts"][sf])
            if max_sf == primary_sf:
                print(f"\n=> Primary SF (SF{primary_sf}) was prioritized.")
            else:
                print(f"\n=> Secondary SF (SF{max_sf}) was prioritized.")

    def _print_summary(self):
        """Print overall summary"""
        primary_sf = min(TEST_SF_LIST)

        print("\n" + "=" * 70)
        print("TEST SUMMARY")
        print("=" * 70)

        print("\nDemodulation Distribution per Scenario:")
        print(f"{'Scenario':>20} ", end="")
        for sf in TEST_SF_LIST:
            sf_type = "P" if sf == primary_sf else "S"
            print(f"{'SF'+str(sf)+'('+sf_type+')':>12}", end="")
        print(f"{'Priority':>10}")
        print("-" * 70)

        for name, result in self.results.items():
            print(f"{result['description'][:20]:>20} ", end="")
            max_sf = None
            max_rate = 0
            for sf in TEST_SF_LIST:
                rate = result["decode_rates"][sf]
                print(f"{rate:>11.1f}%", end="")
                if rate > max_rate:
                    max_rate = rate
                    max_sf = sf
            priority = f"SF{max_sf}" if max_sf else "N/A"
            print(f"{priority:>10}")

        print("-" * 70)

        # Analysis
        print("\nConclusions:")
        equal_power_result = self.results.get("equal_power", {})
        secondary_high_result = self.results.get("secondary_high_power", {})
        primary_high_result = self.results.get("primary_high_power", {})

        if equal_power_result:
            max_sf_equal = max(TEST_SF_LIST,
                              key=lambda sf: equal_power_result.get("decoded_counts", {}).get(sf, 0))
            primary_rate = equal_power_result.get("decode_rates", {}).get(primary_sf, 0)
            if primary_rate > 60:
                print(f"  1. Equal Power: Primary SF (SF{primary_sf}) takes priority ({primary_rate:.1f}%)")
            else:
                print(f"  1. Equal Power: SF{max_sf_equal} was demodulated most frequently.")

        if secondary_high_result and primary_high_result:
            max_sf_sec_high = max(TEST_SF_LIST,
                                 key=lambda sf: secondary_high_result.get("decoded_counts", {}).get(sf, 0))
            max_sf_pri_high = max(TEST_SF_LIST,
                                 key=lambda sf: primary_high_result.get("decoded_counts", {}).get(sf, 0))

            if max_sf_sec_high != primary_sf:
                print(f"  2. Secondary SF High Power: Secondary SF (SF{max_sf_sec_high}) prioritized -> Power Priority!")
            else:
                print(f"  2. Secondary SF High Power: Primary SF still prioritized -> Primary SF Priority!")

            if max_sf_pri_high == primary_sf:
                print(f"  3. Primary SF High Power: Primary SF (SF{primary_sf}) prioritized.")
            else:
                print(f"  3. Primary SF High Power: SF{max_sf_pri_high} prioritized.")

        print("=" * 70)

    # ---- Result Saving ----

    def save_results(self):
        """Save all test results"""
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        CHARTS_DIR.mkdir(parents=True, exist_ok=True)

        self._save_csv()
        self._save_report()
        self._save_raw_data()
        self._generate_charts()

        print(f"[{self._now()}] Results saved to {OUTPUT_DIR}/")

    def _save_csv(self):
        csv_path = OUTPUT_DIR / "results.csv"
        fieldnames = ["scenario", "sf", "is_primary", "tx_power", "count", "rate"]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            primary_sf = min(TEST_SF_LIST)
            for name, result in self.results.items():
                for sf in TEST_SF_LIST:
                    row = {
                        "scenario": name,
                        "sf": sf,
                        "is_primary": "Yes" if sf == primary_sf else "No",
                        "tx_power": result["powers"][sf],
                        "count": result["decoded_counts"][sf],
                        "rate": f"{result['decode_rates'][sf]:.2f}",
                    }
                    writer.writerow(row)

        print(f"[{self._now()}] CSV Saved: {csv_path}")

    def _save_report(self):
        report_path = OUTPUT_DIR / "report.txt"
        elapsed = time.time() - self.start_time
        primary_sf = min(TEST_SF_LIST)

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("Multi-SF Priority Test Report\n")
            f.write(f"Test Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Duration:  {elapsed / 60:.1f} minutes\n")
            f.write("=" * 70 + "\n\n")

            f.write("[Objective]\n")
            f.write("  Verify demodulation priority when multiple SF signals arrive simultaneously:\n")
            f.write("  - Primary SF priority vs. Maximum Power priority.\n\n")

            f.write("[Device Info]\n")
            f.write(f"  RX: {self.rx_module}\n")
            for i, (tx, sf) in enumerate(zip(self.tx_modules, TEST_SF_LIST)):
                sf_type = "Primary SF" if sf == primary_sf else "Secondary SF"
                f.write(f"  TX{i+1} (SF{sf}, {sf_type}): {tx}\n")
            f.write("\n")

            f.write("[Test Parameters]\n")
            f.write(f"  Frequency: {TEST_FREQUENCY} MHz\n")
            f.write(f"  Bandwidth: {TEST_BANDWIDTH} kHz\n")
            f.write(f"  Primary SF: SF{primary_sf}\n")
            f.write(f"  Secondary SFs: {[sf for sf in TEST_SF_LIST if sf != primary_sf]}\n")
            f.write(f"  Iterations/Scenario: {PACKETS_PER_TEST}\n\n")

            f.write("[Results]\n")
            for name, result in self.results.items():
                f.write(f"\n{result['description']}:\n")
                f.write(f"  {'SF':>4} {'Type':>8} {'Power':>8} {'Count':>6} {'Rate':>8}\n")
                f.write("  " + "-" * 40 + "\n")
                for sf in TEST_SF_LIST:
                    sf_type = "Primary" if sf == primary_sf else "Second"
                    power = result["powers"][sf]
                    count = result["decoded_counts"][sf]
                    rate = result["decode_rates"][sf]
                    f.write(f"  {sf:>4} {sf_type:>8} {power:>7}dB {count:>6} {rate:>7.1f}%\n")

            f.write("\n" + "=" * 70 + "\n")
            f.write("[Conclusion]\n")
            # Analysis would be appended here...

        print(f"[{self._now()}] Report Saved: {report_path}")

    def _save_raw_data(self):
        raw_path = OUTPUT_DIR / "raw_data.json"
        serializable_results = {}
        for name, result in self.results.items():
            serializable_results[name] = {
                "scenario": result["scenario"],
                "description": result["description"],
                "powers": result["powers"],
                "tx_count": result["tx_count"],
                "decoded_counts": result["decoded_counts"],
                "decode_rates": result["decode_rates"],
                "total_decoded": result["total_decoded"],
                "no_decode_count": result["no_decode_count"],
                "primary_sf": result["primary_sf"],
            }

        output = {
            "test_config": {
                "frequency": TEST_FREQUENCY,
                "bandwidth": TEST_BANDWIDTH,
                "sf_list": TEST_SF_LIST,
                "primary_sf": min(TEST_SF_LIST),
                "packets_per_test": PACKETS_PER_TEST,
                "coding_rate": CODING_RATE,
                "sync_word": SYNC_WORD,
                "preamble_length": PREAMBLE_LENGTH,
                "payload_size": PAYLOAD_SIZE,
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
                        "sf": sf,
                        "port": tx.port,
                        "model": tx.model,
                        "node_id": tx.node_id,
                        "firmware": tx.firmware,
                    }
                    for tx, sf in zip(self.tx_modules, TEST_SF_LIST)
                ],
            },
            "results": serializable_results,
            "raw_rx_packets": self.raw_rx_data,
        }

        with open(raw_path, "w", encoding="utf-8") as f:
            json.dump(output, f, indent=2, ensure_ascii=False)

        print(f"[{self._now()}] Raw data saved: {raw_path}")

    # ---- Chart Generation ----

    def _generate_charts(self):
        if not self.results:
            return

        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "figure.dpi": 300,
            "savefig.dpi": 300,
        })

        self._plot_decode_distribution()
        self._plot_scenario_comparison()

        print(f"[{self._now()}] Charts saved to {CHARTS_DIR}/")

    def _plot_decode_distribution(self):
        primary_sf = min(TEST_SF_LIST)
        n_scenarios = len(self.results)

        fig, axes = plt.subplots(1, n_scenarios, figsize=(5 * n_scenarios, 5), squeeze=False)
        axes = axes.flatten()

        for idx, (name, result) in enumerate(self.results.items()):
            ax = axes[idx]
            sfs = TEST_SF_LIST
            rates = [result["decode_rates"][sf] for sf in sfs]
            colors = ["#3498db" if sf == primary_sf else "#2ecc71" for sf in sfs]

            bars = ax.bar([f"SF{sf}" for sf in sfs], rates, color=colors, edgecolor="black", linewidth=1)

            for bar, rate in zip(bars, rates):
                height = bar.get_height()
                if height > 0:
                    ax.annotate(f'{rate:.1f}%',
                                xy=(bar.get_x() + bar.get_width() / 2, height),
                                xytext=(0, 3), textcoords="offset points",
                                ha='center', va='bottom', fontweight='bold')

            ax.set_xlabel("Spreading Factor")
            ax.set_ylabel("Decode Rate (%)")
            ax.set_title(f"{result['description'][:30]}", fontweight='bold')
            ax.set_ylim(0, 110)
            ax.grid(True, alpha=0.3, axis='y', linestyle='--')

            # Power info box
            power_text = "\n".join([f"SF{sf}: {result['powers'][sf]}dB" for sf in sfs])
            ax.text(0.02, 0.98, power_text, transform=ax.transAxes,
                    fontsize=8, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#3498db', edgecolor='black', label=f'Primary SF (SF{primary_sf})'),
            Patch(facecolor='#2ecc71', edgecolor='black', label='Secondary SF'),
        ]
        fig.legend(handles=legend_elements, loc='upper center', ncol=2, bbox_to_anchor=(0.5, 0.02))

        fig.suptitle("Multi-SF Decode Distribution by Scenario", fontsize=14, fontweight='bold')
        fig.tight_layout(rect=[0, 0.05, 1, 0.95])
        fig.savefig(CHARTS_DIR / "decode_distribution.png", bbox_inches="tight")
        plt.close(fig)

    def _plot_scenario_comparison(self):
        primary_sf = min(TEST_SF_LIST)
        fig, ax = plt.subplots(figsize=(12, 6))

        scenarios = list(self.results.keys())
        x = np.arange(len(scenarios))
        width = 0.2
        colors = ['#3498db', '#2ecc71', '#e74c3c', '#9b59b6']

        for i, sf in enumerate(TEST_SF_LIST):
            rates = [self.results[s]["decode_rates"][sf] for s in scenarios]
            sf_label = f"SF{sf} (Primary)" if sf == primary_sf else f"SF{sf}"
            bars = ax.bar(x + i * width, rates, width, label=sf_label, color=colors[i], edgecolor='black')

            for bar in bars:
                height = bar.get_height()
                if height > 5:
                    ax.annotate(f'{height:.0f}%',
                                xy=(bar.get_x() + bar.get_width() / 2, height),
                                xytext=(0, 2), textcoords="offset points",
                                ha='center', va='bottom', fontsize=7)

        ax.set_xlabel("Test Scenario")
        ax.set_ylabel("Decode Rate (%)")
        ax.set_title("Multi-SF Priority Test - Scenario Comparison\n"
                     "Which SF is decoded when 4 signals arrive simultaneously?",
                     fontweight='bold')
        ax.set_xticks(x + width * 1.5)
        ax.set_xticklabels([self.results[s]["description"][:25] for s in scenarios], fontsize=8)
        ax.set_ylim(0, 110)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y', linestyle='--')

        fig.tight_layout()
        fig.savefig(CHARTS_DIR / "scenario_comparison.png", bbox_inches="tight")
        plt.close(fig)

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _signal_handler(self, sig, frame):
        print(f"\n[{self._now()}] Interrupt signal received...")
        self._interrupted = True

    def cleanup(self):
        for tx in self.tx_modules:
            tx.close()
        if self.rx_module:
            self.rx_module.close()


def replot_from_json(json_path: str | Path):
    """Regenerate charts from a JSON result file"""
    json_path = Path(json_path)
    if not json_path.exists():
        print(f"Error: File not found: {json_path}")
        sys.exit(1)

    print(f"Reading data: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    results = data.get("results", {})
    if not results:
        print("Error: No result data found in JSON.")
        sys.exit(1)

    tester = MultiSFPriorityTester()
    tester.results = results

    # Restore config
    test_config = data.get("test_config", {})
    global TEST_SF_LIST, TEST_FREQUENCY, TEST_BANDWIDTH
    TEST_SF_LIST = test_config.get("sf_list", TEST_SF_LIST)
    TEST_FREQUENCY = test_config.get("frequency", TEST_FREQUENCY)
    TEST_BANDWIDTH = test_config.get("bandwidth", TEST_BANDWIDTH)

    output_dir = json_path.parent
    charts_dir = output_dir / "charts"
    charts_dir.mkdir(parents=True, exist_ok=True)

    global OUTPUT_DIR, CHARTS_DIR
    OUTPUT_DIR = output_dir
    CHARTS_DIR = charts_dir

    print(f"SF List: {TEST_SF_LIST}")
    print(f"Scenarios: {len(results)}")
    print(f"Saving charts to: {charts_dir}")

    tester._generate_charts()
    print("Re-plotting complete!")


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Multi-SF Priority Test - Verify priority when multiple SF signals arrive simultaneously.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Run full test
  python multi_sf_sequential_test.py

  # Re-plot from JSON
  python multi_sf_sequential_test.py --replot "Multi-SF Priority Test/raw_data.json"
        """
    )

    parser.add_argument(
        "-r", "--replot",
        metavar="JSON_FILE",
        help="Regenerate charts from a JSON file (doesn't run test)"
    )

    args = parser.parse_args()

    if args.replot:
        replot_from_json(args.replot)
        return

    tester = MultiSFPriorityTester()
    try:
        tester.select_ports()
        tester.print_test_plan()
        tester.run()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        if tester.results:
            tester.save_results()
    except Exception as e:
        print(f"\nFatal Error: {e}")
        import traceback
        traceback.print_exc()
        if tester.results:
            tester.save_results()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
