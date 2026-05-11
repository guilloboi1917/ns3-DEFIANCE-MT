#!/usr/bin/env python3

import argparse
import glob
import os
import sys

# Try to import NS3_HOME from defiance, fall back to relative path detection
try:
    from defiance import NS3_HOME
except ImportError:
    # Auto-detect: assume script is in contrib/defiance/examples/uav-handover/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    NS3_HOME = os.path.normpath(os.path.join(script_dir, "..", "..", "..", ".."))
    print(f"[info] defiance module not found, using NS3_HOME={NS3_HOME}")

import matplotlib.pyplot as plt
import pandas as pd


DATA_DIR = NS3_HOME + "/contrib/defiance/examples/uav-handover/"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot throughput and cwnd of UE 0 over time")
    parser.add_argument("-s", "--seed", type=int, default=None, help="Seed of the simulation")
    parser.add_argument("-r", "--run-id", type=int, default=None, help="Run ID (for parallel runs)")
    return parser.parse_args()


def find_best_file() -> tuple:
    """Auto-pick the stats file with the most data."""
    stats_files = glob.glob(DATA_DIR + "uav-handover-stats_*.csv")
    if not stats_files:
        print(f"No stats files found in {DATA_DIR}")
        print("Expected pattern: uav-handover-stats_SEED_RUNID.csv")
        sys.exit(1)

    best = max(stats_files, key=lambda f: os.path.getsize(f))
    basename = os.path.basename(best)
    parts = basename.replace("uav-handover-stats_", "").replace(".csv", "").split("_")
    return int(parts[0]), int(parts[1])


def main() -> None:
    args = parse_args()

    if args.seed is None or args.run_id is None:
        seed, run_id = find_best_file()
        print(f"Auto-selected: seed={seed}, runId={run_id} (largest stats file)")
    else:
        seed = args.seed
        run_id = args.run_id

    suffix = f"{seed}_{run_id}"
    stats_path = DATA_DIR + f"uav-handover-stats_{suffix}.csv"
    cwnd_path = DATA_DIR + f"uav-handover-cwnd_{suffix}.csv"

    if not os.path.exists(stats_path):
        print(f"Stats file not found: {stats_path}")
        print("Available files:")
        for f in sorted(glob.glob(DATA_DIR + "uav-handover-stats_*.csv")):
            print(f"  {os.path.basename(f)}")
        sys.exit(1)

    # Plot throughput
    data = pd.read_csv(stats_path, names=["Time", "Throughput"])
    data["Throughput (Mbps)"] = data["Throughput"] / 1_000_000

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    data.plot(ax=ax1, x="Time", y="Throughput (Mbps)", legend=False)
    ax1.set_ylabel("Throughput (Mbps)")
    ax1.set_title(f"UE 0 Throughput (TCP BBR, 200m, 43/23dBm, seed={seed}, run={run_id})")
    ax1.grid(True)

    # Plot cwnd
    try:
        cwnd_data = pd.read_csv(cwnd_path, names=["Time", "Cwnd"])
        # Filter out uint32_max (uninitialized read)
        cwnd_data = cwnd_data[cwnd_data["Cwnd"] < 1000000]
        cwnd_data.plot(ax=ax2, x="Time", y="Cwnd", legend=False, color="orange", marker=".", linestyle="-")
        ax2.set_ylabel("Congestion Window (bytes)")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("TCP Congestion Window (BBR)")
        ax2.grid(True)
    except FileNotFoundError:
        print("cwnd data file not found, skipping cwnd plot")
        ax2.set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = DATA_DIR + f"uav-handover-throughput_{suffix}.png"
    plt.savefig(out_path)
    print(f"Saved plot to {out_path}")


if __name__ == "__main__":
    main()
