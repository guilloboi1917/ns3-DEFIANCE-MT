#!/usr/bin/env python3

import sys

import matplotlib.pyplot as plt
import pandas as pd
import os
import numpy as np


def bin_data_mbps(df, bin_width=0.2):
    """
    Bin a DataFrame with columns ['time', 'size'] and compute
    throughput/goodput in Mbps over each bin.

    Sums the packet sizes (bytes) in each bin, converts to bits,
    divides by bin duration, and converts to Mbps.
    """
    t_min = df["time"].min()
    t_max = df["time"].max()
    bins = np.arange(t_min, t_max + bin_width, bin_width)
    bin_centers = (bins[:-1] + bins[1:]) / 2

    sums, _ = np.histogram(df["time"], bins=bins, weights=df["size"])
    mbps = sums * 8.0 / bin_width / 1e6  # bytes -> bits -> Mbps
    return bin_centers, mbps


def bin_count(df, bin_width=0.2):
    """
    Bin a DataFrame with columns ['time', 'size'] and count
    occurrences per bin.
    """
    t_min = df["time"].min()
    t_max = df["time"].max()
    bins = np.arange(t_min, t_max + bin_width, bin_width)
    bin_centers = (bins[:-1] + bins[1:]) / 2

    counts, _ = np.histogram(df["time"], bins=bins)
    return bin_centers, counts


def cumulative_bytes(df):
    """
    Return cumulative bytes over time from a DataFrame with columns
    ['time', 'size'].

    Sorts by time, computes a running sum of sizes, and returns
    (times, cumulative_bytes) suitable for plotting with step().
    """
    df = df.sort_values("time")
    cum = df["size"].cumsum()
    return df["time"].values, cum.values


def main(argv=None):
    outfile = "harl-tcp-stats.png"
    # If available from argv, set the output file name
    if len(argv) > 1:
        out_file = argv[1]
        if out_file.endswith(".png"):
            outfile = out_file
        else:
            outfile = out_file + ".png"
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = script_dir + '/output'
    CWND_FILE = data_dir + '/harl-tcp-cwnd.csv'
    HO_FILE = data_dir + '/harl-tcp-handovers.csv'
    RTT_FILE = data_dir + '/harl-tcp-rtt.csv'
    PACING_FILE = data_dir + '/harl-tcp-pacing-gain.csv'
    CWND_GAIN_FILE = data_dir + '/harl-tcp-cwnd-gain.csv'
    DELIVERY_RATE_FILE = data_dir + '/harl-tcp-rate.csv'
    RSRP_SINR_FILE = data_dir + '/rsrp_sinr.csv'
    UL_SINR_FILE = data_dir + '/ul_sinr.csv'
    MSC_FILE = data_dir + '/mcs.csv'
    RETRANS_FILE = data_dir + '/retransmissions.csv'
    SINK_FILE = data_dir + '/sink-packets.csv'
    SOURCE_FILE = data_dir + '/source-packets.csv'
    TX_POWER_FILE = data_dir + '/ue_tx_power.csv'

    if not os.path.exists(CWND_FILE):
        print(f"CWND data file not found: {CWND_FILE}")
        print("Run 'ns3 run defiance-tcp-harl' first to generate it.")
        exit(1)

    cwnd = pd.read_csv(CWND_FILE, header=None, names=["time", "cwnd"])
    # Multiple CWND updates can happen at the same timestamp (burst of ACKs).
    # Keep only the last value per timestamp to avoid vertical line artifacts.
    cwnd = cwnd.drop_duplicates(subset="time", keep="last").sort_values("time")

    ho = pd.read_csv(HO_FILE, header=None, names=["time", "cellId"]) \
        if os.path.exists(HO_FILE) else None

    rtt = pd.read_csv(RTT_FILE, header=None, names=["time", "rtt"]) \
        if os.path.exists(RTT_FILE) else None

    rtt = rtt.drop_duplicates(subset="time", keep="last").sort_values("time")

    cwnd_gain = pd.read_csv(CWND_GAIN_FILE, header=None, names=["time", "cwnd_gain"]) \
        if os.path.exists(CWND_GAIN_FILE) else None

    pacing_gain = pd.read_csv(PACING_FILE, header=None, names=["time", "pacing_gain"]) \
        if os.path.exists(PACING_FILE) else None

    pacing_gain = pacing_gain.drop_duplicates(
        subset="time", keep="last").sort_values("time")

    delivery_rate = delivery_rate = pd.read_csv(DELIVERY_RATE_FILE, header=None, names=["time", "rate"]) \
        if os.path.exists(DELIVERY_RATE_FILE) else None

    delivery_rate = delivery_rate.drop_duplicates(
        subset="time", keep="last").sort_values("time")

    rsrp_sinr = pd.read_csv(RSRP_SINR_FILE, header=None, names=["time", "cellId", "rnti", "rsrp", "sinr"]) \
        if os.path.exists(RSRP_SINR_FILE) else None

    if rsrp_sinr is not None and not rsrp_sinr.empty:
        rsrp_sinr["rsrp"] = 10 * np.log10(rsrp_sinr["rsrp"].clip(lower=1e-15) * 1000)
        # Use rolling average
        rsrp_sinr["sinr"] = rsrp_sinr["sinr"].rolling(window=20, min_periods=1).median()

    ul_sinr = pd.read_csv(UL_SINR_FILE, header=None, names=["time", "cellId", "rnti", "sinr"]) \
        if os.path.exists(UL_SINR_FILE) else None

    tx_power = pd.read_csv(TX_POWER_FILE, header=None,
                           names=["time", "cellId", "rnti", "txPowerDbm"]) \
        if os.path.exists(TX_POWER_FILE) else None

    mcs = pd.read_csv(MSC_FILE, header=None, names=["time", "mcs"]) \
        if os.path.exists(MSC_FILE) else None

    # Transport block sizes (in bits) for each MCS index (0-28) based on 100 PRB allocation
    # Used to calculate the theoretical max throughput for each MCS level as: throughput = TransportBlockSize / TTI (1ms) in bps
    TransportBlockSizeTable = [2792,  3624,  4584,  5736,  7224,  8760,  10296, 12216, 14112, 15840, 17568, 19848,
                               22920, 25456, 28336, 30576, 32856, 36696, 39232, 43816, 46888, 51024, 55056, 57336, 61664, 63776, 75376]

    McsToItbsUl = [0, 1, 2, 3, 4,  5,  6,  7,  8,  9,  10, 10, 11, 12, 13,
                   14, 15, 16, 17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26,]
    TTI = 0.001  # 1 ms in seconds
    if mcs is not None and not mcs.empty:
        mcs["theoretical_rate"] = mcs["mcs"].apply(
            lambda x: TransportBlockSizeTable[int(McsToItbsUl[int(x)])] / TTI / 1e6)  # Convert to Mbps

    # ── New data: retransmissions, sink (goodput), source (throughput) ──
    retrans = pd.read_csv(RETRANS_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(RETRANS_FILE) else None

    sink = pd.read_csv(SINK_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(SINK_FILE) else None

    source = pd.read_csv(SOURCE_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(SOURCE_FILE) else None

    # ── 4x2 layout ──────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
    (ax1, ax2), (ax3, ax4), (ax5, ax6), (ax7, ax8) = axes

    all_times = [cwnd["time"].max()]
    if rtt is not None:
        all_times.append(rtt["time"].max())
    if pacing_gain is not None:
        all_times.append(pacing_gain["time"].max())
    if delivery_rate is not None:
        all_times.append(delivery_rate["time"].max())
    if rsrp_sinr is not None:
        all_times.append(rsrp_sinr["time"].max())
    if ul_sinr is not None:
        all_times.append(ul_sinr["time"].max())
    if source is not None:
        all_times.append(source["time"].max())

    max_time = max(all_times)

    # ═══════════════════════════════════════════════════════════════════
    # (1,1) Congestion Window
    # ═══════════════════════════════════════════════════════════════════
    ax1.step(cwnd["time"], cwnd["cwnd"] / 1024.0,
             linewidth=1.0, color="tab:brown")
    ax1.set_ylabel("Congestion Window (KB)")
    ax1.set_title("TCP Congestion Window over Time")
    ax1.grid(True)

    # ═══════════════════════════════════════════════════════════════════
    # (1,2) RTT
    # ═══════════════════════════════════════════════════════════════════
    if rtt is not None and not rtt.empty:
        ax2.step(rtt["time"], rtt["rtt"], linewidth=1.0, color="tab:blue")
        ax2.set_ylabel("Round Trip Time (ms)")
        ax2.set_title("TCP Round Trip Time over Time")
        ax2.grid(True)
        ax2.set_ylim(0, max(rtt["rtt"].max() * 1.5, 50))
        ax2.axhline(y=20.0, xmin=0, xmax=max_time, color="red", linestyle="--",
                    alpha=0.6, linewidth=0.8, label="PGW-Server RTT")

    # ═══════════════════════════════════════════════════════════════════
    # (2,1) Pacing Gain
    # ═══════════════════════════════════════════════════════════════════
    if pacing_gain is not None and not pacing_gain.empty:
        ax3.step(pacing_gain["time"], pacing_gain["pacing_gain"],
                 linewidth=1.0, color="tab:purple", where="post")
        ax3.set_ylabel("Pacing Gain")
        ax3.set_ylim(0, 2.5)
        ax3.axhline(y=1.0, color="gray", linestyle=":", alpha=0.4, linewidth=0.5)
        ax3.grid(True)

    # ═══════════════════════════════════════════════════════════════════
    # (2,2) Delivery Rate + Theoretical Max Rate
    # ═══════════════════════════════════════════════════════════════════
    if delivery_rate is not None and not delivery_rate.empty:
        ax4.step(delivery_rate["time"], delivery_rate["rate"] /
                 1e6, linewidth=1.0, color="tab:red")
        ax4.set_ylabel("Delivery Rate (Mbps)")
        ax4.set_title("Delivery Rate over Time")
        ax4.grid(True)
    if mcs is not None and not mcs.empty:
        ax4.step(mcs["time"], mcs["theoretical_rate"], linewidth=1.0,
                 color="tab:cyan", label="Theoretical Max Rate")
        ax4.legend(fontsize=8)

    # ═══════════════════════════════════════════════════════════════════
    # (3,1) MCS Index + UE TX Power
    # ═══════════════════════════════════════════════════════════════════
    if mcs is not None and not mcs.empty:
        ax5.step(mcs["time"], mcs["mcs"], linewidth=1.0, color="tab:cyan",
                 label="MCS")
        ax5.set_ylabel("MCS Index")
        ax5.set_title("MCS Index and UE TX Power over Time")
        ax5.grid(True)
        ax5.set_ylim(0, 30)
    if tx_power is not None and not tx_power.empty:
        ax5b = ax5.twinx()
        ax5b.plot(tx_power["time"], tx_power["txPowerDbm"], linewidth=0.8,
                  color="tab:red", alpha=0.7, label="UE TX Power (dBm)")
        ax5b.set_ylabel("UE TX Power (dBm)")
        ax5b.legend(fontsize=6, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════
    # (3,2) Throughput (source) + Goodput (sink) — binned rate + cumulative
    # ═══════════════════════════════════════════════════════════════════
    ax6b = ax6.twinx()  # twin axis for cumulative

    if source is not None and not source.empty:
        t_src, thr = bin_data_mbps(source)
        ax6.step(t_src, thr, linewidth=1.0, color="tab:cyan",
                 label="Throughput (source)", alpha=0.8)
        t_src_cum, src_cum = cumulative_bytes(source)
        ax6b.step(t_src_cum, src_cum / 1e6, linewidth=1.0, color="tab:cyan",
                  linestyle="--", label="Throughput (cum. MB)", alpha=0.6)

    if sink is not None and not sink.empty:
        t_snk, gput = bin_data_mbps(sink)
        ax6.step(t_snk, gput, linewidth=1.0, color="tab:purple",
                 label="Goodput (sink)", alpha=0.8)
        t_snk_cum, snk_cum = cumulative_bytes(sink)
        ax6b.step(t_snk_cum, snk_cum / 1e6, linewidth=1.0, color="tab:purple",
                  linestyle="--", label="Goodput (cum. MB)", alpha=0.6)

    ax6.set_ylabel("Rate (Mbps)")
    ax6b.set_ylabel("Cumulative (MB)")
    ax6.set_title("Throughput vs Goodput (200 ms bins)")
    ax6.grid(True)

    # Combined legend
    lines1, labels1 = ax6.get_legend_handles_labels()
    lines2, labels2 = ax6b.get_legend_handles_labels()
    ax6.legend(lines1 + lines2, labels1 + labels2, fontsize=6, loc="upper left")

    # ═══════════════════════════════════════════════════════════════════
    # (4,1) RSRP
    # ═══════════════════════════════════════════════════════════════════
    if rsrp_sinr is not None and not rsrp_sinr.empty:
        ax7.plot(rsrp_sinr["time"], rsrp_sinr["rsrp"], linewidth=0.8, color="tab:orange")
        ax7.set_ylabel("RSRP (dBm)")
        ax7.set_title("RSRP and SINR over Time")
        ax7.grid(True)
        # twinx for SINR
        ax7b = ax7.twinx()
        ax7b.plot(rsrp_sinr["time"], rsrp_sinr["sinr"], linewidth=0.8,
                  color="tab:green", linestyle="--", alpha=0.7, label="DL SINR")
        if ul_sinr is not None and not ul_sinr.empty:
            ax7b.plot(ul_sinr["time"], ul_sinr["sinr"], linewidth=0.8,
                      color="tab:blue", linestyle="-.", alpha=0.7, label="UL SINR")
        ax7b.set_ylabel("SINR (dB)")
        ax7b.legend(fontsize=6, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════
    # (4,2) Retransmissions (binned count)
    # ═══════════════════════════════════════════════════════════════════
    if retrans is not None and not retrans.empty:
        t_ret, cnt = bin_count(retrans)
        ax8.bar(t_ret, cnt, width=0.18, color="tab:red", alpha=0.7,
                edgecolor="tab:red", linewidth=0.3)
        ax8.set_ylabel("Retransmissions (count)")
    ax8.set_title("Retransmissions (200 ms bins)")
    ax8.grid(True)

    # ═══════════════════════════════════════════════════════════════════
    # Handover markers on all axes
    # ═══════════════════════════════════════════════════════════════════
    all_axes = [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8]
    if ho is not None and not ho.empty:
        # Draw a single invisible line to create one shared legend entry
        ax1.plot([], [], color="green", linestyle="--", linewidth=0.8,
                 label="Handover")
        for ax in all_axes:
            for _, row in ho.iterrows():
                ax.axvline(x=row["time"], color="green", linestyle="--",
                           alpha=0.5, linewidth=0.7)

    ax8.set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = script_dir + "/output/" + outfile
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved to {out_path}")

    plt.show()


if __name__ == "__main__":
    main(sys.argv)
