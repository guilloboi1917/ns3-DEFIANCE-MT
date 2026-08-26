#!/usr/bin/env python3

import sys

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import os
import numpy as np
import argparse


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


def _uav_rnti_mask(df, ue_meas):
    """Mask scheduling rows that belong to the UAV, tracking RNTI changes.

    The UAV's C-RNTI is reallocated by the target gNB at every handover, so a
    fixed `rnti == 1` filter silently drops the UAV's scheduling after the
    first handover — and can even pick up an interferer's entries, because
    RNTI allocation restarts at 1 per cell. ue_meas_report.csv is the UAV's
    own PHY measurement report and carries its current RNTI at every report,
    so it provides the exact time -> RNTI mapping.

    Args:
        df: scheduling DataFrame with "time" and "rnti" columns.
        ue_meas: the raw ue_meas_report.csv DataFrame (time, cellId, rnti,
            rsrp, rsrq, isServingCell), or None.

    Returns:
        Boolean Series/ndarray over df's rows: True where the row is the UAV.
    """
    if ue_meas is None or ue_meas.empty:
        print("[warn] ue_meas_report.csv missing/unreadable — falling back to "
              "rnti==1 (wrong after the first handover); enable logging=true "
              "for correct per-UAV plots")
        return df["rnti"] == 1
    tl = ue_meas[["time", "rnti"]].drop_duplicates(subset="time", keep="last")
    tl = tl.sort_values("time")
    times = tl["time"].to_numpy()
    rntis = tl["rnti"].to_numpy()
    idx = np.searchsorted(times, df["time"].to_numpy(), side="right") - 1
    idx = np.clip(idx, 0, len(tl) - 1)
    return df["rnti"].to_numpy() == rntis[idx]


def _insert_nan_at_gaps(t, y, max_gap):
    """
    Insert NaN between consecutive points when the time gap exceeds max_gap.
    This breaks the line in step() plots, preventing misleading connecting
    lines across long gaps (e.g. TCP death periods).

    Returns (t_out, y_out) with NaN separators inserted.
    """
    if len(t) < 2:
        return t, y
    t_out = []
    y_out = []
    for i in range(len(t)):
        if i > 0 and (t[i] - t[i - 1]) > max_gap:
            t_out.append(np.nan)
            y_out.append(np.nan)
        t_out.append(t[i])
        y_out.append(y[i])
    return np.array(t_out), np.array(y_out)


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


# Argument parsing
parser = argparse.ArgumentParser(
    description="Plot NR-RL handover statistics from CSV logs.")
parser.add_argument("-o", "--output", type=str, default="nr-rl-stats.png",
                    help="Output PNG file name for the plot.")
parser.add_argument("-i", "--input_dir", type=str, default=None,
                    help="Directory containing the CSV log files. Defaults to 'output' in the script directory. Relative to script directory if not absolute.")
args = parser.parse_args()


def main(argv=None):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = script_dir + '/output'

    if args.output:
        out_file = args.output
        # If available from argv, set the output file name
        if out_file.endswith(".png"):
            outfile = out_file
        else:
            outfile = out_file + ".png"
    if args.input_dir:
        data_dir = args.input_dir
        if data_dir.startswith("./") or data_dir.startswith("../"):
            data_dir = os.path.abspath(os.path.join(script_dir, data_dir))
        else:
            data_dir = os.path.abspath(data_dir)
        if not os.path.exists(data_dir):
            print(
                f"Input directory does not exist, creating directory: {data_dir}")
            os.makedirs(data_dir, exist_ok=True)
            print(f"Using input directory: {data_dir}")

    HARQ_FILE = data_dir + '/nr-rl-harq.csv'
    SLOT_STATS_FILE = data_dir + '/nr-rl-slot-stats.csv'
    CWND_FILE = data_dir + '/nr-rl-cwnd.csv'
    HO_FILE = data_dir + '/nr-rl-handovers.csv'
    RTT_FILE = data_dir + '/nr-rl-rtt.csv'
    PACING_FILE = data_dir + '/nr-rl-pacing-gain.csv'
    CWND_GAIN_FILE = data_dir + '/nr-rl-cwnd-gain.csv'
    DELIVERY_RATE_FILE = data_dir + '/nr-rl-rate.csv'
    DL_SINR_FILE = data_dir + '/dl_sinr.csv'
    UL_SINR_SRS_FILE = data_dir + '/ul_sinr_srs.csv'
    RETRANS_FILE = data_dir + '/retransmissions.csv'
    SINK_FILE = data_dir + '/sink-packets.csv'
    SOURCE_FILE = data_dir + '/source-packets.csv'
    TX_POWER_FILE = data_dir + '/ue_tx_power.csv'
    RSRP_RSRQ_200ms = data_dir + '/ue_meas_report.csv'
    RSRP_SINR_FILE = data_dir + '/rsrp_sinr.csv'
    REWARD_FILE = data_dir + '/rl_reward.csv'

    if not os.path.exists(CWND_FILE):
        print(f"CWND data file not found: {CWND_FILE}")
        print("Some TCP-specific plots will be empty (expected for UDP runs).")

    harq = pd.read_csv(HARQ_FILE, skiprows=1, header=None,
                       names=["time", "cellId", "rnti", "bwpId", "harqId", "k1Delay"]) \
        if os.path.exists(HARQ_FILE) else None

    slot_stats = pd.read_csv(SLOT_STATS_FILE, skiprows=1, header=None,
                             names=["time", "cellId", "scheduledUe", "usedReg",
                                    "usedSym", "availableRb", "availableSym", "utilPct"]) \
        if os.path.exists(SLOT_STATS_FILE) else None

    cwnd = pd.read_csv(CWND_FILE, skiprows=1, header=None, names=["time", "cwnd"])
    # Multiple CWND updates can happen at the same timestamp (burst of ACKs).
    # Keep only the last value per timestamp to avoid vertical line artifacts.
    # Filter out any values larger than uint32 max (shouldn't happen, but just in case of logging bugs).
    # 4 GB in bytes, well above any reasonable CWND
    cwnd = cwnd[cwnd["cwnd"] <= 4 * 1024 * 1024]
    cwnd = cwnd.drop_duplicates(subset="time", keep="last").sort_values("time")

    ho = pd.read_csv(HO_FILE, skiprows=1, header=None, names=["time", "cellId"]) \
        if os.path.exists(HO_FILE) else None

    rtt = pd.read_csv(RTT_FILE, skiprows=1, header=None, names=["time", "rtt"]) \
        if os.path.exists(RTT_FILE) else None

    rtt = rtt.drop_duplicates(subset="time", keep="last").sort_values("time")

    cwnd_gain = pd.read_csv(CWND_GAIN_FILE, skiprows=1, header=None, names=["time", "cwnd_gain"]) \
        if os.path.exists(CWND_GAIN_FILE) else None

    pacing_gain = pd.read_csv(PACING_FILE, skiprows=1, header=None, names=["time", "pacing_gain"]) \
        if os.path.exists(PACING_FILE) else None

    pacing_gain = pacing_gain.drop_duplicates(
        subset="time", keep="last").sort_values("time")

    delivery_rate = delivery_rate = pd.read_csv(DELIVERY_RATE_FILE, skiprows=1, header=None, names=["time", "rate"]) \
        if os.path.exists(DELIVERY_RATE_FILE) else None

    delivery_rate = delivery_rate.drop_duplicates(
        subset="time", keep="last").sort_values("time")

    rsrp_rsrq_full = pd.read_csv(RSRP_RSRQ_200ms, skiprows=1, header=None,
                                 names=["time", "cellId", "rnti", "rsrp", "rsrq", "isServingCell"]) \
        if os.path.exists(RSRP_RSRQ_200ms) else None
    rsrp_rsrq = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy() \
        if rsrp_rsrq_full is not None and not rsrp_rsrq_full.empty else None

    ul_sinr_srs = pd.read_csv(UL_SINR_SRS_FILE, skiprows=1, header=None, names=["time", "cellId", "sinr"]) \
        if os.path.exists(UL_SINR_SRS_FILE) else None

    # ── UL scheduling data (MCS, TBS per slot) ──
    UL_SCHED_FILE = data_dir + '/nr-rl-ul-sched.csv'
    ul_sched = pd.read_csv(UL_SCHED_FILE, skiprows=1, header=None,
                           names=["time", "cellId", "rnti", "mcs", "tbSize", "symStart", "numSym"]) \
        if os.path.exists(UL_SCHED_FILE) else None

    dl_sinr = pd.read_csv(DL_SINR_FILE, skiprows=1, header=None, names=["time", "cellId", "rnti", "sinr"]) \
        if os.path.exists(DL_SINR_FILE) else None

    tx_power = pd.read_csv(TX_POWER_FILE, skiprows=1, header=None,
                           names=["time", "cellId", "rnti", "txPowerDbm"]) \
        if os.path.exists(TX_POWER_FILE) else None

    # ── DL scheduling data (replaces mcs.csv) ──
    DL_SCHED_FILE = data_dir + '/nr-rl-dl-sched.csv'
    dl_sched = pd.read_csv(DL_SCHED_FILE, skiprows=1, header=None,
                           names=["time", "cellId", "rnti", "mcs", "tbSize", "symStart", "numSym"]) \
        if os.path.exists(DL_SCHED_FILE) else None

    # ── rl_reward (multiplicative R_G x R_H reward components) ──
    rsrp_sinr = pd.read_csv(RSRP_SINR_FILE, skiprows=1, header=None, names=["time", "cellId", "rnti", "rsrp", "sinrDb"]) \
        if os.path.exists(RSRP_SINR_FILE) else None

    rl_reward = pd.read_csv(REWARD_FILE, skiprows=1, header=None,
                            names=["time", "goodput_mbps", "dynRef_mbps", "dynMin_mbps",
                                   "normGoodputRaw", "normGoodput", "rg", "iHo", "rH",
                                   "pingPong", "reward"]) \
        if os.path.exists(REWARD_FILE) else None

    # ── New data: retransmissions, sink (goodput), source (throughput) ──
    retrans = pd.read_csv(RETRANS_FILE, skiprows=1, header=None, names=["time", "size"]) \
        if os.path.exists(RETRANS_FILE) else None

    sink = pd.read_csv(SINK_FILE, skiprows=1, header=None, names=["time", "size"]) \
        if os.path.exists(SINK_FILE) else None

    source = pd.read_csv(SOURCE_FILE, skiprows=1, header=None, names=["time", "size"]) \
        if os.path.exists(SOURCE_FILE) else None

    # ── 6x2 layout (was 7x2, removed per-cell RB utilization) ──────────
    fig = plt.figure(figsize=(12, 17))
    gs = gridspec.GridSpec(6, 2, figure=fig)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 0])
    ax6 = fig.add_subplot(gs[2, 1])
    ax7 = fig.add_subplot(gs[3, 0])
    ax8 = fig.add_subplot(gs[3, 1])
    ax9 = fig.add_subplot(gs[4, :])
    ax10 = fig.add_subplot(gs[5, :])

    for ax in [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10]:
        ax.minorticks_on()

    # Share x-axis across all subplots
    for ax in [ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10]:
        ax.sharex(ax1)

    all_times = []
    if cwnd is not None and not cwnd.empty:
        all_times.append(cwnd["time"].max())
    if rtt is not None and not rtt.empty:
        all_times.append(rtt["time"].max())
    if pacing_gain is not None and not pacing_gain.empty:
        all_times.append(pacing_gain["time"].max())
    if delivery_rate is not None and not delivery_rate.empty:
        all_times.append(delivery_rate["time"].max())
    if rsrp_rsrq is not None and not rsrp_rsrq.empty:
        all_times.append(rsrp_rsrq["time"].max())
    if ul_sinr_srs is not None and not ul_sinr_srs.empty:
        all_times.append(ul_sinr_srs["time"].max())
    if ul_sched is not None and not ul_sched.empty:
        all_times.append(ul_sched["time"].max())
    if dl_sinr is not None and not dl_sinr.empty:
        all_times.append(dl_sinr["time"].max())
    if source is not None and not source.empty:
        all_times.append(source["time"].max())
    if rl_reward is not None and not rl_reward.empty:
        all_times.append(rl_reward["time"].max())

    # Fallback if no time-series data available
    if not all_times:
        all_times = [0.0]

    max_time = max(all_times)

    # Start all axes at 1s (skip the pre-TCP setup phase)
    ax1.set_xlim(0, max_time * 1.05)

    # ═══════════════════════════════════════════════════════════════════
    # (1,1) Congestion Window
    # ═══════════════════════════════════════════════════════════════════
    if cwnd is not None and not cwnd.empty:
        # Break line at gaps > 200ms (CWND updates per ACK, ~6ms interval)
        t_cwnd, y_cwnd = _insert_nan_at_gaps(
            cwnd["time"].values, cwnd["cwnd"].values, 0.2)
        ax1.step(t_cwnd, y_cwnd / 1024.0, linewidth=1.0,
                 color="tab:brown", where="post")
        ax1.set_ylabel("Congestion Window (KB)")
        ax1.set_title("TCP Congestion Window over Time")
        ax1.grid(True)

    # ═══════════════════════════════════════════════════════════════════
    # (1,2) RTT
    # ═══════════════════════════════════════════════════════════════════
    if rtt is not None and not rtt.empty:
        # Break line at gaps > 200ms (RTT updates per ACK, ~6ms interval)
        t_rtt, y_rtt = _insert_nan_at_gaps(
            rtt["time"].values, rtt["rtt"].values, 0.2)
        ax2.step(t_rtt, y_rtt, linewidth=1.0, color="tab:blue", where="post")
        ax2.set_ylabel("Round Trip Time (ms)")
        ax2.set_title("TCP Round Trip Time over Time")
        ax2.grid(True)
        ax2.set_ylim(0, max(rtt["rtt"].max() * 1.5, 50))
        ax2.axhline(y=20.0, xmin=0, xmax=max_time, color="red", linestyle="--",
                    alpha=0.6, linewidth=0.8, label="PGW-Server RTT")

    # ═══════════════════════════════════════════════════════════════════
    # (2,1) DL/UL SINR — serving cell only for clarity
    # ═══════════════════════════════════════════════════════════════════
    ax3.set_ylabel("SINR (dB)")
    ax3.set_title("DL and UL SINR over Time (serving cell)")
    ax3.grid(True)
    if dl_sinr is not None and not dl_sinr.empty:
        # DlDataSinr trace only fires for the serving cell (UE PHY), so
        # dl_sinr.csv already contains serving-cell SINR. Plot as-is.
        dl_plot = dl_sinr
        if not dl_plot.empty:
            dl_sinr_smooth = dl_plot["sinr"].rolling(
                window=100, center=True, min_periods=1).median()
            ax3.plot(dl_plot["time"], dl_sinr_smooth, linewidth=0.8,
                     color="tab:green", linestyle="-", alpha=0.7, label="DL SINR (data)")
    if ul_sinr_srs is not None and not ul_sinr_srs.empty:
        # The UlSrsSinrLogger already filters by g_currentCellId in C++, so
        # all rows are already serving-cell SINR.  No merge needed.
        ul_srs_smooth = ul_sinr_srs["sinr"].rolling(
            window=400, center=True, min_periods=1).mean()
        ax3.plot(ul_sinr_srs["time"], ul_srs_smooth, linewidth=0.8,
                 color="tab:cyan", linestyle="--", alpha=0.7, label="UL SINR (SRS)")
    ax3.legend(fontsize=6, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════
    # (2,2) Delivery Rate + UL MCS + PHY Throughput
    # ═══════════════════════════════════════════════════════════════════
    ax4b = ax4.twinx()
    if delivery_rate is not None and not delivery_rate.empty:
        t_dr, y_dr = _insert_nan_at_gaps(delivery_rate["time"].values,
                                         delivery_rate["rate"].values / 1e6, 0.2)
        ax4.step(t_dr, y_dr, linewidth=1.0, color="tab:red", where="post")
        ax4.set_ylabel("Delivery Rate (Mbps)")
        ax4.set_title("Delivery Rate, UL/DL MCS, and PHY Throughput")
        ax4.grid(True)
    if dl_sched is not None and not dl_sched.empty:
        # DL PHY throughput from scheduling, tracked to the UAV via its
        # time-varying RNTI (C-RNTI changes at every handover).
        uav_mask = _uav_rnti_mask(dl_sched, rsrp_rsrq_full)
        dl_tbs_main = dl_sched[uav_mask].copy()
        if not dl_tbs_main.empty:
            dl_tbs_main["rate_mbps"] = dl_tbs_main["tbSize"] * 8.0 / 1e-3 / 1e6
            dl_tbs_main["rate_avg"] = dl_tbs_main["rate_mbps"].rolling(
                window=100, min_periods=1).mean()
            ax4.step(dl_tbs_main["time"], dl_tbs_main["rate_avg"], linewidth=0.8, color="tab:orange",
                     label="DL PHY rate (100slots avg Mbps)", alpha=0.9)
            # DL MCS on the same MCS axis as the UL MCS (green dashed)
            dl_mcs_smooth = dl_tbs_main["mcs"].rolling(
                window=100, min_periods=1).mean()
            ax4b.step(dl_tbs_main["time"], dl_mcs_smooth, linewidth=1.0,
                      color="tab:green", where="post",
                      label="DL MCS (100slots avg)")
    if ul_sched is not None and not ul_sched.empty:
        # Filter to the UAV (RNTI changes on handover) and smooth UL MCS
        uav_mask_ul = _uav_rnti_mask(ul_sched, rsrp_rsrq_full)
        ul_mcs_main = ul_sched[uav_mask_ul].copy()
        if not ul_mcs_main.empty:
            ul_mcs_smooth = ul_mcs_main["mcs"].rolling(
                window=100, min_periods=1).mean()
            ax4b.step(ul_mcs_main["time"], ul_mcs_smooth, linewidth=1.0,
                      color="tab:blue", where="post", label="UL MCS (100slots avg)")
            ax4b.set_ylabel("MCS index (UL blue / DL green)")
            ax4b.set_ylim(-1, 29)
        # Also show UL PHY throughput (TBS-based)
        ul_tbs_main = ul_sched[uav_mask_ul].copy()
        if not ul_tbs_main.empty:
            ul_tbs_main["rate_mbps"] = ul_tbs_main["tbSize"] * 8.0 / 1e-3 / 1e6
            ul_tbs_main["rate_avg"] = ul_tbs_main["rate_mbps"].rolling(
                window=100, min_periods=1).mean()
            ax4.step(ul_tbs_main["time"], ul_tbs_main["rate_avg"], linewidth=0.8,
                     color="tab:blue", linestyle=":", alpha=0.7, label="UL PHY rate (100ms avg Mbps)")
    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4b.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2,
               fontsize=6, loc="upper left")

    # ═══════════════════════════════════════════════════════════════════
    # (3,1) HARQ Transmissions (binned) — replaces MCS + UE TX Power
    # ═══════════════════════════════════════════════════════════════════
    if harq is not None and not harq.empty:
        # Bin HARQ entries per 200ms window, colored by cell ID
        cell_ids_harq = sorted(harq["cellId"].unique())
        colors_h = plt.cm.Set1(np.linspace(0, 1, len(cell_ids_harq)))
        t_min = harq["time"].min()
        t_max = harq["time"].max()
        bin_width = 0.2
        bins = np.arange(t_min, t_max + bin_width, bin_width)
        bin_centers = (bins[:-1] + bins[1:]) / 2

        bottom = np.zeros(len(bin_centers))
        for idx, cid in enumerate(cell_ids_harq):
            cid_data = harq[harq["cellId"] == cid]
            counts, _ = np.histogram(cid_data["time"], bins=bins)
            ax5.bar(bin_centers, counts, width=bin_width * 0.9,
                    bottom=bottom, color=colors_h[idx], alpha=0.7,
                    label=f"Cell {int(cid)}")
            bottom += counts
        ax5.set_ylabel("HARQ feedback count")
        ax5.set_title("HARQ Transmissions (200ms bins, colored by cell)")
        ax5.grid(True)
        ax5.set_yscale("symlog", linthresh=10)
        ax5.legend(fontsize=6, loc="upper right")

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

        # Compute EWMA of goodput directly from sink data (200ms bins, alpha=0.2)
        # to match what the reward app computes in C++
        gput_series = pd.Series(gput, index=t_snk)
        ewma_gput = gput_series.ewm(alpha=0.2).mean()
        ax6.plot(ewma_gput.index, ewma_gput.values, linewidth=1.0,
                 color="tab:orange", linestyle="--", alpha=0.8,
                 label="Goodput EWMA (alpha=0.1)")

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
    ax6.legend(lines1 + lines2, labels1 + labels2,
               fontsize=6, loc="upper left")

    # ═══════════════════════════════════════════════════════════════════
    # (4,1) RSRP / RSRQ (200ms serving cell)
    # ═══════════════════════════════════════════════════════════════════
    if rsrp_rsrq is not None and not rsrp_rsrq.empty:
        ax7.plot(rsrp_rsrq["time"], rsrp_rsrq["rsrp"], linewidth=1.2,
                 color="tab:orange", label="RSRP 200ms serving")
        ax7.set_ylabel("RSRP (dBm)")
        ax7.set_title("Serving Cell RSRP and RSRQ")
        ax7.grid(True)
        ax7.legend(fontsize=6, loc="upper left")
        # twinx for RSRQ
        ax7b = ax7.twinx()
        ax7b.plot(rsrp_rsrq["time"], rsrp_rsrq["rsrq"], linewidth=0.8,
                  color="tab:purple", linestyle=":", alpha=0.7,
                  label="RSRQ 200ms serving")
        ax7b.set_ylabel("RSRQ (dB)")
        ax7b.legend(fontsize=6, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════
    # (4,2) Reward Components (multiplicative R_G x R_H)
    # ═══════════════════════════════════════════════════════════════════
    if rl_reward is not None and not rl_reward.empty:
        t = rl_reward["time"]

        # Goodput vs the reward reference. The campaign uses the FIXED
        # rlRewardRefMbps (the EWMA-adaptive ref is an optional legacy
        # reward-app flag, off by default), so dynRef/dynMin plot the fixed
        # reference and floor.
        ax8.plot(t, rl_reward["goodput_mbps"], linewidth=1.0, color="tab:purple",
                 label="Goodput (sink Mbps)")
        ax8.plot(t, rl_reward["dynRef_mbps"], linewidth=0.8, color="tab:green",
                 linestyle="--", label="reward ref (rlRewardRefMbps)")
        ax8.plot(t, rl_reward["dynMin_mbps"], linewidth=0.8, color="tab:olive",
                 linestyle=":", label="reward min")

        # Plot-side EWMA of the goodput (visual smoothing only)
        ewma = rl_reward["goodput_mbps"].ewm(alpha=0.2).mean()
        ax8.plot(t, ewma, linewidth=0.8, color="tab:orange",
                 linestyle="--", alpha=0.7, label="Goodput EWMA (plot, alpha=0.2)")

        ax8.set_ylabel("Throughput (Mbps)")
        ax8.set_title("Reward Components (multiplicative R_G x R_H)")
        ax8.grid(True)

        # Twin axis for the reward terms
        ax8b = ax8.twinx()
        ax8b.plot(t, rl_reward["normGoodput"], linewidth=0.8, color="tab:blue",
                  linestyle="--", alpha=0.6, label="normGoodput")
        ax8b.plot(t, rl_reward["rg"], linewidth=0.8, color="tab:cyan",
                  linestyle="--", alpha=0.6, label="R_G (goodput term)")
        ax8b.plot(t, rl_reward["rH"], linewidth=0.8, color="tab:orange",
                  linestyle=":", alpha=0.6, label="R_H (handover term)")
        ax8b.plot(t, rl_reward["reward"], linewidth=1.2, color="tab:red",
                  label="Reward (total)")
        ax8b.axhline(y=0, color="gray", linestyle=":",
                     alpha=0.3, linewidth=0.5)
        ax8b.set_ylabel("Normalized reward components")

        # Combined legend
        lines1, labels1 = ax8.get_legend_handles_labels()
        lines2, labels2 = ax8b.get_legend_handles_labels()
        ax8.legend(lines1 + lines2, labels1 + labels2,
                   fontsize=5, loc="upper left")
    else:
        # Serving cell utilization with 200ms rolling average for readability
        ss_sorted = slot_stats.sort_values("time")
        ss_sorted["util_smooth"] = ss_sorted["utilPct"].rolling(
            window=200, min_periods=1, center=True).mean()
        ax8.step(ss_sorted["time"], ss_sorted["util_smooth"], linewidth=0.8,
                 color="tab:red", alpha=0.8, where="post", label="RB util (%)")
        if slot_stats["scheduledUe"].max() > 1:
            busy = slot_stats[slot_stats["scheduledUe"] > 1]
            ax8.plot(busy["time"], [100]*len(busy), "v", color="darkred",
                     markersize=3, alpha=0.5, label=f">1 UE ({len(busy)} slots)")
            ax8.set_ylabel("Utilization (%)")
            ax8.set_title("Serving Cell RB Utilization (per slot)")
            ax8.set_ylim(-5, 105)
            ax8.grid(True)
            ax8.legend(fontsize=6, loc="upper right")
        else:
            ax8.set_title("Serving Cell RB Utilization")
            ax8.grid(True)
    # if rsrp_sinr is not None and not rsrp_sinr.empty:
    #     ax8.plot(rsrp_sinr["time"], rsrp_sinr["rsrp"], linewidth=0.8, color="tab:orange")
    #     ax8.set_ylabel("RSRP (dBm)")
    #     ax8.grid(True)
    #     # twinx for SINR
    #     ax8.plot(rsrp_sinr["time"], rsrp_sinr["rsrp_ewma"], linewidth=0.7,
    #               color="tab:red", linestyle="--", alpha=0.7, label="RSRP EWMA")
    # ═══════════════════════════════════════════════════════════════════
    # Handover markers on all axes
    # ═══════════════════════════════════════════════════════════════════
    # ═══════════════════════════════════════════════════════════════════
    # (5,1) RSRP all cells + serving cell highlight
    # ═══════════════════════════════════════════════════════════════════
    if rsrp_rsrq_full is not None and not rsrp_rsrq_full.empty:
        cell_ids = sorted(rsrp_rsrq_full["cellId"].unique())
        colors = plt.cm.gist_ncar(np.linspace(0, 0.9, len(cell_ids)))
        for idx, cell_id in enumerate(cell_ids):
            cell_data = rsrp_rsrq_full[rsrp_rsrq_full["cellId"] == cell_id]
            ax9.plot(cell_data["time"], cell_data["rsrp"], linewidth=0.6,
                     color=colors[idx], alpha=0.5,
                     label=f"Cell {int(cell_id)}")
        # Overlay serving cell RSRP with thicker lines, split at gaps to avoid
        # connecting non-contiguous serving periods (e.g. across handovers).
        serving_data = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy(
        )
        if not serving_data.empty:
            for idx, cell_id in enumerate(cell_ids):
                seg = serving_data[serving_data["cellId"] == cell_id]
                if not seg.empty:
                    seg = seg.sort_values("time")
                    # Gap > 300ms = handover away and back; split into segments
                    gap_threshold = 0.3
                    seg = seg.reset_index(drop=True)
                    seg["_gap"] = seg["time"].diff() > gap_threshold
                    seg["_segment"] = seg["_gap"].cumsum()
                    for _, segment in seg.groupby("_segment"):
                        # Draw at least a marker for single-point segments (rapid handovers)
                        if len(segment) >= 2:
                            ax9.plot(segment["time"], segment["rsrp"], linewidth=1.2,
                                     color=colors[idx], alpha=1.0)
                        elif len(segment) == 1:
                            ax9.plot(segment["time"], segment["rsrp"], marker="o",
                                     markersize=4, color=colors[idx], alpha=1.0, linestyle="")
        ax9.set_ylabel("RSRP (dBm)")
        ax9.set_title("RSRP per Cell (200ms ReportUeMeasurements)")
        ax9.grid(True)
        ax9.set_xlabel("")

    # Place the legend to the right of ax9 to avoid hiding data
    ax9.legend(fontsize=6, loc="upper left", bbox_to_anchor=(1.02, 1.0))

    # ═══════════════════════════════════════════════════════════════════
    # (6,1) RSRQ all cells + serving cell highlight
    # ═══════════════════════════════════════════════════════════════════
    if rsrp_rsrq_full is not None and not rsrp_rsrq_full.empty:
        cell_ids = sorted(rsrp_rsrq_full["cellId"].unique())
        colors = plt.cm.gist_ncar(np.linspace(0, 0.9, len(cell_ids)))
        for idx, cell_id in enumerate(cell_ids):
            cell_data = rsrp_rsrq_full[rsrp_rsrq_full["cellId"] == cell_id]
            ax10.plot(cell_data["time"], cell_data["rsrq"], linewidth=0.6,
                      color=colors[idx], alpha=0.5,
                      label=f"Cell {int(cell_id)}")
        # Overlay serving cell RSRQ with thicker lines, split at gaps
        serving_data = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy(
        )
        if not serving_data.empty:
            for idx, cell_id in enumerate(cell_ids):
                seg = serving_data[serving_data["cellId"] == cell_id]
                if not seg.empty:
                    seg = seg.sort_values("time")
                    gap_threshold = 0.3
                    seg = seg.reset_index(drop=True)
                    seg["_gap"] = seg["time"].diff() > gap_threshold
                    seg["_segment"] = seg["_gap"].cumsum()
                    for _, segment in seg.groupby("_segment"):
                        if len(segment) >= 2:
                            ax10.plot(segment["time"], segment["rsrq"], linewidth=1.2,
                                      color=colors[idx], alpha=1.0)
                        elif len(segment) == 1:
                            ax10.plot(segment["time"], segment["rsrq"], marker="o",
                                      markersize=4, color=colors[idx], alpha=1.0, linestyle="")
        ax10.set_ylabel("RSRQ (dB)")
        ax10.set_title("RSRQ per Cell (200ms ReportUeMeasurements)")
        ax10.grid(True)
        ax10.set_xlabel("Time (s)")
    ax10.legend(fontsize=6, loc="upper left", bbox_to_anchor=(1.02, 1.0))

    all_axes = [ax1, ax2, ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10]
    if ho is not None and not ho.empty:
        # Draw a single invisible line to create one shared legend entry
        ax1.plot([], [], color="green", linestyle="--", linewidth=0.8,
                 label="Handover")
        for ax in all_axes:
            for _, row in ho.iterrows():
                ax.axvline(x=row["time"], color="green", linestyle="--",
                           alpha=0.5, linewidth=0.7)

    plt.tight_layout()
    out_path = data_dir + "/" + outfile
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved to {out_path}")

    plt.show()


if __name__ == "__main__":
    main(sys.argv)
