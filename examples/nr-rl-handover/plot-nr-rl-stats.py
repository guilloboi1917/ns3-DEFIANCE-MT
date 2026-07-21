#!/usr/bin/env python3

import sys

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
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


def main(argv=None):
    outfile = "nr-rl-stats.png"
    # If available from argv, set the output file name
    if len(argv) > 1:
        out_file = argv[1]
        if out_file.endswith(".png"):
            outfile = out_file
        else:
            outfile = out_file + ".png"
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = script_dir + '/output'
    CWND_FILE = data_dir + '/nr-rl-cwnd.csv'
    HO_FILE = data_dir + '/nr-rl-handovers.csv'
    RTT_FILE = data_dir + '/nr-rl-rtt.csv'
    PACING_FILE = data_dir + '/nr-rl-pacing-gain.csv'
    CWND_GAIN_FILE = data_dir + '/nr-rl-cwnd-gain.csv'
    DELIVERY_RATE_FILE = data_dir + '/nr-rl-rate.csv'
    UL_SINR_FILE = data_dir + '/ul_sinr.csv'
    MSC_FILE = data_dir + '/mcs.csv'
    RETRANS_FILE = data_dir + '/retransmissions.csv'
    SINK_FILE = data_dir + '/sink-packets.csv'
    SOURCE_FILE = data_dir + '/source-packets.csv'
    TX_POWER_FILE = data_dir + '/ue_tx_power.csv'
    RSRP_RSRQ_200ms = data_dir + '/ue_meas_report.csv'
    RSRP_SINR_FILE = data_dir + '/rsrp_sinr.csv'
    REWARD_FILE = data_dir + '/rl_reward.csv'

    if not os.path.exists(CWND_FILE):
        print(f"CWND data file not found: {CWND_FILE}")
        print("Run 'ns3 run defiance-nr-rl-handover' first to generate it.")
        exit(1)

    cwnd = pd.read_csv(CWND_FILE, header=None, names=["time", "cwnd"])
    # Multiple CWND updates can happen at the same timestamp (burst of ACKs).
    # Keep only the last value per timestamp to avoid vertical line artifacts.
    # Filter out any values larger than uint32 max (shouldn't happen, but just in case of logging bugs).
    # 4 GB in bytes, well above any reasonable CWND
    cwnd = cwnd[cwnd["cwnd"] <= 4 * 1024 * 1024]
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

    rsrp_rsrq_full = pd.read_csv(RSRP_RSRQ_200ms, header=None,
                                names=["time", "cellId", "rnti", "rsrp", "rsrq", "isServingCell"]) \
        if os.path.exists(RSRP_RSRQ_200ms) else None
    rsrp_rsrq = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy() \
        if rsrp_rsrq_full is not None and not rsrp_rsrq_full.empty else None

    ul_sinr = pd.read_csv(UL_SINR_FILE, header=None, names=["time", "cellId", "rnti", "sinr"]) \
        if os.path.exists(UL_SINR_FILE) else None

    tx_power = pd.read_csv(TX_POWER_FILE, header=None,
                           names=["time", "cellId", "rnti", "txPowerDbm"]) \
        if os.path.exists(TX_POWER_FILE) else None

    # Parse MCS file — supports both LTE (3 cols: time, mcs, tbs) and NR (2 cols: time, tbSize)
    if os.path.exists(MSC_FILE):
        mcs_raw = pd.read_csv(MSC_FILE, header=None)
        n_cols = mcs_raw.shape[1]
        if n_cols == 3:
            # LTE format: time, mcs, tbs
            mcs = mcs_raw.copy()
            mcs.columns = ["time", "mcs", "tbs"]
        else:
            # NR format: time, tbSize (MCS not logged, compute rate directly from TBS)
            mcs = mcs_raw.copy()
            mcs.columns = ["time", "tbs"]
            mcs["mcs"] = 0  # placeholder
    else:
        mcs = None

    TTI = 0.001  # 1 ms in seconds (1 slot at numerology 0 ~ 15kHz SCS, numerology 1 ~ 30 kHz SCS (0.5ms))
    if mcs is not None and not mcs.empty:
        # Instantaneous PHY rate per slot (1ms): TBS (bytes) * 8 / 1ms / 1e6 = Mbps
        # This is noisy per-slot; use rolling average for the plot.
        pass

    # ── New data: rl_reward (EWMA reward components) ──
    rsrp_sinr = pd.read_csv(RSRP_SINR_FILE, header=None, names=["time", "cellId", "rnti", "rsrp", "sinrDb"]) \
        if os.path.exists(RSRP_SINR_FILE) else None

    rl_reward = pd.read_csv(REWARD_FILE, header=None, names=["time", "goodput_mbps", "dynRef_mbps", "dynMin_mbps", "normGoodput", "rtt_ms", "rttPenalty", "tcpPenalty", "rlfTerm", "tbsBonus", "handoverPenalty", "rsrpDeltaBonus", "reward"]) \
        if os.path.exists(REWARD_FILE) else None

    # ── New data: retransmissions, sink (goodput), source (throughput) ──
    retrans = pd.read_csv(RETRANS_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(RETRANS_FILE) else None

    sink = pd.read_csv(SINK_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(SINK_FILE) else None

    source = pd.read_csv(SOURCE_FILE, header=None, names=["time", "size"]) \
        if os.path.exists(SOURCE_FILE) else None

    # ── 6x2 layout ──────────────────────────────────────────────────────
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

    all_times = [cwnd["time"].max()]
    if rtt is not None:
        all_times.append(rtt["time"].max())
    if pacing_gain is not None:
        all_times.append(pacing_gain["time"].max())
    if delivery_rate is not None:
        all_times.append(delivery_rate["time"].max())
    if rsrp_rsrq is not None:
        all_times.append(rsrp_rsrq["time"].max())
    if ul_sinr is not None:
        all_times.append(ul_sinr["time"].max())
    if source is not None:
        all_times.append(source["time"].max())
    if rl_reward is not None:
        all_times.append(rl_reward["time"].max())

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
        # Break line at gaps > 200ms (RTT updates per ACK, ~6ms interval)
        t_rtt, y_rtt = _insert_nan_at_gaps(rtt["time"].values, rtt["rtt"].values, 0.2)
        ax2.step(t_rtt, y_rtt, linewidth=1.0, color="tab:blue", where="post")
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
        ax3.axhline(y=1.0, color="gray", linestyle=":",
                    alpha=0.4, linewidth=0.5)
        ax3.grid(True)

    # ═══════════════════════════════════════════════════════════════════
    # (2,2) Delivery Rate + Theoretical Max Rate
    # ═══════════════════════════════════════════════════════════════════
    if delivery_rate is not None and not delivery_rate.empty:
        # Break line at gaps > 200ms (delivery rate updates per ACK, ~6ms interval)
        t_dr, y_dr = _insert_nan_at_gaps(delivery_rate["time"].values,
                                          delivery_rate["rate"].values / 1e6, 0.2)
        ax4.step(t_dr, y_dr, linewidth=1.0, color="tab:red", where="post")
        ax4.set_ylabel("Delivery Rate (Mbps)")
        ax4.set_title("Delivery Rate over Time")
        ax4.grid(True)
    if mcs is not None and not mcs.empty:
        # Per-slot TBS is noisy (fires every 1ms). Smooth with a 100ms rolling window.
        mcs_smooth = mcs.copy()
        mcs_smooth["rate_mbps"] = mcs_smooth["tbs"] * 8.0 / TTI / 1e6
        mcs_smooth["rate_avg"] = mcs_smooth["rate_mbps"].rolling(window=100, min_periods=1).mean()
        ax4.step(mcs_smooth["time"], mcs_smooth["rate_avg"], linewidth=0.8, color="tab:orange",
                 label="PHY throughput (100ms avg, Mbps)", alpha=0.9)
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
    # (4,1) RSRP / RSRQ / SINR (200ms serving cell)
    # ═══════════════════════════════════════════════════════════════════
    if rsrp_rsrq is not None and not rsrp_rsrq.empty:
        ax7.plot(rsrp_rsrq["time"], rsrp_rsrq["rsrp"], linewidth=1.2,
                 color="tab:orange", label="RSRP 200ms serving")
        ax7.set_ylabel("RSRP (dBm)")
        ax7.set_title("Serving Cell RSRP, RSRQ and SINR")
        ax7.grid(True)
        ax7.legend(fontsize=6, loc="upper left")
        # twinx for RSRQ + SINR
        ax7b = ax7.twinx()
        ax7b.plot(rsrp_rsrq["time"], rsrp_rsrq["rsrq"], linewidth=0.8,
                  color="tab:purple", linestyle=":", alpha=0.7,
                  label="RSRQ 200ms serving")
        if ul_sinr is not None and not ul_sinr.empty:
            # UL SINR fires every ~0.6ms (dense). Smooth with rolling window.
            ul_sinr_smooth = ul_sinr["sinr"].rolling(window=100, center=True, min_periods=1).median()
            ax7b.plot(ul_sinr["time"], ul_sinr_smooth, linewidth=0.8,
                      color="tab:blue", linestyle="-.", alpha=0.7, label="UL SINR serving")
        if rsrp_sinr is not None and not rsrp_sinr.empty:
            # Apply median moving average filter (window=10 ~ 10ms at 1ms SRS)
            dl_sinr_smooth = rsrp_sinr["sinrDb"].rolling(window=10, center=True, min_periods=1).median()
            ax7b.plot(rsrp_sinr["time"], dl_sinr_smooth, linewidth=0.8,
                      color="tab:green", linestyle=":", alpha=0.7, label="DL SINR (serving)")
        ax7b.set_ylabel("RSRQ / SINR (dB)")
        ax7b.legend(fontsize=6, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════
    # (4,2) Reward Components (EWMA-based adaptive reward)
    # ═══════════════════════════════════════════════════════════════════
    if rl_reward is not None and not rl_reward.empty:
        t = rl_reward["time"]

        # Goodput (from sink) vs dynamicRef/dynamicMin
        ax8.plot(t, rl_reward["goodput_mbps"], linewidth=1.0, color="tab:purple",
                 label="Goodput (sink Mbps)")
        ax8.plot(t, rl_reward["dynRef_mbps"], linewidth=0.8, color="tab:green",
                 linestyle="--", label="dynRef (EWMA x 1.1)")
        ax8.plot(t, rl_reward["dynMin_mbps"], linewidth=0.8, color="tab:olive",
                 linestyle=":", label="dynMin (EWMA x 0.3)")

        # Compute and plot EWMA goodput (for reference)
        ewma = rl_reward["goodput_mbps"].ewm(alpha=0.2).mean()
        ax8.plot(t, ewma, linewidth=0.8, color="tab:orange",
                 linestyle="--", alpha=0.7, label="Goodput EWMA (alpha=0.2)")

        ax8.set_ylabel("Throughput (Mbps)")
        ax8.set_title("Reward Components: Goodput, EWMA Ref, TBS Bonus")
        ax8.grid(True)

        # Twin axis for normalized reward values
        ax8b = ax8.twinx()
        ax8b.plot(t, rl_reward["normGoodput"], linewidth=0.8, color="tab:blue",
                  linestyle="--", alpha=0.6, label="normGoodput")
        ax8b.plot(t, rl_reward["tbsBonus"], linewidth=0.8, color="tab:cyan",
                  linestyle="--", alpha=0.6, label="TBS bonus")
        ax8b.plot(t, rl_reward["reward"], linewidth=1.2, color="tab:red",
                  label="Reward (total)")
        ax8b.axhline(y=0, color="gray", linestyle=":", alpha=0.3, linewidth=0.5)
        ax8b.set_ylabel("Normalized reward components")

        # Combined legend
        lines1, labels1 = ax8.get_legend_handles_labels()
        lines2, labels2 = ax8b.get_legend_handles_labels()
        ax8.legend(lines1 + lines2, labels1 + labels2,
                   fontsize=5, loc="upper left")
    else:
        # Fallback: retransmissions if no reward data
        if retrans is not None and not retrans.empty:
            t_ret, cnt = bin_count(retrans)
            ax8.bar(t_ret, cnt, width=0.18, color="tab:red", alpha=0.7,
                    edgecolor="tab:red", linewidth=0.3)
            ax8.set_ylabel("Retransmissions (count)")
        ax8.set_title("Retransmissions (200 ms bins)")
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
        serving_data = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy()
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
        serving_data = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy()
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
    out_path = script_dir + "/output/" + outfile
    plt.savefig(out_path, dpi=300)
    print(f"Plot saved to {out_path}")

    plt.show()


if __name__ == "__main__":
    main(sys.argv)
