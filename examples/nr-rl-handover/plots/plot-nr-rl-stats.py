#!/usr/bin/env python3
"""Plot the per-seed NR-RL handover statistics from a seed dir's CSV logs.

Panels are drawn from the nr-rl-*.csv logs (throughput, SINR, cwnd/RTT,
scheduling, RLC, reward, ...); --split-figures writes one PNG per panel.

Usage: python3 plots/plot-nr-rl-stats.py [seed_dir] [-o out.png] [--ewma]
"""

import sys

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
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


def _ewma_time(t, y, alpha, bin_s=0.1):
    """EWMA on a uniform time grid so smoothing is time-consistent across
    panels with very different sampling rates (1 kHz slot stats vs 200 ms
    goodput bins). y is averaged per bin_s seconds, then a sample EWMA
    (alpha per bin) runs over the bin means, resetting at data gaps.

    Returns (bin centers, smoothed values); empty bins are NaN so the
    smoothed line breaks where no data exists.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if t.size < 2:
        return t, y
    edges = np.arange(t.min(), t.max() + bin_s, bin_s)
    centers = (edges[:-1] + edges[1:]) / 2.0
    idx = np.clip(np.searchsorted(edges, t, side="right") - 1,
                  0, centers.size - 1)
    sums = np.zeros(centers.size)
    cnts = np.zeros(centers.size)
    good = ~np.isnan(y)
    if good.any():
        np.add.at(sums, idx[good], y[good])
        np.add.at(cnts, idx[good], 1)
    means = np.where(cnts > 0, sums / np.maximum(cnts, 1), np.nan)
    out = np.full(centers.size, np.nan)
    mask = ~np.isnan(means)
    if mask.any():
        bounds = np.flatnonzero(np.diff(mask.astype(int)) != 0) + 1
        for seg in np.split(np.arange(centers.size), bounds):
            if mask[seg[0]]:
                out[seg] = pd.Series(means[seg]).ewm(alpha=alpha).mean()
    return centers, out


def _uav_rnti_mask(df, ue_meas):
    """Mask scheduling rows that belong to the UAV, tracking RNTI changes.

    The UAV's C-RNTI is reallocated by the target gNB at every handover, so a
    fixed `rnti == 1` filter silently drops the UAV's scheduling after the
    first handover - and can even pick up an interferer's entries, because
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


def _split_segments(t, y, max_gap):
    """Split (t, y) into segments at gaps larger than max_gap.

    Returns a list of (t_seg, y_seg) numpy arrays. Needed because
    matplotlib step() does not break lines at NaN, so gap-breaking must
    be done by plotting each segment separately.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(t) < 2:
        return [(t, y)]
    cut = np.flatnonzero(np.diff(t) > max_gap) + 1
    segs = []
    prev = 0
    for i in list(cut) + [len(t)]:
        segs.append((t[prev:i], y[prev:i]))
        prev = i
    return segs


def _step_segments(ax, t, y, max_gap, where, **kw):
    """Step-plot (t, y) on ax, breaking the line at gaps > max_gap.

    The label is attached to the first drawn segment only, so isolated
    leading samples (which are skipped) cannot swallow the legend entry.
    """
    labeled = False
    for ts, ys in _split_segments(t, y, max_gap):
        if len(ts) < 2:
            continue
        seg_kw = dict(kw)
        if labeled:
            seg_kw.pop("label", None)
        ax.step(ts, ys, where=where, **seg_kw)
        labeled = True


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


# Positional input + -o mirror run-evaluations.py; relative inputs resolve
# against the current directory.
parser = argparse.ArgumentParser(
    description="Plot per-seed NR-RL handover statistics from the CSV logs "
                "in a data dir (a seed dir under results/ or output/).")
parser.add_argument("input", nargs="?", type=str, default=None,
                    help="data dir with the nr-rl-*.csv logs "
                         "(default: <example_dir>/output)")
parser.add_argument("-o", "--output", type=str, default=None,
                    help="output PNG path (default: <input>/nr-rl-stats.png)")
parser.add_argument("--smooth", nargs="?", type=int, const=100, default=None,
                    help="rolling window (samples) for the SINR traces "
                         "(median DL / mean UL; default 100); 1 disables it")
parser.add_argument("--raw", action="store_true",
                    help="plot raw DL/UL SINR (no rolling window) so "
                         "fast-fading / channel-update staircase artifacts "
                         "are visible")
parser.add_argument("--ewma", nargs="?", type=float, const=0.05, default=None,
                    help="EWMA smoothing over a 100 ms time grid (alpha per "
                         "bin; 0.05 ~ 1.4 s half-life): smoothed series at "
                         "full alpha over the raw at low alpha, for the "
                         "continuous noisy series only (SINR, serving-cell "
                         "RB utilization, PHY rate and MCS; discrete signals "
                         "like cwnd/RTT/corruption stay raw); overrides "
                         "--smooth/--raw on SINR")
parser.add_argument("--split-figures", action="store_true",
                    help="save each panel as its own PNG under "
                         "<out_dir>/figures/ (panels without data are skipped)")
parser.add_argument("--panels", type=str, default=None,
                    help="comma-separated panel names to draw, in order, as one "
                         "full-width row each (default: all populated panels); "
                         "names: cwnd,rtt,sinr,phy-rate,mcs,corrupt,goodput,"
                         "serving,reward,rb-util,rsrp-cells,rsrq-cells")
args = parser.parse_args()


def main(argv=None):
    example_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = (os.path.join(example_dir, 'output')) if args.input is None \
        else os.path.abspath(args.input)
    if not os.path.isdir(data_dir):
        sys.exit(f"[ERROR] data dir not found: {data_dir} (see --help)")
    # Reject multi-seed dirs (scenario dir or eval root) with a pointer to a
    # concrete seed dir; the plots are per single episode.
    for child in sorted(os.listdir(data_dir)):
        p = os.path.join(data_dir, child)
        if child.startswith("seed_"):
            example = p
            break
        if os.path.isdir(p) and any(x.startswith("seed_") for x in os.listdir(p)):
            example = os.path.join(p, "seed_1")
            break
    else:
        example = None
    if example:
        sys.exit(f"[ERROR] {data_dir} is a multi-seed dir; pass one seed dir "
                 f"(e.g. {example}) (see --help)")
    if not any(os.path.exists(os.path.join(data_dir, f))
               for f in ("nr-rl-slot-stats.csv", "meta.yaml")):
        sys.exit(f"[ERROR] no per-run logs in {data_dir}; pass a seed dir "
                 f"(results/<eval>/<tag>/seed_N) or a run dir under output/ "
                 "(see --help)")
    out_file = os.path.abspath(args.output) if args.output \
        else os.path.join(data_dir, "nr-rl-stats.png")
    ewma_alpha = args.ewma

    # ---- data loading -----------------------------------------------------
    slot_stats = pd.read_csv(
        os.path.join(data_dir, 'nr-rl-slot-stats.csv'), skiprows=1,
        header=None, names=["time", "cellId", "scheduledUe", "usedReg",
                            "usedSym", "availableRb", "availableSym",
                            "utilPct"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-slot-stats.csv')) \
        else None

    cwnd = pd.read_csv(os.path.join(data_dir, 'nr-rl-cwnd.csv'), skiprows=1,
                       header=None, names=["time", "cwnd"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-cwnd.csv')) else None
    # Multiple CWND updates can share a timestamp (ACK bursts): keep the last.
    # Guard against counter wrap only (uint32 ceiling, ~4 GiB); cwnd legitimately
    # reaches tens of MiB when slow start never exits.
    if cwnd is not None:
        cwnd = cwnd[cwnd["cwnd"] <= 0xFFFFFFFF]
        cwnd = cwnd.drop_duplicates(subset="time", keep="last").sort_values("time")

    # Real window ceiling for the flow (meta.yaml since 2026-09-08).
    sndbuf_bytes = 1 << 20
    _meta = os.path.join(data_dir, "meta.yaml")
    if os.path.exists(_meta):
        for _line in open(_meta):
            if _line.startswith("tcpSndBufBytes:"):
                try:
                    sndbuf_bytes = int(_line.split(":", 1)[1].strip())
                except ValueError:
                    pass

    ho = pd.read_csv(os.path.join(data_dir, 'nr-rl-handovers.csv'),
                     skiprows=1, header=None, names=["time", "cellId"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-handovers.csv')) \
        else None

    rtt = pd.read_csv(os.path.join(data_dir, 'nr-rl-rtt.csv'), skiprows=1,
                      header=None, names=["time", "rtt"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-rtt.csv')) else None
    if rtt is not None:
        rtt = rtt.drop_duplicates(subset="time", keep="last").sort_values("time")


    rsrp_rsrq_full = pd.read_csv(os.path.join(data_dir, 'ue_meas_report.csv'),
                                 skiprows=1, header=None,
                                 names=["time", "cellId", "rnti", "rsrp",
                                        "rsrq", "isServingCell"]) \
        if os.path.exists(os.path.join(data_dir, 'ue_meas_report.csv')) \
        else None
    rsrp_rsrq = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1].copy() \
        if rsrp_rsrq_full is not None and not rsrp_rsrq_full.empty else None

    ul_sinr_srs = pd.read_csv(os.path.join(data_dir, 'ul_sinr_srs.csv'),
                              skiprows=1, header=None,
                              names=["time", "cellId", "sinr"]) \
        if os.path.exists(os.path.join(data_dir, 'ul_sinr_srs.csv')) else None

    ul_sched = pd.read_csv(os.path.join(data_dir, 'nr-rl-ul-sched.csv'),
                           skiprows=1, header=None,
                           names=["time", "cellId", "rnti", "mcs", "tbSize",
                                  "symStart", "numSym"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-ul-sched.csv')) \
        else None

    dl_sinr = pd.read_csv(os.path.join(data_dir, 'dl_sinr.csv'), skiprows=1,
                          header=None, names=["time", "cellId", "rnti", "sinr"]) \
        if os.path.exists(os.path.join(data_dir, 'dl_sinr.csv')) else None

    dl_rx = pd.read_csv(os.path.join(data_dir, 'nr-rl-dl-rx-sinr.csv'),
                        skiprows=1, header=None,
                        names=["time", "cellId", "rnti", "sinrDb", "mcs",
                               "tbSize", "corrupt", "tbler"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-dl-rx-sinr.csv')) \
        else None
    ul_rx = pd.read_csv(os.path.join(data_dir, 'nr-rl-ul-rx-sinr.csv'),
                        skiprows=1, header=None,
                        names=["time", "cellId", "rnti", "sinrDb", "mcs",
                               "tbSize", "corrupt", "tbler"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-ul-rx-sinr.csv')) \
        else None

    dl_sched = pd.read_csv(os.path.join(data_dir, 'nr-rl-dl-sched.csv'),
                           skiprows=1, header=None,
                           names=["time", "cellId", "rnti", "mcs", "tbSize",
                                  "symStart", "numSym"]) \
        if os.path.exists(os.path.join(data_dir, 'nr-rl-dl-sched.csv')) \
        else None

    rl_reward = pd.read_csv(os.path.join(data_dir, 'rl_reward.csv'),
                            skiprows=1, header=None,
                            names=["time", "goodput_mbps", "dynRef_mbps",
                                   "dynMin_mbps", "normGoodputRaw",
                                   "normGoodput", "rg", "iHo", "rH",
                                   "pingPong", "reward"]) \
        if os.path.exists(os.path.join(data_dir, 'rl_reward.csv')) else None

    sink = pd.read_csv(os.path.join(data_dir, 'sink-packets.csv'), skiprows=1,
                       header=None, names=["time", "size"]) \
        if os.path.exists(os.path.join(data_dir, 'sink-packets.csv')) else None

    # ---- panel availability ----------------------------------------------
    def ok(df):
        return df is not None and not df.empty

    has_cwnd = ok(cwnd)
    has_rtt = ok(rtt)
    has_sinr = ok(dl_sinr) or ok(ul_sinr_srs)
    has_corrupt = ok(dl_rx) or ok(ul_rx)
    has_goodput = ok(sink)
    has_serving = ok(rsrp_rsrq)
    has_reward = ok(rl_reward)
    has_rbutil = (not has_reward) and ok(slot_stats)
    has_cells = ok(rsrp_rsrq_full)

    # UAV scheduling rows (C-RNTI tracked across handovers): one mask pass
    # shared by the PHY-throughput and MCS panels.
    dl_uav = dl_sched[_uav_rnti_mask(dl_sched, rsrp_rsrq_full)].copy() \
        if ok(dl_sched) else None
    ul_uav = ul_sched[_uav_rnti_mask(ul_sched, rsrp_rsrq_full)].copy() \
        if ok(ul_sched) else None
    has_phy = ok(dl_uav) or ok(ul_uav)

    # Max simulation time seen across the loaded series (shared x range).
    max_time = 0.0
    for frame in (cwnd, rtt, dl_sinr, ul_sinr_srs, rsrp_rsrq,
                  sink, rl_reward, slot_stats, ul_sched, dl_sched):
        if ok(frame):
            max_time = max(max_time, float(frame["time"].max()))
    if max_time <= 0.0:
        max_time = 0.0

    # ---- figure/legend helpers -------------------------------------------
    legend_specs = []  # (ax, handles, labels, loc, bbox) recorded per panel

    def _record_legend(ax, handles, labels, loc="upper left", bbox=None,
                       fontsize=6, owner=None):
        # owner is the axes that carries the legend. It differs from ax on
        # panels with a twin axis, because the twin is drawn after the primary
        # axis and would otherwise paint over the legend.
        legend_specs.append(dict(ax=ax,
                                 owner=owner if owner is not None else ax,
                                 handles=list(handles), labels=list(labels),
                                 loc=loc, bbox=bbox, fontsize=fontsize))

    def _draw_ho_markers(ax):
        if ho is not None and not ho.empty:
            for _, row in ho.iterrows():
                t = row["time"]
                ax.axvline(x=t, color="green", linestyle="--",
                           alpha=0.85, linewidth=1.3)
                # Tick below the axis so the handover times stay legible when
                # the dashed line is thin or the panel is crowded.
                ax.plot([t, t], [-0.03, 0.0],
                        transform=ax.get_xaxis_transform(),
                        color="green", linewidth=2.0, clip_on=False,
                        solid_capstyle="butt", zorder=5)

    def _legend(ax, handles, labels, **kw):
        """Panel legend drawn above every artist, with an opaque frame so the
        traced lines cannot show through the labels."""
        kw.setdefault("framealpha", 1.0)
        leg = ax.legend(handles, labels, **kw)
        leg.set_zorder(60)
        return leg

    def _attach_ho_legend():
        """Add one 'Handover' entry to the last recorded legend of this
        figure (vlines appear on every panel, but only one legend entry)."""
        if not legend_specs or ho is None or ho.empty:
            return
        spec = legend_specs[-1]
        spec["handles"].append(Line2D([0], [0], color="green", linestyle="--",
                                      linewidth=0.8))
        spec["labels"].append("Handover")
        kw = dict(fontsize=spec["fontsize"], loc=spec["loc"])
        if spec["bbox"] is not None:
            kw["bbox_to_anchor"] = spec["bbox"]
        _legend(spec["owner"], spec["handles"], spec["labels"], **kw)

    def _render_legends():
        """Render every recorded panel legend, then add the figure-wide
        'Handover' entry to the last one."""
        for spec in legend_specs:
            kw = dict(fontsize=spec["fontsize"], loc=spec["loc"])
            if spec["bbox"] is not None:
                kw["bbox_to_anchor"] = spec["bbox"]
            _legend(spec["owner"], spec["handles"], spec["labels"], **kw)
        _attach_ho_legend()

    def _pad_top_for_legend(fig):
        """Extend the y-range top of every inside-axes legend so the legend's
        own height is always empty space above the data (legend box is
        anchored to the axes top, so data must be pushed down below it).
        The headroom is reserved on the panel's own axis only, never on a twin
        axis: the twin carries bounded quantities in the reward panel, so
        pushing its top would show a normalized axis above one."""
        fig.canvas.draw()
        renderer = fig.canvas.get_renderer()
        for spec in legend_specs:
            if spec["bbox"] is not None:
                continue  # legend outside the axes, nothing overlaps
            lg = spec["owner"].get_legend()
            if lg is None:
                continue
            leg_h = lg.get_window_extent(renderer).height
            ax = spec["ax"]
            ax_h = ax.get_window_extent(renderer).height
            if ax_h <= 0 or leg_h <= 0:
                continue
            ylo, yhi = ax.get_ylim()
            span = yhi - ylo
            if span <= 0:
                continue
            # Data must stay below (1 - f_leg) of the axes height.
            f_leg = min(leg_h / ax_h, 0.85)
            ax.set_ylim(ylo, yhi + span * f_leg / (1.0 - f_leg))

    def _draw_series(ax, t, y, max_gap, color, label, kind="line", where=None,
                     lw=1.0, smooth_lw=1.2, ls="-", raw_alpha=0.25):
        """Plot y(t) with gap splitting. With --ewma: raw ghost at low alpha
        plus a time-binned EWMA (100 ms grid) at full alpha. kind 'step'
        step-plots y, 'line' breaks at NaNs instead."""
        if ewma_alpha is None:
            if kind == "step":
                _step_segments(ax, t, y, max_gap, where, linewidth=lw,
                               color=color, linestyle=ls, label=label)
            else:
                t2, y2 = _insert_nan_at_gaps(np.asarray(t, float),
                                             np.asarray(y, float), max_gap)
                ax.plot(t2, y2, linewidth=lw, color=color, linestyle=ls,
                        label=label)
            return
        tc, sm = _ewma_time(t, y, ewma_alpha)
        if kind == "step":
            _step_segments(ax, t, y, max_gap, where, linewidth=lw,
                           color=color, linestyle=ls, alpha=raw_alpha)
        else:
            t2, y2 = _insert_nan_at_gaps(np.asarray(t, float),
                                         np.asarray(y, float), max_gap)
            ax.plot(t2, y2, linewidth=lw, color=color, linestyle=ls,
                    alpha=raw_alpha)
        ax.plot(tc, sm, linewidth=smooth_lw, color=color, linestyle=ls,
                label=label)

    def _ewma_suffix(name):
        return f"{name} (EWMA a={ewma_alpha:.2g})" if ewma_alpha is not None \
            else name

    # ---- panel draw functions --------------------------------------------
    def draw_cwnd(ax):
        # ACK-driven staircase: the sawtooth is the TCP signal, so it is
        # always drawn raw (no EWMA/ghost). Two series: the raw cwnd and the
        # EFFECTIVE window min(cwnd, socket buffer), which is what actually
        # limits the flow. Without losses cwnd inflates far past the socket
        # buffer, so the raw curve alone does not track the achieved goodput.
        t = cwnd["time"].values
        y = cwnd["cwnd"].values
        eff = np.minimum(y, sndbuf_bytes)
        _step_segments(ax, t, y / 1024.0, 0.2, "post", linewidth=0.6,
                       color="tab:brown", alpha=0.35, label="cwnd (raw)")
        _step_segments(ax, t, eff / 1024.0, 0.2, "post", linewidth=1.2,
                       color="tab:brown",
                       label="min(cwnd, socket buffer)")
        ax.axhline(y=sndbuf_bytes / 1024.0, color="black", linestyle="--",
                   linewidth=0.8, alpha=0.7,
                   label=f"socket buffer {sndbuf_bytes / 2**20:.2f} MiB")
        # Keep the readable 0..2.2x socket-buffer band: when the loss-free flow
        # inflates cwnd far past the buffer, the raw curve leaves the axes.
        view_kb = 2.2 * sndbuf_bytes / 1024.0
        if y.max() / 1024.0 > view_kb:
            ax.set_ylim(0, view_kb)
        ax.set_ylabel("cwnd (KiB)")
        ax.set_title("TCP Congestion Window over Time")
        ax.grid(True)
        _record_legend(ax, *ax.get_legend_handles_labels(), loc="upper left")

    def draw_rtt(ax):
        # ACK-driven staircase, raw only (no EWMA/ghost): RTT spikes at
        # handovers are the signal this panel shows.
        _step_segments(ax, rtt["time"].values, rtt["rtt"].values, 0.2,
                       "post", linewidth=1.0, color="tab:blue")
        ax.set_ylabel("Round Trip Time (ms)")
        ax.set_title("TCP Round Trip Time over Time")
        ax.grid(True)
        ax.set_ylim(0, max(float(rtt["rtt"].max()) * 1.5, 50))
        ax.axhline(y=20.0, color="red", linestyle="--", alpha=0.6,
                   linewidth=0.8, label="PGW-Server RTT")

    def draw_sinr(ax):
        ax.set_ylabel("SINR (dB)")
        ax.set_title("DL and UL SINR over Time (serving cell)")
        ax.grid(True)
        if ok(dl_sinr):
            # DlDataSinr trace fires only for the serving cell (UE PHY).
            t = dl_sinr["time"].to_numpy()
            y = dl_sinr["sinr"].to_numpy()
            if ewma_alpha is not None:
                _draw_series(ax, t, y, 0.2, "tab:orange",
                             _ewma_suffix("DL SINR"))
            else:
                win = 1 if args.raw else (args.smooth or 100)
                s = dl_sinr["sinr"].rolling(window=win, center=True,
                                            min_periods=1).median()
                t2, y2 = _insert_nan_at_gaps(t, s.to_numpy(), 0.2)
                ax.plot(t2, y2, linewidth=0.8, color="tab:orange", alpha=0.7,
                        label="DL SINR")
        if ok(ul_sinr_srs):
            # The UlSrsSinrLogger already filters to the serving cell.
            t = ul_sinr_srs["time"].to_numpy()
            y = ul_sinr_srs["sinr"].to_numpy()
            if ewma_alpha is not None:
                _draw_series(ax, t, y, 0.2, "tab:blue",
                             _ewma_suffix("UL SINR"))
            else:
                win = 1 if args.raw else (args.smooth or 100)
                s = ul_sinr_srs["sinr"].rolling(window=win, center=True,
                                                min_periods=1).mean()
                t2, y2 = _insert_nan_at_gaps(t, s.to_numpy(), 0.2)
                ax.plot(t2, y2, linewidth=0.8, color="tab:blue",
                        alpha=0.7, label="UL SINR")
        _record_legend(ax, *ax.get_legend_handles_labels(), loc="upper right")

    def draw_rate(ax):
        # Scheduled PHY-layer throughput of the UAV derived from the per-slot
        # TBS: an upper bound on the link rate (ignores CRC failures and HARQ
        # retransmissions), DL orange / UL blue dotted. The app-layer
        # delivery picture lives in the goodput panel.
        ax.set_ylabel("Rate (Mbps)")
        ax.set_title("UL/DL PHY Rate (TBS max)")
        ax.grid(True)
        for uav, color, ls, base, alph in (
                (dl_uav, "tab:orange", "-", "DL PHY", 0.9),
                (ul_uav, "tab:blue", ":", "UL PHY", 0.7)):
            if not ok(uav):
                continue
            rate_mbps = uav["tbSize"] * 8.0 / 1e-3 / 1e6
            t = uav["time"].to_numpy()
            if ewma_alpha is None:
                y = rate_mbps.rolling(window=100, min_periods=1).mean()
                _step_segments(ax, t, y.to_numpy(), 0.2, "post",
                               linewidth=0.8, color=color, linestyle=ls,
                               label=f"{base} (100-slot avg)", alpha=alph)
            else:
                _draw_series(ax, t, rate_mbps.to_numpy(), 0.2, color,
                             f"{base} (EWMA a={ewma_alpha:.2g})",
                             kind="step", where="post", lw=0.8)
        _record_legend(ax, *ax.get_legend_handles_labels())

    def draw_mcs(ax):
        # Scheduled MCS of the UAV per direction; the default draws a
        # 100-slot average, --ewma draws the raw per-slot steps as a faint
        # ghost under the EWMA line.
        ax.set_ylabel("MCS index")
        ax.set_title("UL/DL MCS")
        ax.set_ylim(-1, 29)
        ax.grid(True)
        for uav, color, base in ((dl_uav, "tab:green", "DL MCS"),
                                 (ul_uav, "tab:blue", "UL MCS")):
            if not ok(uav):
                continue
            t = uav["time"].to_numpy()
            if ewma_alpha is None:
                y = uav["mcs"].rolling(window=100, min_periods=1).mean()
                _step_segments(ax, t, y.to_numpy(), 0.2, "post",
                               linewidth=1.0, color=color,
                               label=f"{base} (100-slot avg)")
            else:
                _draw_series(ax, t, uav["mcs"].to_numpy(), 0.2, color,
                             f"{base} (EWMA a={ewma_alpha:.2g})",
                             kind="step", where="post", lw=0.8)
        _record_legend(ax, *ax.get_legend_handles_labels())

    def draw_corrupt(ax):
        # Corrupt % per 200 ms bin (same cadence as the goodput panel):
        # fraction of TBs with CRC failure (rx-sinr corrupt flag), drawn as
        # a step, plus the run average as a dashed line per direction. Bins
        # without TBs are NaN and break the step.
        ax.set_ylabel("corrupt %")
        ax.set_title("Per-TB Corruption (200 ms bins)")
        ax.grid(True)
        bin_w = 0.2
        for rx_df, color, lbl in ((dl_rx, "tab:green", "DL"),
                                  (ul_rx, "tab:orange", "UL")):
            if not ok(rx_df):
                continue
            t = rx_df["time"].to_numpy()
            c = rx_df["corrupt"].to_numpy(dtype=float)
            bins = np.arange(t.min(), t.max() + bin_w, bin_w)
            sums, _ = np.histogram(t, bins=bins, weights=c)
            cnts, _ = np.histogram(t, bins=bins)
            with np.errstate(divide="ignore", invalid="ignore"):
                pct = np.where(cnts > 0, sums / cnts * 100.0, np.nan)
            overall = c.mean() * 100.0
            ax.step(bins[:-1], pct, where="post", linewidth=0.9,
                    color=color, label=f"{lbl} (avg {overall:.1f}%)")
            ax.axhline(overall, color=color, linestyle="--", linewidth=0.6,
                       alpha=0.5)
        _record_legend(ax, *ax.get_legend_handles_labels(), loc="upper right")

    def draw_goodput(ax):
        # Goodput at the sink only: the source/throughput curve is the fixed
        # offered rate on UDP and equals goodput on TCP, so it adds no
        # information. Cumulative goodput rides the twin axis.
        axb = ax.twinx()
        t_snk, gput = bin_data_mbps(sink)
        ga = ewma_alpha if ewma_alpha is not None else 0.2
        if ewma_alpha is None:
            ax.step(t_snk, gput, linewidth=1.0, color="tab:purple",
                    label="Goodput (sink)", alpha=0.8)
        else:
            ax.step(t_snk, gput, linewidth=0.8, color="tab:purple",
                    alpha=0.25)
        ewma_gput = pd.Series(gput, index=t_snk).ewm(alpha=ga).mean()
        ax.plot(ewma_gput.index, ewma_gput.values, linewidth=1.0,
                color="tab:orange", linestyle="--",
                label=f"Goodput EWMA (a={ga:.2g})")
        t_cum, cum = cumulative_bytes(sink)
        axb.step(t_cum, cum / 1e6, linewidth=1.0, color="tab:purple",
                 linestyle="--", label="cum. goodput (MB)", alpha=0.6)
        ax.set_ylabel("Goodput (Mbps)")
        axb.set_ylabel("Cumulative (MB)")
        ax.set_title("Goodput at Sink (200 ms bins)")
        ax.grid(True)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axb.get_legend_handles_labels()
        _record_legend(ax, h1 + h2, l1 + l2, owner=axb)

    def draw_serving(ax):
        axb = ax.twinx()
        t_rp, y_rp = _insert_nan_at_gaps(
            rsrp_rsrq["time"].to_numpy(), rsrp_rsrq["rsrp"].to_numpy(), 0.6)
        ax.plot(t_rp, y_rp, linewidth=1.2, color="tab:orange",
                label="RSRP 200ms serving")
        ax.set_ylabel("RSRP (dBm)")
        ax.set_title("Serving Cell RSRP and RSRQ")
        ax.grid(True)
        t_rq, y_rq = _insert_nan_at_gaps(
            rsrp_rsrq["time"].to_numpy(), rsrp_rsrq["rsrq"].to_numpy(), 0.6)
        axb.plot(t_rq, y_rq, linewidth=0.8, color="tab:purple",
                 linestyle=":", alpha=0.7, label="RSRQ 200ms serving")
        axb.set_ylabel("RSRQ (dB)")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axb.get_legend_handles_labels()
        _record_legend(ax, h1 + h2, l1 + l2, owner=axb)

    def draw_reward(ax):
        t = rl_reward["time"]
        ax.plot(t, rl_reward["goodput_mbps"], linewidth=1.0,
                color="tab:purple", label="Goodput (sink Mbps)")
        ax.set_ylabel("Goodput (Mbps)")
        ax.set_title("Reward Components")
        ax.grid(True)
        axb = ax.twinx()
        axb.plot(t, rl_reward["rg"], linewidth=0.8, color="tab:cyan",
                 linestyle="--", alpha=0.6, label="R_G (goodput term)")
        axb.plot(t, rl_reward["rH"], linewidth=1.1, color="tab:orange",
                 linestyle="-.", alpha=0.8, label="R_H (handover term)")
        axb.plot(t, rl_reward["reward"], linewidth=1.2, color="tab:red",
                 label="Reward (total)")
        axb.axhline(y=0, color="gray", linestyle=":", alpha=0.3,
                    linewidth=0.5)
        axb.set_ylabel("Normalized reward components")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axb.get_legend_handles_labels()
        _record_legend(ax, h1 + h2, l1 + l2, loc="upper right", fontsize=5,
                       owner=axb)

    def draw_rbutil(ax):
        # Serving-cell RB utilization, per slot (1 ms). The default draw uses
        # a centered 200-slot rolling average; with --ewma the raw slot
        # occupancy shows as a faint band under the EWMA.
        ss = slot_stats.sort_values("time")
        t_ss = ss["time"].to_numpy()
        y_ss = ss["utilPct"].to_numpy()
        if ewma_alpha is None:
            util_smooth = ss["utilPct"].rolling(
                window=200, min_periods=1, center=True).mean()
            _step_segments(ax, t_ss, util_smooth.to_numpy(), 0.2, "post",
                           linewidth=0.8, color="tab:red", alpha=0.8,
                           label="RB util (%)")
        else:
            _draw_series(ax, t_ss, y_ss, 0.2, "tab:red", "RB util (%)",
                         kind="step", where="post", lw=0.6)
        ax.set_ylabel("Utilization (%)")
        ax.set_ylim(-5, 105)
        ax.grid(True)
        if ss["scheduledUe"].max() > 1:
            busy = ss[ss["scheduledUe"] > 1]
            ax.plot(busy["time"], [100] * len(busy), "v", color="darkred",
                    markersize=3, alpha=0.5,
                    label=f">1 UE ({len(busy)} slots)")
            ax.set_title("Serving Cell RB Utilization (per slot)")
        else:
            ax.set_title("Serving Cell RB Utilization")
        _record_legend(ax, *ax.get_legend_handles_labels(), loc="upper right")

    def _draw_cells(ax, column, ylabel, title):
        # Per-cell measurements with the serving-cell highlight overlaid as a
        # continuous bold trace, coloured by the serving cell of each edge.
        cell_ids = sorted(rsrp_rsrq_full["cellId"].unique())
        colors = plt.cm.gist_ncar(np.linspace(0, 0.9, len(cell_ids)))
        for idx, cell_id in enumerate(cell_ids):
            cell_data = rsrp_rsrq_full[rsrp_rsrq_full["cellId"] == cell_id]
            ax.plot(cell_data["time"], cell_data[column], linewidth=0.6,
                    color=colors[idx], alpha=0.5,
                    label=f"Cell {int(cell_id)}")
        # Serving highlight: connect every consecutive serving sample so even
        # periods shorter than the report interval (fast ping-pong) stay
        # visible - a single sample is drawn as the tail of the previous edge
        # rather than an isolated dot. The trace breaks only where the
        # serving report itself is missing (> 0.5 s with no serving sample).
        # Each edge is attributed to the serving cell of its END sample and is
        # anchored on that cell's own measurement at both report instants, so
        # at a handover the t -> t+1 edge starts from the NEW cell's RSRP at t
        # (not the old cell's value), which also makes the colour switch at t.
        serving = rsrp_rsrq_full[rsrp_rsrq_full["isServingCell"] == 1] \
            .sort_values("time")
        color_of = {int(cid): col for cid, col in zip(cell_ids, colors)}
        if not serving.empty:
            t_srv = serving["time"].to_numpy()
            y_srv = serving[column].to_numpy()
            c_srv = serving["cellId"].to_numpy()
            # Every cell's measurement at every report time, so a handover
            # edge can be anchored on the new cell's own value.
            meas = {(int(cid), float(tt)): vv
                    for cid, tt, vv in zip(rsrp_rsrq_full["cellId"],
                                           rsrp_rsrq_full["time"],
                                           rsrp_rsrq_full[column])}
            pairs = np.column_stack([np.arange(len(t_srv) - 1),
                                     np.arange(1, len(t_srv))]) \
                if len(t_srv) > 1 else np.empty((0, 2), dtype=int)
            if len(pairs):
                keep = np.diff(t_srv) <= 0.5
                pairs = pairs[keep]
                edge_cell = c_srv[pairs[:, 1]]
                p_start = np.column_stack([
                    t_srv[pairs[:, 0]],
                    [meas.get((int(c), float(tt)), np.nan)
                     for c, tt in zip(edge_cell, t_srv[pairs[:, 0]])]])
                p_end = np.column_stack([t_srv[pairs[:, 1]],
                                         y_srv[pairs[:, 1]]])
                ax.add_collection(LineCollection(
                    np.stack([p_start, p_end], axis=1),
                    colors=[color_of[int(c)] for c in edge_cell],
                    linewidths=1.4))
            # Serving samples isolated by a reporting gap on both sides.
            isolated = np.ones(len(t_srv), dtype=bool)
            if len(t_srv) > 1:
                gaps = np.diff(t_srv) > 0.5
                isolated[1:] &= gaps
                isolated[:-1] &= gaps
            if isolated.any():
                ax.scatter(t_srv[isolated], y_srv[isolated], s=8, zorder=3,
                           c=[color_of[int(cid)] for cid in c_srv[isolated]])
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        _record_legend(ax, *ax.get_legend_handles_labels(),
                       bbox=(1.02, 1.0))

    def draw_rsrp_cells(ax):
        _draw_cells(ax, "rsrp", "RSRP (dBm)", "RSRP per Cell")

    def draw_rsrq_cells(ax):
        _draw_cells(ax, "rsrq", "RSRQ (dB)", "RSRQ per Cell")

    # Ordered panel list: (name, draw fn, full-width row).
    half_panels = []
    if has_cwnd:
        half_panels.append(("cwnd", draw_cwnd))
    if has_rtt:
        half_panels.append(("rtt", draw_rtt))
    if has_sinr:
        half_panels.append(("sinr", draw_sinr))
    if has_phy:
        half_panels.append(("phy-rate", draw_rate))
        half_panels.append(("mcs", draw_mcs))
    if has_corrupt:
        half_panels.append(("corrupt", draw_corrupt))
    if has_goodput:
        half_panels.append(("goodput", draw_goodput))
    if has_serving:
        half_panels.append(("serving", draw_serving))
    if has_reward:
        half_panels.append(("reward", draw_reward))
    elif has_rbutil:
        half_panels.append(("rb-util", draw_rbutil))
    full_panels = []
    if has_cells:
        full_panels.append(("rsrp-cells", draw_rsrp_cells))
        full_panels.append(("rsrq-cells", draw_rsrq_cells))

    if args.panels:
        # Curated selection: one full-width row per requested panel, since the
        # default two-column grid is tuned for the full panel set.
        available = dict(half_panels + full_panels)
        want = [p.strip() for p in args.panels.split(",") if p.strip()]
        missing = [p for p in want if p not in available]
        if missing:
            sys.exit(f"[ERROR] panel(s) not available: {', '.join(missing)}; "
                     f"choose from: {', '.join(available)}")
        half_panels, full_panels = [], [(p, available[p]) for p in want]

    if args.split_figures:
        # ---- one PNG per populated panel --------------------------------
        figs_dir = os.path.join(os.path.dirname(out_file), "figures")
        os.makedirs(figs_dir, exist_ok=True)
        idx = 0
        for name, fn in half_panels + full_panels:
            idx += 1
            fig = plt.figure(figsize=(12, 3.6))
            ax = fig.add_subplot(111)
            ax.minorticks_on()
            legend_specs.clear()
            fn(ax)
            _draw_ho_markers(ax)
            ax.set_xlim(0, max_time * 1.05)
            ax.set_xlabel("Time (s)")
            _render_legends()
            _pad_top_for_legend(fig)
            fig.tight_layout()
            path = os.path.join(figs_dir, f"{idx:02d}-{name}.png")
            fig.savefig(path, dpi=300, bbox_inches="tight")
            plt.close(fig)
            print(f"Saved {path}")
        print(f"Split figures written to {figs_dir}")

    # ---- composite grid figure -------------------------------------------
    n_half_rows = (len(half_panels) + 1) // 2
    n_rows = n_half_rows + len(full_panels)
    if n_rows == 0:
        sys.exit("[ERROR] no plottable data in this directory (see --help)")
    fig = plt.figure(figsize=(12, 2.9 * n_rows + 0.5))
    gs = gridspec.GridSpec(n_rows, 2, figure=fig)
    legend_specs.clear()
    first_ax = None
    axes = []
    for i, (name, fn) in enumerate(half_panels):
        ax = fig.add_subplot(gs[i // 2, i % 2], sharex=first_ax)
        if first_ax is None:
            first_ax = ax
        axes.append(ax)
        ax.minorticks_on()
        fn(ax)
    for j, (name, fn) in enumerate(full_panels):
        ax = fig.add_subplot(gs[n_half_rows + j, :], sharex=first_ax)
        if first_ax is None:
            first_ax = ax
        axes.append(ax)
        ax.minorticks_on()
        fn(ax)

    # Bottom-most axes of each column keep the "Time (s)" label.
    for i, ax in enumerate(axes):
        if i < len(half_panels):
            bottom = (i // 2) == n_half_rows - 1
        else:
            bottom = True  # full-width rows are always at the bottom
        ax.tick_params(labelbottom=bottom)
        if bottom:
            ax.set_xlabel("Time (s)")

    if first_ax is not None:
        first_ax.set_xlim(0, max_time * 1.05)
    for ax in axes:
        _draw_ho_markers(ax)
    _render_legends()
    _pad_top_for_legend(fig)

    fig.tight_layout()
    fig.savefig(out_file, dpi=300, bbox_inches="tight")
    print(f"Plot saved to {out_file}")
    if not args.split_figures:
        plt.show()


if __name__ == "__main__":
    main(sys.argv)
