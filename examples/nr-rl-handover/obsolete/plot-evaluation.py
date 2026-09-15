#!/usr/bin/env python3
"""
plot-evaluation.py — Thesis-quality plots from run-evaluation.py output.

Supports tags with format: {transport}-{direction}-{if_type}
e.g. tcp-dl-aerial-if, udp-ul-no-if

Usage:
    python3 plot-evaluation.py results/ --output figures/
"""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "legend.fontsize": 8,
    "xtick.labelsize": 9,
    "ytick.labelsize": 10,
    "figure.dpi": 150,
})

DIR_COLORS = {"ul": "#4C72B0", "dl": "#DD8452"}
TP_COLORS = {"tcp": "#4C72B0", "udp": "#DD8452"}
IF_LABELS = {
    "no-if": "No IF", "aerial-if": "Aerial IF",
    "ground-if": "Ground IF", "mixed-if": "Mixed IF",
}
IF_ORDER = ["no-if", "aerial-if", "ground-if", "mixed-if"]


def parse_tag(tag: str):
    """Parse tag like 'tcp-a3-dl-aerial-if' or 'tcp-dl-aerial-if'.

    New format (5 parts):  {transport}-{algorithm}-{direction}-{if_type}
    Old format (4 parts):  {transport}-{direction}-{if_type}  (algorithm defaults to "a3")
    Returns (transport, algorithm, direction, if_type).
    """
    parts = tag.split("-")
    if len(parts) == 5:
        return parts[0], parts[1], parts[2], "-".join(parts[3:])
    else:
        # old format, default algorithm to "a3"
        return parts[0], "a3", parts[1], "-".join(parts[2:])


def load_data(results_dir: Path):
    """Load raw.csv from each scenario. Returns DataFrame."""
    rows = []
    for d in sorted(results_dir.iterdir()):
        if not d.is_dir() or not d.name[0].isalpha() or len(d.name.split("-")) < 4:
            continue
        tag = d.name
        transport, algorithm, direction, if_type = parse_tag(tag)
        raw_path = d / "raw.csv"
        if not raw_path.exists():
            continue
        df = pd.read_csv(raw_path)
        df["transport"] = transport
        df["algorithm"] = algorithm
        df["direction"] = direction
        df["if_type"] = if_type
        df["if_label"] = IF_LABELS.get(if_type, if_type)
        rows.append(df)
    if not rows:
        raise SystemExit(f"No raw.csv found in {results_dir}")
    return pd.concat(rows, ignore_index=True)


def load_ho_intervals(results_dir: Path):
    """Load inter-handover intervals from per-seed CSVs."""
    records = []
    for d in sorted(results_dir.iterdir()):
        if not d.is_dir() or not d.name[0].isalpha() or len(d.name.split("-")) < 4:
            continue
        transport, algorithm, direction, if_type = parse_tag(d.name)
        for sd in sorted(d.iterdir()):
            if not sd.name.startswith("seed_"):
                continue
            p = sd / "nr-rl-handovers.csv"
            if not p.exists():
                continue
            times = pd.read_csv(p, skiprows=1, header=None, names=["time", "cellId"])
            if len(times) < 2:
                continue
            for iv in np.diff(times["time"].to_numpy()):
                records.append({"transport": transport, "algorithm": algorithm,
                                "direction": direction, "if_type": if_type,
                                "interval_s": iv})
    return pd.DataFrame(records)


# ── Helpers ──────────────────────────────────────────────────────────────

def make_positions_labels(if_order, directions, transports):
    """Build x-axis positions and labels for grouped bars/boxes.

    Returns (positions, labels, colors).
    """
    positions = []
    labels = []
    colors = []
    n_bars = len(directions) * len(transports)
    group_width = n_bars
    for gi, if_type in enumerate(if_order):
        base = gi * (group_width + 1)
        for di, direction in enumerate(directions):
            for ti, transport in enumerate(transports):
                pos = base + di * len(transports) + ti
                positions.append(pos)
                labels.append(f"{direction.upper()}\n{transport.upper()}\n{IF_LABELS[if_type]}")
                colors.append(DIR_COLORS[direction])
    return positions, labels, colors


def make_box_data(raw, metric, if_order, directions, transports):
    """Build list of arrays for boxplot, filtering out empty groups."""
    data = []
    valid_positions = []
    valid_labels = []
    valid_colors = []
    idx = 0
    for gi, if_type in enumerate(if_order):
        for di, direction in enumerate(directions):
            for ti, transport in enumerate(transports):
                vals = raw[(raw["direction"] == direction)
                           & (raw["if_type"] == if_type)
                           & (raw["transport"] == transport)][metric].dropna()
                if len(vals) == 0:
                    idx += 1
                    continue
                data.append(vals.values)
                valid_positions.append(idx)
                valid_labels.append(
                    f"{direction.upper()}\n{transport.upper()}\n{IF_LABELS[if_type]}")
                valid_colors.append(DIR_COLORS[direction])
                idx += 1
    return data, valid_positions, valid_labels, valid_colors


# ── Plots ────────────────────────────────────────────────────────────────

def plot_goodput_box(raw, out_dir):
    """Box plot: goodput per scenario, faceted by transport protocol."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
    for ax_idx, transport in enumerate(["tcp", "udp"]):
        ax = axes[ax_idx]
        sub = raw[raw["transport"] == transport]
        data, positions, labels, colors = make_box_data(
            sub, "goodputMbps", IF_ORDER, ["ul", "dl"], [transport])

        if not data:
            ax.text(0.5, 0.5, f"No {transport.upper()} data", ha="center",
                    va="center", transform=ax.transAxes)
            continue

        bp = ax.boxplot(data, positions=positions, widths=0.6, patch_artist=True,
                        medianprops=dict(color="black", linewidth=1.2),
                        flierprops=dict(marker=".", markersize=3, alpha=0.4))
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax.set_xticks(positions)
        ax.set_xticklabels(labels, fontsize=7, rotation=20, ha="right")
        ax.set_ylabel("Goodput (Mbps)")
        ax.set_title(f"{transport.upper()} Flow")
        ax.grid(axis="y", alpha=0.3)

    fig.suptitle("Goodput Distribution by Transport Protocol", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "01-goodput-box.png")
    plt.close(fig)
    print("  -> 01-goodput-box.png")


def plot_goodput_cdf(raw, out_dir):
    """CDF of goodput, one line per (transport, direction, if_type)."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    ls_map = {"no-if": "-", "aerial-if": "--", "ground-if": ":", "mixed-if": "-."}

    for ax_idx, transport in enumerate(["tcp", "udp"]):
        ax = axes[ax_idx]
        sub = raw[raw["transport"] == transport]
        for direction in ["ul", "dl"]:
            for if_type in IF_ORDER:
                vals = sorted(sub[(sub["direction"] == direction)
                                  & (sub["if_type"] == if_type)]["goodputMbps"].dropna())
                if len(vals) < 2:
                    continue
                y = np.linspace(0, 1, len(vals))
                ax.plot(vals, y, color=DIR_COLORS[direction],
                        linestyle=ls_map[if_type], linewidth=1.2,
                        label=f"{direction.upper()} {IF_LABELS[if_type]}")
        ax.set_xlabel("Goodput (Mbps)")
        ax.set_title(f"{transport.upper()} CDF")
        ax.legend(fontsize=7, loc="lower right")
        ax.grid(alpha=0.3)

    axes[0].set_ylabel("CDF")
    fig.suptitle("Goodput CDF by Transport Protocol")
    fig.tight_layout()
    fig.savefig(out_dir / "02-goodput-cdf.png")
    plt.close(fig)
    print("  -> 02-goodput-cdf.png")


def plot_observability_gap(raw, out_dir):
    """Side-by-side: Goodput (left) vs RSRP (right), faceted by transport."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for row, transport in enumerate(["tcp", "udp"]):
        sub = raw[raw["transport"] == transport]
        data_g, pos_g, lbl_g, col_g = make_box_data(
            sub, "goodputMbps", IF_ORDER, ["ul", "dl"], [transport])
        data_r, pos_r, lbl_r, col_r = make_box_data(
            sub, "rsrpServingDbm_avg", IF_ORDER, ["ul", "dl"], [transport])

        if not data_g:
            axes[row, 0].text(0.5, 0.5, f"No {transport.upper()} data",
                              ha="center", va="center", transform=axes[row, 0].transAxes)
            continue

        for col_idx, (ax, data, ylabel, ylim) in enumerate([
            (axes[row, 0], data_g, "Goodput (Mbps)", (0, 30)),
            (axes[row, 1], data_r, "RSRP (dBm)", (-100, -60)),
        ]):
            bp = ax.boxplot(data, positions=pos_g, widths=0.6, patch_artist=True,
                            medianprops=dict(color="black", linewidth=1.2),
                            flierprops=dict(marker=".", markersize=3, alpha=0.4))
            for patch, color in zip(bp["boxes"], col_g):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            ax.set_xticks(pos_g)
            ax.set_xticklabels(lbl_g, fontsize=6, rotation=25, ha="right")
            ax.set_ylabel(ylabel)
            ax.set_ylim(ylim)
            ax.grid(axis="y", alpha=0.3)

        axes[row, 0].set_title(f"{transport.upper()} — Goodput", fontsize=11)
        axes[row, 1].set_title(f"{transport.upper()} — RSRP", fontsize=11)

    fig.suptitle("Observability Gap: Same RSRP, Different Goodput", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "03-observability-gap.png")
    plt.close(fig)
    print("  -> 03-observability-gap.png")


def plot_rtt_facet(raw, out_dir):
    """RTT: TCP only (UDP has no RTT). UL liner, DL log scale."""
    tcp = raw[raw["transport"] == "tcp"]
    if tcp.empty:
        print("  SKIP rtt-facet: no TCP data")
        return

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.5),
                             gridspec_kw={"width_ratios": [1, 1.5]})

    # UL panel (linear)
    ax = axes[0]
    data_ul = []
    for if_type in IF_ORDER:
        vals = tcp[(tcp["direction"] == "ul")
                   & (tcp["if_type"] == if_type)]["rttMs_p50"].dropna()
        data_ul.append(vals.values if len(vals) > 0 else [np.nan])
    bp = ax.boxplot(data_ul, positions=range(len(IF_ORDER)), widths=0.5,
                    patch_artist=True,
                    boxprops=dict(facecolor=DIR_COLORS["ul"], alpha=0.6),
                    medianprops=dict(color="black"),
                    flierprops=dict(marker=".", markersize=3, alpha=0.3))
    ax.set_xticks(range(len(IF_ORDER)))
    ax.set_xticklabels([IF_LABELS[k] for k in IF_ORDER], fontsize=8)
    ax.set_ylabel("RTT (ms)")
    ax.set_title("UL Flow (linear)")
    ax.set_ylim(0, 120)
    ax.grid(axis="y", alpha=0.3)

    # DL panel (log)
    ax = axes[1]
    data_dl = []
    for if_type in IF_ORDER:
        vals = tcp[(tcp["direction"] == "dl")
                   & (tcp["if_type"] == if_type)]["rttMs_p50"].dropna()
        data_dl.append(vals.values if len(vals) > 0 else [1])
    bp = ax.boxplot(data_dl, positions=range(len(IF_ORDER)), widths=0.5,
                    patch_artist=True,
                    boxprops=dict(facecolor=DIR_COLORS["dl"], alpha=0.6),
                    medianprops=dict(color="black"),
                    flierprops=dict(marker=".", markersize=3, alpha=0.3))
    ax.set_xticks(range(len(IF_ORDER)))
    ax.set_xticklabels([IF_LABELS[k] for k in IF_ORDER], fontsize=8)
    ax.set_ylabel("RTT (ms)")
    ax.set_title("DL Flow (log scale)")
    ax.set_yscale("log")
    ax.set_ylim(10, 10000)
    ax.grid(axis="y", alpha=0.3)
    ax.axhline(y=55, color="gray", linestyle="--", linewidth=0.8, alpha=0.5)
    ax.annotate("UL baseline ~55ms", xy=(3.5, 60), fontsize=7, color="gray")

    fig.suptitle("Median RTT Distribution (TCP only)")
    fig.tight_layout()
    fig.savefig(out_dir / "04-rtt-facet.png")
    plt.close(fig)
    print("  -> 04-rtt-facet.png")


def plot_ho_rate_box(raw, out_dir):
    """Handover rate per minute, faceted by transport."""
    raw = raw.copy()
    raw["ho_rate"] = raw["handovers"] / (raw["simTime"] / 60.0)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
    for ax_idx, transport in enumerate(["tcp", "udp"]):
        ax = axes[ax_idx]
        sub = raw[raw["transport"] == transport]
        data, positions, labels, colors = make_box_data(
            sub, "ho_rate", IF_ORDER, ["ul", "dl"], [transport])

        if not data:
            ax.text(0.5, 0.5, f"No {transport.upper()} data", ha="center",
                    va="center", transform=ax.transAxes)
            continue

        bp = ax.boxplot(data, positions=positions, widths=0.6, patch_artist=True,
                        medianprops=dict(color="black", linewidth=1.2),
                        flierprops=dict(marker=".", markersize=3, alpha=0.4))
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

        ax.set_xticks(positions)
        ax.set_xticklabels(labels, fontsize=7, rotation=20, ha="right")
        ax.set_ylabel("Handovers per Minute")
        ax.set_title(f"{transport.upper()} Rate")
        ax.grid(axis="y", alpha=0.3)

    fig.suptitle("Handover Rate by Transport Protocol")
    fig.tight_layout()
    fig.savefig(out_dir / "05-ho-rate-box.png")
    plt.close(fig)
    print("  -> 05-ho-rate-box.png")


def plot_ho_binned_goodput(raw, out_dir):
    """Bin seeds by handover count, show goodput. TCP only (UDP not affected by HO)."""
    tcp = raw[raw["transport"] == "tcp"].copy()
    if tcp.empty:
        print("  SKIP ho-binned-goodput: no TCP data")
        return
    bins = [0, 1, 4, 7, 100]
    labels = ["0", "1-3", "4-6", "7+"]
    tcp["ho_bin"] = pd.cut(tcp["handovers"], bins=bins, labels=labels, right=False)

    fig, axes = plt.subplots(1, 2, figsize=(9, 4), sharey=True)
    for idx, direction in enumerate(["ul", "dl"]):
        ax = axes[idx]
        sub = tcp[tcp["direction"] == direction]
        groups = [sub[sub["ho_bin"] == lbl]["goodputMbps"].dropna() for lbl in labels]
        bp = ax.boxplot(groups, positions=range(len(labels)), widths=0.5,
                        patch_artist=True,
                        boxprops=dict(facecolor=DIR_COLORS[direction], alpha=0.6),
                        medianprops=dict(color="black"),
                        flierprops=dict(marker=".", markersize=3, alpha=0.3))
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, fontsize=9)
        ax.set_xlabel("Handover Count")
        ax.set_title(f"{direction.upper()} Flow (TCP)")
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("Goodput (Mbps)")
    fig.suptitle("Goodput by Handover Activity (TCP only)")
    fig.tight_layout()
    fig.savefig(out_dir / "06-ho-binned-goodput.png")
    plt.close(fig)
    print("  -> 06-ho-binned-goodput.png")


def plot_ho_interval_hist(ho_intervals, out_dir):
    """Histogram of inter-handover intervals, UL vs DL."""
    if ho_intervals.empty:
        print("  SKIP ho-interval-hist: no data")
        return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for idx, direction in enumerate(["ul", "dl"]):
        ax = axes[idx]
        intervals = ho_intervals[ho_intervals["direction"] == direction]["interval_s"]
        if len(intervals) == 0:
            ax.text(0.5, 0.5, "No data", ha="center", va="center",
                    transform=ax.transAxes)
            continue
        ax.hist(intervals, bins=30, color=DIR_COLORS[direction],
                edgecolor="white", linewidth=0.3, alpha=0.8)
        median_iv = np.median(intervals)
        ax.axvline(median_iv, color="black", linestyle="--", linewidth=0.8)
        ax.annotate(f"median={median_iv:.1f}s", xy=(median_iv, ax.get_ylim()[1] * 0.9),
                    fontsize=8)
        ax.set_xlabel("Inter-Handover Interval (s)")
        ax.set_title(f"{direction.upper()} Flow")
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("Count")
    fig.suptitle("Inter-Handover Time Distribution")
    fig.tight_layout()
    fig.savefig(out_dir / "07-ho-interval-hist.png")
    plt.close(fig)
    print("  -> 07-ho-interval-hist.png")


def plot_goodput_udp_vs_tcp(raw, out_dir):
    """Direct TCP vs UDP goodput comparison bar chart."""
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax_idx, direction in enumerate(["ul", "dl"]):
        ax = axes[ax_idx]
        x = np.arange(len(IF_ORDER))
        width = 0.35
        for tp_idx, transport in enumerate(["tcp", "udp"]):
            medians = []
            err_low = []
            err_high = []
            for if_type in IF_ORDER:
                vals = raw[(raw["direction"] == direction)
                           & (raw["if_type"] == if_type)
                           & (raw["transport"] == transport)]["goodputMbps"].dropna()
                if len(vals) == 0:
                    medians.append(0); err_low.append(0); err_high.append(0)
                else:
                    svals = sorted(vals)
                    medians.append(np.median(svals))
                    err_low.append(np.median(svals) - np.percentile(svals, 5))
                    err_high.append(np.percentile(svals, 95) - np.median(svals))
            offset = (tp_idx - 0.5) * width
            ax.bar(x + offset, medians, width, label=transport.upper(),
                   color=TP_COLORS[transport], yerr=[err_low, err_high],
                   capsize=3, edgecolor="white", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels([IF_LABELS[k] for k in IF_ORDER])
        ax.set_title(f"{direction.upper()} Flow")
        ax.legend()
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("Goodput (Mbps)")
    fig.suptitle("UDP vs TCP Goodput by Interference Type")
    fig.tight_layout()
    fig.savefig(out_dir / "08-udp-vs-tcp.png")
    plt.close(fig)
    print("  -> 08-udp-vs-tcp.png")


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Generate thesis-quality plots from evaluation output")
    parser.add_argument("results_dir", type=str)
    parser.add_argument("--output", "-o", type=str, default="figures")
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    if not results_dir.exists():
        raise SystemExit(f"Not found: {results_dir}")

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Loading per-seed data ...")
    raw = load_data(results_dir)
    print(f"  {len(raw)} seeds, transports={list(raw['transport'].unique())}, "
          f"IF types={list(raw['if_type'].unique())}, "
          f"directions={list(raw['direction'].unique())}")

    print("Loading handover intervals ...")
    ho_intervals = load_ho_intervals(results_dir)
    print(f"  {len(ho_intervals)} intervals")

    print("Generating plots ...")
    plot_goodput_box(raw, out_dir)
    plot_goodput_cdf(raw, out_dir)
    plot_observability_gap(raw, out_dir)
    plot_rtt_facet(raw, out_dir)
    plot_ho_rate_box(raw, out_dir)
    plot_ho_binned_goodput(raw, out_dir)
    plot_ho_interval_hist(ho_intervals, out_dir)
    plot_goodput_udp_vs_tcp(raw, out_dir)

    print(f"\nDone. Figures saved to {out_dir.resolve()}")


if __name__ == "__main__":
    main()
