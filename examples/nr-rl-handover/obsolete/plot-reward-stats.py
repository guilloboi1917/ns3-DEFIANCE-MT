#!/usr/bin/env python3
"""Plot the reward structure of one evaluation episode from rl_reward.csv.

Shows the per-step reward over time (with hangover/ping-pong context), the
cumulative (partial) reward, the R_G / R_H decomposition, the per-step reward
distribution split by state, and a statistics panel. When the obs file has
the 30-dim layout, the windowed handover count (ho_count_10s) is overlaid on
the per-step panel.

rl_reward.csv columns (written by nr-rl-handover-rwd-app, no header):
    t, goodput(Mbps), ref, min, normG_raw, normG, R_G, I_ho, R_H, pingPong, reward

Usage:
    python3 plot-reward-stats.py -i <data_dir> [-o out.png] [--min-time 1.0] [--max-time 70.0]
"""

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REWARD_COLS = ["t", "goodput", "ref", "min", "normG_raw", "normG",
               "R_G", "I_ho", "R_H", "pingPong", "reward"]


def load_ho_count(data_dir, min_time, max_time):
    """Load the ho_count_10s obs dimension from rl_obs.csv (t + 30 obs cols).

    Returns (times, counts) aligned to the reward time grid, or None if the
    obs file is missing or predates the 30-dim layout (then it has no window
    count column).
    """
    obs_path = os.path.join(data_dir, "rl_obs.csv")
    if not os.path.exists(obs_path):
        return None
    obs = pd.read_csv(obs_path, )
    if obs.shape[1] < 31:  # t + obs[0..29]
        return None
    o = obs.iloc[:, [0, 30]].copy()
    o.columns = ["t", "ho_count"]
    if min_time is not None:
        o = o[o["t"] >= min_time]
    if max_time is not None:
        o = o[o["t"] <= max_time]
    return o


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-i", "--input_dir", type=str, default=None,
                   help="directory containing rl_reward.csv (default: nr-rl-handover/output)")
    p.add_argument("-o", "--output", type=str, default="reward-stats.png",
                   help="output png path (default: reward-stats.png)")
    p.add_argument("--min-time", type=float, default=None,
                   help="drop rows before this sim time (s)")
    p.add_argument("--max-time", type=float, default=None,
                   help="drop rows after this sim time (s)")
    args = p.parse_args()

    data_dir = args.input_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "output")
    csv_path = os.path.join(data_dir, "rl_reward.csv")
    if not os.path.exists(csv_path):
        print(f"[ERROR] {csv_path} not found")
        sys.exit(1)

    df = pd.read_csv(csv_path, skiprows=1, header=None, names=REWARD_COLS)
    if df.empty:
        print("[ERROR] rl_reward.csv is empty")
        sys.exit(1)
    if args.min_time is not None:
        df = df[df["t"] >= args.min_time]
    if args.max_time is not None:
        df = df[df["t"] <= args.max_time]
    df = df.reset_index(drop=True)

    t = df["t"].values
    reward = df["reward"].values
    rg = df["R_G"].values
    rh = df["R_H"].values
    iho = df["I_ho"].values
    pp = df["pingPong"].values
    gput = df["goodput"].values
    n = len(df)

    # State classification per step. NOTE: the pingPong column is the A->B->A
    # pattern FLAG — it can stay 1 on calm steps (pattern in history, but no
    # penalty because I_ho=0). The penalized buckets must key on I_ho==1.
    calm = iho == 0
    hangover = (iho == 1) & (pp == 0)
    pingpong = (iho == 1) & (pp == 1)  # ping-pong with hangover active -> penalty applied

    # Handover events = rising edges of I_ho (a handover fired that step).
    # NOTE: under sustained churn, handovers fired while still in hangover do
    # NOT produce a new rising edge, so this undercounts; nr-rl-handovers.csv
    # is the authoritative count when present.
    ho_events = np.where((iho == 1) & (np.concatenate([[0], iho[:-1]]) == 0))[0]
    pp_events = np.where(pp == 1)[0]

    # Authoritative handover count from the handover log, when available.
    ho_csv = os.path.join(data_dir, "nr-rl-handovers.csv")
    n_ho_csv = None
    if os.path.exists(ho_csv):
        try:
            ho_log = pd.read_csv(ho_csv, skiprows=1, header=None, names=["time", "cellId"])
            if args.min_time is not None:
                ho_log = ho_log[ho_log["time"] >= args.min_time]
            if args.max_time is not None:
                ho_log = ho_log[ho_log["time"] <= args.max_time]
            n_ho_csv = len(ho_log)
        except Exception:
            n_ho_csv = None

    # Windowed handover count (ho_count_10s, obs index 29) for the overlay.
    ho_count = load_ho_count(data_dir, args.min_time, args.max_time)
    if ho_count is not None:
        ho_count = ho_count.sort_values("t")

    cum = np.cumsum(reward)

    # ---- Statistics ----
    def stats(x):
        return (x.mean(), x.std(), np.median(x), x.min(), x.max(),
                np.percentile(x, 5), np.percentile(x, 95))

    r_mean, r_std, r_med, r_min, r_max, r_p5, r_p95 = stats(reward)
    total = reward.sum()
    g_mean = gput.mean()
    calm_frac = calm.mean()
    ho_frac = hangover.mean()
    pp_frac = pingpong.mean()
    pp_flag_frac = (pp == 1).mean()  # A->B->A pattern flag, any step
    n_ho = len(ho_events)
    n_pp_events = len(np.where(np.diff(np.concatenate([[0], pp.astype(int)])) > 0)[0])

    metrics = [
        ("Steps (in window)", f"{n}"),
        ("Sim time span", f"{t[0]:.1f} - {t[-1]:.1f} s"),
        ("Total reward", f"{total:.2f}"),
        ("Reward/step mean", f"{r_mean:.4f}  (std {r_std:.4f})"),
        ("Reward/step median", f"{r_med:.4f}"),
        ("Reward/step min / max", f"{r_min:.4f} / {r_max:.4f}"),
        ("Reward/step p5 / p95", f"{r_p5:.4f} / {r_p95:.4f}"),
        ("Handovers", f"{n_ho_csv if n_ho_csv is not None else n_ho}"
         + ("" if n_ho_csv is not None else " (I_ho edges; undercounts churn)")),
        ("Ping-pong events", f"{n_pp_events}"),
        ("Ping-pong flag steps", f"{int(pp.sum())} ({pp_flag_frac * 100:.0f}% of steps)"),
        ("Calm steps", f"{calm_frac * 100:.1f}%"),
        ("Hangover steps", f"{ho_frac * 100:.1f}%"),
        ("Mean goodput", f"{g_mean:.1f} Mbps"),
        ("Mean R_G", f"{rg.mean():.3f}"),
        ("Mean R_H", f"{rh.mean():.3f}"),
    ]
    state_reward = {
        "calm": reward[calm].mean() if calm.any() else float("nan"),
        "hangover": reward[hangover].mean() if hangover.any() else float("nan"),
        "ping-pong": reward[pingpong].mean() if pingpong.any() else float("nan"),
    }

    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    fig.suptitle(f"Reward structure — {os.path.basename(os.path.normpath(data_dir))} "
                 f"(total {total:.2f}, mean {r_mean:.4f}/step)", fontsize=13)

    # ── (0,0) per-step reward + cumulative ──
    ax = axes[0, 0]
    ax.fill_between(t, 0, iho, step="post", color="tab:red", alpha=0.12,
                    label="hangover (I_ho)")
    ax.scatter(t[pingpong], reward[pingpong], s=8, color="tab:orange", zorder=5,
               label="ping-pong step")
    ax.step(t, reward, where="post", color="tab:blue", lw=1.0, label="reward/step")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("reward/step")
    ax.set_title("Per-step reward + cumulative")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, cum, color="tab:green", lw=1.2, label="cumulative reward")
    ax2.set_ylabel("cumulative reward", color="tab:green")
    ax2.tick_params(axis="y", labelcolor="tab:green")
    if ho_count is not None:
        # Overlay the windowed handover count (obs index 29) on the per-step
        # panel as a generic handover-activity signal (the rate penalty was
        # removed; the obs feature is kept).
        merged = pd.merge_asof(df[["t"]], ho_count, on="t",
                               direction="nearest", tolerance=0.25)
        hc = merged["ho_count"].fillna(0).to_numpy()
        ax3 = ax.twinx()
        ax3.step(t, hc, where="post", color="tab:purple", lw=0.8, ls="--",
                 label="ho_count_10s")
        ax3.set_ylabel("handovers in window", color="tab:purple")
        ax3.tick_params(axis="y", labelcolor="tab:purple")
        l1, la1 = ax.get_legend_handles_labels()
        l3, la3 = ax3.get_legend_handles_labels()
        ax.legend(l1 + l3, la1 + la3, fontsize=8, loc="upper left")
    ax2.legend(fontsize=8, loc="upper right")

    # ── (0,1) decomposition: R_G, R_H ──
    ax = axes[0, 1]
    ax.step(t, rg, where="post", color="tab:purple", lw=1.0, label="R_G (goodput)")
    ax.step(t, rh, where="post", color="tab:red", lw=1.0, alpha=0.8, label="R_H (handover)")
    for e in ho_events:
        ax.axvline(t[e], color="gray", ls=":", lw=0.7, alpha=0.6)
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("reward term")
    ax.set_ylim(-0.05, 1.05)
    ax.set_title(f"Reward decomposition (handovers: {n_ho})")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.text(0.98, 0.02,
            f"calm  mean R = {state_reward['calm']:.3f}\n"
            f"hangover mean R = {state_reward['hangover']:.3f}\n"
            f"ping-pong mean R = {state_reward['ping-pong']:.3f}",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
            bbox=dict(boxstyle="round", fc="white", alpha=0.8))

    # ── (1,0) reward distribution by state ──
    ax = axes[1, 0]
    bins = np.linspace(min(reward.min(), -0.01), max(reward.max(), 1.01), 60)
    for mask, color, label in [(calm, "tab:green", "calm"),
                               (hangover, "tab:red", "hangover"),
                               (pingpong, "tab:orange", "ping-pong")]:
        ax.hist(reward[mask], bins=bins, alpha=0.5, color=color, label=label)
    ax.axvline(r_mean, color="k", ls="--", lw=1.2, label=f"mean {r_mean:.3f}")
    ax.axvline(r_med, color="k", ls=":", lw=1.2, label=f"median {r_med:.3f}")
    ax.set_xlabel("reward/step")
    ax.set_ylabel("step count")
    ax.set_title("Reward distribution by state")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── (1,1) statistics panel ──
    ax = axes[1, 1]
    ax.axis("off")
    lines = [f"{k:<22} {v}" for k, v in metrics]
    ax.text(0.02, 0.98, "\n".join(lines), transform=ax.transAxes,
            va="top", ha="left", fontsize=10, family="monospace",
            bbox=dict(boxstyle="round", fc="#f8f8f8", ec="gray"))
    ax.set_title("Statistics")

    fig.tight_layout()
    out_path = args.output
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved {out_path}")
    print("  " + "\n  ".join(f"{k}: {v}" for k, v in metrics))
    try:
        plt.show()
    except Exception:
        pass


if __name__ == "__main__":
    main()
