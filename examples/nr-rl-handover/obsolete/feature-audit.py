#!/usr/bin/env python3
"""A-priori feature audit for the NR-RL handover observation space.

Screens the observation features against the environment's reward/throughput
statistics BEFORE training, so feature choices are evidence-based instead of
guessed. Runs on logged episodes (rl_obs.csv + rl_reward.csv) — use data from a
RANDOM or untrained policy for unbiased results (`run-agent random ...` with
logging=true); the tool works on any logged episode, but policy-generated data
confounds the statistics.

Metrics per feature (Spearman rank correlation — the Deng mapping is nonlinear):
  1. Informativeness : rho(feature, reward) and rho(feature, goodput)
                       (goodput isolates the throughput signal from the HO penalty)
  2. Deadness        : std, fraction of unique values, % time at a declared bound
  3. Redundancy      : pairwise correlation matrix -> top correlated pairs
  4. Leading         : rho(feature(t), goodput(t+k)) for k=1..lag_max steps —
                       validates the 1s time-delta block (do trends lead throughput?)
  5. Handover trigger: feature mean in the 1s window BEFORE each handover vs the
                       global baseline (which features mark an imminent handover?)

Outputs (in --out-dir):
  feature-audit.csv : per-feature stats + verdict (keep/drop/watch)
  redundancy.csv    : top correlated feature pairs
  feature-audit.png : |rho| vs reward bar chart
  redundancy.png    : feature x feature Spearman heatmap

Usage:
  python3 feature-audit.py --obs-file output/rl_obs.csv \
      --reward-file output/rl_reward.csv --tag random
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Single source of truth for the obs layout (same as feature-importance.py).
# The file is hyphenated (feature-importance.py) so it can't be imported by
# name — load it by path via importlib.
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "feature_importance_mod",
    str(Path(__file__).parent / "feature-importance.py"))
_fi_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_fi_mod)
FEATURE_NAMES = _fi_mod.FEATURE_NAMES

N_FEAT = len(FEATURE_NAMES)

# Declared obs-space bounds (must match nr-rl-handover-agent-app.cc).
# UL block appended: ul_sinr [-40, 50], ul_rb_util [0, 1], ul_sched_ue [0, 1];
# trajectory block: pos/next normalized [0, 3.0], TTA/12 [0, 1.0].
LOW = np.array([-160.0, -100.0] + [-160.0] * 5 + [-60.0] * 5
               + [-20.0, -20.0, -20.0, -2.0, -20.0] + [-20.0] * 5
               + [-40.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0]
               + [-40.0, 0.0, 0.0]
               + [0.0] * 7)
HIGH = np.array([-40.0, -3.0] + [-40.0] * 5 + [60.0] * 5
                + [20.0, 20.0, 20.0, 2.0, 20.0] + [20.0] * 5
                + [50.0, 1.0, 1.0, 1.0, 100000.0, 10.0, 2.0]
                + [50.0, 1.0, 1.0]
                + [3.0] * 6 + [1.0])

REWARD_COLS = ["t", "goodput", "ref", "min", "normG_raw", "normG", "R_G",
               "I_ho", "R_H", "pingPong", "reward"]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--obs-file", required=True, help="rl_obs.csv (time + 30..37 obs)")
    p.add_argument("--reward-file", required=True, help="rl_reward.csv")
    p.add_argument("--out-dir", default=None,
                   help="output dir (default: obs-file's parent)")
    p.add_argument("--min-time", type=float, default=1.0,
                   help="skip steps before this sim time (default 1.0 = blind zone)")
    p.add_argument("--max-time", type=float, default=None)
    p.add_argument("--tag", default="", help="label for the report header")
    p.add_argument("--lag-max", type=int, default=5,
                   help="max lead steps for the lag analysis (1 step = 0.2s)")
    p.add_argument("--redundancy-threshold", type=float, default=0.9,
                   help="|rho| above this marks a redundant pair")
    return p.parse_args()


def spearman(a: np.ndarray, b: np.ndarray) -> float:
    """Rank correlation without scipy: Pearson on ranks (NaN for constant input)."""
    with np.errstate(invalid="ignore", divide="ignore"):
        ra = pd.Series(a).rank().to_numpy()
        rb = pd.Series(b).rank().to_numpy()
        c = np.corrcoef(ra, rb)[0, 1]
    return float(c) if c == c else np.nan


def load_aligned(args: argparse.Namespace):
    obs = pd.read_csv(args.obs_file, )
    rwd = pd.read_csv(args.reward_file, skiprows=1, header=None, names=REWARD_COLS)
    if obs.shape[1] != N_FEAT + 1:
        raise ValueError(f"obs file has {obs.shape[1]} cols (expect {N_FEAT + 1}); "
                         "is this a pre-2026-08-05 layout file?")
    obs.columns = ["t"] + FEATURE_NAMES
    # Both apps log on the 0.2s grid independently — merge on rounded time.
    obs["t"] = obs["t"].round(2)
    rwd["t"] = rwd["t"].round(2)
    df = obs.merge(rwd[["t", "goodput", "normG", "I_ho", "reward"]],
                   on="t", how="inner")
    t = df["t"].to_numpy()
    mask = t >= (args.min_time if args.min_time is not None else -np.inf)
    if args.max_time is not None:
        mask &= t <= args.max_time
    df = df[mask].reset_index(drop=True)
    if len(df) < 20:
        print(f"[WARN] only {len(df)} aligned rows after filtering")
    return df


def main() -> int:
    args = parse_args()
    df = load_aligned(args)
    feats = df[FEATURE_NAMES].to_numpy(dtype=float)
    reward = df["reward"].to_numpy(dtype=float)
    goodput = df["goodput"].to_numpy(dtype=float)
    iho = df["I_ho"].to_numpy(dtype=float)
    n = len(df)

    print(f"=== Feature audit {'(' + args.tag + ')' if args.tag else ''} ===")
    print(f"rows (aligned, t>={args.min_time}): {n}")
    print(f"handover (I_ho=1) steps: {int(iho.sum())} / {n} "
          f"({100 * iho.mean():.0f}%)")

    # ---- 1/2. Informativeness + deadness ---------------------------------
    rows = []
    for j, name in enumerate(FEATURE_NAMES):
        v = feats[:, j]
        std = float(v.std())
        n_unique = len(np.unique(v))
        frac_clamped = float(
            np.mean((v <= LOW[j] + 1e-6) | (v >= HIGH[j] - 1e-6)))
        rows.append({
            "feature": name,
            "mean": float(v.mean()),
            "std": std,
            "unique_frac": n_unique / n,
            "clamped_frac": frac_clamped,
            "rho_reward": spearman(v, reward),
            "rho_goodput": spearman(v, goodput),
            "rho_iho": spearman(v, iho),
        })
    stats = pd.DataFrame(rows)

    # ---- 3. Redundancy ----------------------------------------------------
    corr = pd.DataFrame(np.full((N_FEAT, N_FEAT), np.nan),
                        index=FEATURE_NAMES, columns=FEATURE_NAMES)
    for i in range(N_FEAT):
        for j in range(i + 1, N_FEAT):
            r = spearman(feats[:, i], feats[:, j])
            corr.iloc[i, j] = corr.iloc[j, i] = r
    pairs = []
    for i in range(N_FEAT):
        for j in range(i + 1, N_FEAT):
            r = corr.iloc[i, j]
            if abs(r) >= args.redundancy_threshold:
                pairs.append((FEATURE_NAMES[i], FEATURE_NAMES[j], r))
    pairs.sort(key=lambda x: -abs(x[2]))
    max_abs_corr = corr.abs().max(axis=1)
    stats["max_abs_corr"] = [max_abs_corr[name] for name in FEATURE_NAMES]

    # ---- 4. Leading-indicator analysis ------------------------------------
    lag_rows = []
    if args.lag_max > 0 and n > args.lag_max + 5:
        for j, name in enumerate(FEATURE_NAMES):
            row = {"feature": name}
            for k in range(1, args.lag_max + 1):
                row[f"lead{k}"] = spearman(feats[:-k, j], goodput[k:])
            lag_rows.append(row)
        lag = pd.DataFrame(lag_rows)

    # ---- 5. Handover-trigger discrimination --------------------------------
    trig = None
    ho_starts = np.where((iho[1:] == 1) & (iho[:-1] == 0))[0] + 1
    if len(ho_starts) > 0:
        # 1s window before each handover = the 5 steps ending at the event.
        pre = np.zeros((len(ho_starts), N_FEAT))
        k = 0
        for e in ho_starts:
            if e >= 6:
                pre[k] = feats[e - 5:e].mean(axis=0)
            k += 1
        pre = pre[:k]
        calm = iho < 0.5  # global baseline over non-hangover steps
        with np.errstate(invalid="ignore", divide="ignore"):
            calm_std = feats[calm].std(axis=0)
            delta_std = (pre.mean(axis=0) - feats[calm].mean(axis=0)) / calm_std
            delta_std[~np.isfinite(delta_std)] = np.nan
        trig = pd.DataFrame({
            "feature": FEATURE_NAMES,
            "mean_preHO": pre.mean(axis=0) if len(pre) else np.zeros(N_FEAT),
            "mean_calm": feats[calm].mean(axis=0),
            "delta_std": delta_std,
        })
        if calm.sum() < 5:
            print("[WARN] too few non-hangover steps for a calm baseline; "
                  "trigger analysis unreliable")

    # ---- Verdicts ----------------------------------------------------------
    def verdict(r):
        if r["std"] < 1e-6 or r["unique_frac"] < 0.02:
            return "drop: constant"
        if abs(r["rho_reward"]) >= 0.1 or abs(r["rho_goodput"]) >= 0.1:
            return "keep"
        if r["max_abs_corr"] >= args.redundancy_threshold:
            return "watch: redundant"
        return "watch: weak"

    stats["verdict"] = stats.apply(verdict, axis=1)

    # ---- Outputs -----------------------------------------------------------
    out = Path(args.out_dir) if args.out_dir else Path(args.obs_file).parent
    out.mkdir(parents=True, exist_ok=True)

    stats = stats.sort_values("rho_reward", key=lambda s: s.abs(), ascending=False)
    print("\n" + stats[["feature", "std", "unique_frac", "clamped_frac",
                        "rho_reward", "rho_goodput", "rho_iho", "verdict"]]
          .to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    print("\nTop redundant pairs (|rho| >= "
          f"{args.redundancy_threshold}):")
    print("  " + ("\n  ".join(f"{a} ~ {b} ({r:+.2f})" for a, b, r in pairs[:8])
                  if pairs else "  none"))

    if args.lag_max > 0 and n > args.lag_max + 5:
        top5 = stats.head(5)["feature"].tolist()
        print("\nLeading indicators  rho(feature(t), goodput(t+k)) for top-5:")
        print(lag[lag["feature"].isin(top5)].set_index("feature")
              .round(3).to_string())

    if trig is not None:
        top3 = trig.reindex(trig["delta_std"].abs().sort_values(ascending=False).index)
        print("\nHandover-trigger discrimination (top 5, in std units):")
        print(top3.head(5)[["feature", "mean_preHO", "mean_calm", "delta_std"]]
              .to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    stats.to_csv(out / "feature-audit.csv", index=False)
    pd.DataFrame(pairs, columns=["a", "b", "rho"]).to_csv(
        out / "redundancy.csv", index=False)
    if args.lag_max > 0 and n > args.lag_max + 5:
        lag.to_csv(out / "feature-lag.csv", index=False)
    print(f"\nWrote {out / 'feature-audit.csv'} (+ redundancy.csv, feature-lag.csv)")

    # ---- Plots -------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 6))
        order = np.argsort(np.abs(stats["rho_reward"].to_numpy()))[::-1]
        names = [stats["feature"].iloc[i] for i in order]
        vals = stats["rho_reward"].to_numpy()[order]
        colors = ["tab:green" if "keep" in stats["verdict"].iloc[i]
                  else "tab:orange" if "watch" in stats["verdict"].iloc[i]
                  else "tab:red" for i in order]
        ax.barh(range(len(vals)), vals, color=colors, tick_label=names,
                edgecolor="black", linewidth=0.5)
        ax.invert_yaxis()
        ax.set_xlabel("Spearman rho(feature, reward)")
        ax.set_title(f"Feature audit — rho vs reward "
                     f"{'(' + args.tag + ')' if args.tag else ''}")
        ax.axvline(0.1, color="gray", ls="--", lw=0.8)
        ax.axvline(-0.1, color="gray", ls="--", lw=0.8)
        ax.grid(axis="x", alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / "feature-audit.png", dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(11, 9))
        im = ax.imshow(corr.to_numpy(), cmap="RdBu_r", vmin=-1, vmax=1)
        ax.set_xticks(range(N_FEAT), FEATURE_NAMES, rotation=90, fontsize=6)
        ax.set_yticks(range(N_FEAT), FEATURE_NAMES, fontsize=6)
        ax.figure.colorbar(im, ax=ax, shrink=0.7)
        ax.set_title("Feature x feature Spearman correlation")
        fig.tight_layout()
        fig.savefig(out / "redundancy.png", dpi=150)
        plt.close(fig)
        print(f"Wrote {out / 'feature-audit.png'}, {out / 'redundancy.png'}")
    except Exception as e:  # plotting is optional
        print(f"[WARN] plotting failed: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
