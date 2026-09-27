#!/usr/bin/env python3
"""Aggregate per-seed feature importance into the thesis block-share summary.

Runs feature-importance.py per seed, normalises to shares, means across seeds
and writes feature-importance.csv, feature-importance-summary.md and
feature-importance-avg.png into the cell dir.

Usage: python3 analysis/feature-importance-summary.py --checkpoint <run> --cell-dir <cell> [--seeds 20]
"""

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

BLOCKS = {
    "Serving quality": ["serving_rsrp", "serving_rsrq"],
    "Ranked cells": ["slot_rsrp_0", "slot_rsrp_1", "slot_rsrp_2",
                     "rsrp_delta_0", "rsrp_delta_1", "rsrp_delta_2"],
    "Serving metrics": ["dl_sinr", "time_since_ho", "norm_goodput",
                        "ho_count_10s"],
    "Uplink": ["ul_sinr", "ul_rb_util", "ul_sched_ue"],
    "Deltas": ["d_serving_rsrp", "d_serving_sinr", "d_serving_rsrq",
               "d_norm_goodput", "d_margin_best",
               "d_slot_rsrp_0", "d_slot_rsrp_1", "d_slot_rsrp_2"],
    "Heading": ["heading_x", "heading_y", "heading_z"],
    "Position": ["pos_x", "pos_y", "pos_z"],
    "Lookahead": ["pos2s_x", "pos2s_y", "pos2s_z"],
}


def block_of(name):
    """Map a (possibly stacked) feature name to its block and frame age.
    Stacked names are <feature>_t (current) and <feature>_t-1 (previous)."""
    for block, members in BLOCKS.items():
        for m in members:
            if name == f"{m}_t":
                return block, "current frame _t"
            if name.startswith(f"{m}_t-"):
                return block, "previous frame _t-1"
    return "?", "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--cell-dir", required=True,
                    help="results/<eval>/<tag> holding seed_* dirs")
    ap.add_argument("--note", default="")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--limit", type=float, default=1.0)
    ap.add_argument("--python", default=sys.executable)
    args = ap.parse_args()

    script = Path(__file__).with_name("feature-importance.py")
    cell = Path(args.cell_dir)
    per_seed = []
    dropped = []

    for seed in range(1, args.seeds + 1):
        seed_dir = cell / f"seed_{seed}"
        if not (seed_dir / "rl_obs.csv").exists():
            print(f"seed {seed}: no rl_obs.csv, skipped")
            continue
        with tempfile.TemporaryDirectory() as tmp:
            cmd = [args.python, str(script),
                   "--checkpoint", args.checkpoint,
                   "--obs-file", str(seed_dir),
                   "--out-dir", tmp,
                   "--min-time", "1.0",
                   "--repeat", str(args.repeat),
                   "--limit", str(args.limit)]
            r = subprocess.run(cmd, capture_output=True, text=True)
            csv = Path(tmp) / "feature-importance.csv"
            if r.returncode != 0 or not csv.exists():
                print(f"seed {seed}: failed (rc={r.returncode}) "
                      f"{r.stderr.strip().splitlines()[-1] if r.stderr else ''}")
                continue
            df = pd.read_csv(csv)
        total = df["importance"].sum()
        if total <= 0:
            # Permuting any single feature never flips the argmax action in this
            # seed, so the per-seed share is undefined. Expected in the
            # sparse-action clean regime; record the seed so the reported seed
            # count is traceable.
            dropped.append(seed)
            print(f"seed {seed}: zero permutation importance, skipped")
            continue
        df["share"] = 100.0 * df["importance"] / total
        df["seed"] = seed
        per_seed.append(df)
        print(f"seed {seed}: ok ({len(df)} features)")

    if not per_seed:
        sys.exit("no seeds produced output")

    all_df = pd.concat(per_seed)
    mean = all_df.groupby("name")["share"].mean()
    std = all_df.groupby("name")["share"].std()
    # rank stability: std of the feature's rank within each seed
    ranks = all_df.pivot_table(index="seed", columns="name",
                               values="share").rank(axis=1, ascending=False)
    rank_std = ranks.std(axis=0)
    merged = pd.DataFrame({"mean_pct": mean, "std_pct": std,
                           "rank_std": rank_std}).sort_values(
        "mean_pct", ascending=False)
    merged.to_csv(cell / "feature-importance.csv")

    # block shares
    rows = []
    for name, r in merged.iterrows():
        block, frame = block_of(name)
        rows.append({"block": block, "frame": frame, "mean": r["mean_pct"]})
    bdf = pd.DataFrame(rows)
    by_block = bdf.groupby("block")["mean"].sum().sort_values(ascending=False)
    frame_split = bdf.pivot_table(index="block", columns="frame",
                                  values="mean", aggfunc="sum").fillna(0.0)

    head = f"""# Feature importance — {cell.parent.name}/{cell.name}

Permutation importance, mean over {len(per_seed)} seeds (checkpoint {Path(args.checkpoint).name}{', ' + args.note if args.note else ''}).
{('Seeds attempted: ' + str(args.seeds) + '; dropped (zero permutation importance): ' + ', '.join(str(s) for s in dropped) + '.' + chr(10)) if dropped else ''}Per-feature share of total permutation importance, normalized per seed; rank_std = std of the feature's rank across seeds (low = consistent). The `_t` frame is the current step, `_t-1` the previous stacked frame (obsStackFrames = 2).

## Feature-block shares

| block | total % | current frame _t % | previous frame _t-1 % |
|---|---|---|---|
"""
    for block in by_block.index:
        if block == "?":
            continue
        t = frame_split.loc[block].get("current frame _t", 0.0)
        p = frame_split.loc[block].get("previous frame _t-1", 0.0)
        head += f"| {block} | {by_block[block]:.2f} | {t:.2f} | {p:.2f} |\n"
    head += f"| **all** | **{by_block.sum():.2f}** | **{frame_split.sum().get('current frame _t', 0):.2f}** | **{frame_split.sum().get('previous frame _t-1', 0):.2f}** |\n"

    head += "\n## Per-feature shares\n\n| rank | feature | mean % | rank_std |\n|---|---|---|---|\n"
    for i, (name, r) in enumerate(merged.iterrows(), start=1):
        head += f"| {i} | {name} | {r['mean_pct']:.2f} | {r['rank_std']:.2f} |\n"

    (cell / "feature-importance-summary.md").write_text(head)
    print(f"wrote {cell / 'feature-importance-summary.md'}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    top = merged.head(20)
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.barh(range(len(top)), top["mean_pct"], xerr=top["std_pct"],
            color="tab:red", edgecolor="black", linewidth=0.5)
    ax.set_yticks(range(len(top)), top.index)
    ax.invert_yaxis()
    ax.set_xlabel("share of total permutation importance (%)")
    ax.set_title(f"Feature importance - {cell.parent.name} (mean of "
                 f"{len(per_seed)} seeds)")
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(cell / "feature-importance-avg.png", dpi=150)
    print(f"wrote {cell / 'feature-importance-avg.png'}")


if __name__ == "__main__":
    main()
