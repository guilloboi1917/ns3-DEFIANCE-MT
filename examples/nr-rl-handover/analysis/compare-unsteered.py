#!/usr/bin/env python3
"""compare-unsteered.py - how much do the thesis numbers move when the
opening-window beamforming confound is removed?

Compares, cell by cell and seed by seed (paired), the campaign dataset
(EvaluationData, gNB steered while the UE stays on its initially attached
cell) against the uniformly unsteered rerun (results-unsteered/, gNB
quasi-omni for the whole episode):

  1. level shift per cell        - campaign mean vs rerun mean per metric
  2. RL-vs-A3 paired deltas      - the claim-bearing comparisons, in both
                                   datasets, with paired p-values

Usage:
  python3 analysis/compare-unsteered.py [--campaign DIR] [--rerun DIR]
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy import stats

CAMPAIGN = Path("/home/nisaak/uzh/mt/Master-Thesis-Overleaf/EvaluationData")
RERUN = Path("results-unsteered")

# Metrics reported in the level-shift table (order matters for reading).
LEVEL_METRICS = [
    ("goodputMbps", "gp"),
    ("handovers", "HO"),
    ("e2eLossPct", "loss%"),
    ("retransmissions", "retx"),
    ("rttMs_p50", "rtt50"),
    ("ulSinrDb_avg", "ulSINR"),
    ("perTbSinrDb_avg", "dlSINR"),
    ("tbler", "TBLER"),
]
CLAIM_METRICS = ["handovers", "goodputMbps", "e2eLossPct", "retransmissions"]

# The A3 comparator of each RL cell, keyed by (transport, regime).
A3_COMPARATOR = {
    ("tcp", "no-if"): ("nr-rl-handover-a3-baseline-30mhz-tcp-ul", "tcp-ul-no-if"),
    ("tcp", "if"): ("nr-rl-handover-a3-baseline-30mhz-tcp-ul", "tcp-ul-if"),
    ("quic", "no-if"): ("nr-rl-handover-a3-baseline-30mhz-quic-ul", "quic-ul-no-if"),
    ("quic", "if"): ("nr-rl-handover-a3-baseline-30mhz-quic-ul", "quic-ul-if"),
}


def load(root: Path, evaluation: str, tag: str) -> dict:
    """Return {seed: {metric: value}} for one cell, or {} when absent."""
    p = root / evaluation / tag / "raw.csv"
    if not p.is_file():
        return {}
    out = {}
    with open(p) as f:
        for row in csv.DictReader(f):
            seed = int(float(row["seed"]))
            out[seed] = {k: (float(v) if v not in ("", "nan") else np.nan)
                         for k, v in row.items() if k != "seed"}
    return out


def is_rl(evaluation: str) -> bool:
    return evaluation.startswith("agent-eval")


def classify(evaluation: str, tag: str):
    """(transport, regime) for an RL cell, or None for A3 cells."""
    if not is_rl(evaluation):
        return None
    transport = "quic" if "quic" in tag else "tcp"
    regime = "no-if" if "no-if" in tag else "if"
    return transport, regime


def paired(a: dict, b: dict, metric: str):
    """(n, mean_a, mean_b, mean_delta, p_paired_t, p_wilcoxon) over shared seeds."""
    seeds = sorted(set(a) & set(b))
    x = np.array([a[s][metric] for s in seeds], dtype=float)
    y = np.array([b[s][metric] for s in seeds], dtype=float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    if len(x) < 3:
        return len(x), np.nan, np.nan, np.nan, np.nan, np.nan
    d = x - y
    pt = stats.ttest_rel(x, y).pvalue if np.any(d) else np.nan
    try:
        pw = stats.wilcoxon(x, y).pvalue if np.any(d) else np.nan
    except ValueError:
        pw = np.nan
    return len(x), x.mean(), y.mean(), d.mean(), pt, pw


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--campaign", type=Path, default=CAMPAIGN)
    ap.add_argument("--rerun", type=Path, default=RERUN)
    args = ap.parse_args()

    # ---- inventory of shared cells -------------------------------------
    cells = []
    for ev in sorted(p.name for p in args.campaign.iterdir() if p.is_dir()):
        for tag in sorted(p.name for p in (args.campaign / ev).iterdir() if p.is_dir()):
            if (args.rerun / ev / tag / "raw.csv").is_file():
                cells.append((ev, tag))

    print("=" * 100)
    print("1. LEVEL SHIFT PER CELL - campaign (steered opening window) vs rerun (uniformly unsteered)")
    print("   delta = campaign - rerun, paired on shared seeds; n = shared seeds")
    print("=" * 100)
    header = f"{'evaluation/tag':62s} {'n':>3s} " + " ".join(f"{lbl:>17s}" for _, lbl in LEVEL_METRICS)
    print(header)
    a3_shift = defaultdict(list)
    rl_shift = defaultdict(list)
    for ev, tag in cells:
        a = load(args.campaign, ev, tag)
        b = load(args.rerun, ev, tag)
        if not a or not b:
            continue
        row = f"{ev[:40] + '/' + tag:62s}"
        n_any = len(set(a) & set(b))
        row += f" {n_any:>3d} "
        for metric, _ in LEVEL_METRICS:
            n, ma, mb, md, p, _w = paired(a, b, metric)
            row += f" {md:>+9.2f}{'*' if (p == p and p < 0.05) else ' '}{'':>6s}"
            tgt = rl_shift if is_rl(ev) else a3_shift
            if md == md:
                tgt[metric].append(md)
        print(row)
    print("\n   mean shift over all cells (campaign - rerun):")
    for metric, lbl in LEVEL_METRICS:
        a = np.mean(a3_shift[metric]) if a3_shift[metric] else np.nan
        r = np.mean(rl_shift[metric]) if rl_shift[metric] else np.nan
        print(f"     {lbl:>8s}: A3 {a:>+8.2f} (n={len(a3_shift[metric])})   RL {r:>+8.2f} (n={len(rl_shift[metric])})")

    # ---- claim-bearing RL-vs-A3 paired deltas --------------------------
    print("\n" + "=" * 100)
    print("2. RL-vs-A3 PAIRED DELTAS (RL - A3), campaign vs rerun")
    print("   same seeds within each dataset; p = paired t-test")
    print("=" * 100)
    for ev, tag in cells:
        kind = classify(ev, tag)
        if kind is None:
            continue
        a3_ev, a3_tag = A3_COMPARATOR[kind]
        rl_c = load(args.campaign, ev, tag)
        a3_c = load(args.campaign, a3_ev, a3_tag)
        rl_u = load(args.rerun, ev, tag)
        a3_u = load(args.rerun, a3_ev, a3_tag)
        if not (rl_c and a3_c):
            continue
        print(f"\n  {ev} / {tag}   vs A3 {a3_tag}   (transport={kind[0]}, regime={kind[1]})")
        for metric in CLAIM_METRICS:
            nc, m1, m2, dc, pc, _ = paired(rl_c, a3_c, metric)
            nu, u1, u2, du, pu, _ = paired(rl_u, a3_u, metric)
            sig_c = "sig" if (pc == pc and pc < 0.05) else "n.s."
            sig_u = "sig" if (pu == pu and pu < 0.05) else "n.s."
            keep = "SAME SIGN" if (dc == dc and du == du and np.sign(dc) == np.sign(du)) else "SIGN FLIP"
            print(f"    {metric:16s} campaign n={nc:2d} {m1:>8.2f} vs {m2:>8.2f} -> {dc:>+7.2f} (p={pc:.4f}, {sig_c})"
                  f"   |  unsteered n={nu:2d} {u1:>8.2f} vs {u2:>8.2f} -> {du:>+7.2f} (p={pu:.4f}, {sig_u})  [{keep}]")


if __name__ == "__main__":
    main()
