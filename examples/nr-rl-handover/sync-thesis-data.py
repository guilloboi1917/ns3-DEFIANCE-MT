#!/usr/bin/env python3
"""sync-thesis-data.py - mirror thesis-grade CSVs into the EvaluationData bundle.

Copies aggregate.csv, raw.csv and params.yaml per cell (~360 KB) and writes
manifest.csv (cell -> narrative block, mode, seed count, regime). Skips
_archive/ and the diagnostic probes.

RL cells additionally get any derived feature-importance artifacts present in
the cell dir, plus an action-distribution.csv aggregated from the per-seed
rl_actions_full.csv (2026-09-23: the action-selectivity table needs the
per-action counts split by outcome, and the raw per-seed file is not part of
the reduced bundle, so the aggregate is derived here).

Several result trees can be merged into one bundle, in the order given, e.g.
    python3 sync-thesis-data.py --results results-unsteered results --allow-partial
bundles the uniformly unsteered dataset together with the steered beamforming
ablation (results/a3-bf-onoff). A cell (evaluation/tag) present in more than one
tree is taken from the first tree.

Usage: python3 sync-thesis-data.py [--dest DIR] [--results DIR [DIR ...]]
                                   [--allow-partial]
"""

import argparse
import csv
import shutil
import sys
from collections import Counter
from pathlib import Path

# Diagnostic probes kept in results/ for the KB but not part of the thesis
# bundle (no narrative block; few seeds, variant-specific seed sets).
EXCLUDE = {"delack-race", "buffer-2d-sweep", "rlc-buffer-sweep"}

BLOCKS = {
    "capacity-probe": "B0",
    "a3-bf-onoff": "B5",
    "transport-comparison-load-matched": "B1x",
    "transport-comparison": "B1",
    "tcp-variants": "B1",
    "a3-sweep": "B2",
    "udp-rlc-am": "B2",
    "agent-eval-": "B3",
    "nr-rl-handover-a3-baseline-30mhz-tcp-ul": "B3",
    "nr-rl-handover-a3-baseline-30mhz-quic-ul": "B3",
    "topology-hexgrid": "B3b",
    "mtx-ul": "B4",
    "a3-extremes-alt": "B4",
}

MANIFEST_KEYS = [
    "evaluation", "tag", "block", "mode", "n_seeds", "n_failed", "algorithm",
    "bandwidthMhz", "numerology", "transportProtocol", "trafficRateMbps",
    "ueSpeed", "startHeight", "endHeight", "addInterferingUes",
    "a3HysteresisDb", "a3TttMs", "tcpVariant", "checkpoint", "sim_time",
    "beamformingMethod",
]

NUMERIC = {"n_seeds", "bandwidthMhz", "numerology", "trafficRateMbps",
           "ueSpeed", "startHeight", "endHeight", "addInterferingUes",
           "a3HysteresisDb", "a3TttMs", "sim_time"}


def block_for(eval_name: str) -> str:
    """Narrative block for an evaluation dir. Longest matching prefix wins, so
    transport-comparison-load-matched (B1x, cross-check) does not fall through
    to the transport-comparison (B1) entry."""
    matches = [(p, b) for p, b in BLOCKS.items() if eval_name.startswith(p)]
    if not matches:
        return ""
    return max(matches, key=lambda pb: len(pb[0]))[1]


def load_params(p: Path) -> dict:
    try:
        with open(p) as f:
            txt = f.read()
        return dict(line.split(":", 1) for line in txt.splitlines()
                   if ":" in line and not line.lstrip().startswith("#"))
    except OSError:
        return {}


def write_action_distribution(tag: Path, dst_dir: Path) -> bool:
    """Aggregate seed_*/rl_actions_full.csv into action-distribution.csv.

    One row per (action index, outcome) pair with the decision count and its
    share. The executed rows sum to the episode handover count, which is the
    reconciliation the action-selectivity write-up performs.

    @param tag Cell dir holding seed_* subdirs.
    @param dst_dir Bundle dir for the cell.
    @return True if a file was written.
    """
    seed_files = sorted(tag.glob("seed_*/rl_actions_full.csv"))
    if not seed_files:
        return False
    counts = Counter()
    n_decisions = 0
    for f in seed_files:
        with f.open(newline="") as fh:
            for rec in csv.DictReader(fh):
                counts[(rec["actionIndex"], rec["outcome"])] += 1
                n_decisions += 1
    if not n_decisions:
        return False
    dst = dst_dir / "action-distribution.csv"
    dst.parent.mkdir(parents=True, exist_ok=True)
    with dst.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["n_seeds", "n_decisions", "action_index", "outcome",
                    "count", "share_pct"])
        for (idx, outcome), c in sorted(counts.items(),
                                        key=lambda kv: (int(kv[0][0]), kv[0][1])):
            w.writerow([len(seed_files), n_decisions, idx, outcome, c,
                        f"{100.0 * c / n_decisions:.4f}"])
    return True


def fmt_cell(v: str) -> str:
    """Extract the value after the first colon, strip comments/quotes."""
    v = v.strip()
    if "  #" in v:
        v = v.split("  #", 1)[0]
    return v.strip().strip('"').strip("'")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dest",
                    default="/home/nisaak/uzh/mt/Master-Thesis-Overleaf/EvaluationData")
    ap.add_argument("--results", nargs="+", default=["results"],
                    help="Source results tree(s) to sync, merged in the order given "
                         "(default: results/ in the current directory). Pass "
                         "'results-unsteered results' to bundle the uniformly unsteered "
                         "dataset together with the steered beamforming ablation. A cell "
                         "present in more than one tree is taken from the first tree.")
    ap.add_argument("--allow-partial", action="store_true",
                    help="Also sync cells that hold fewer valid seeds than their "
                         "params.yaml n_seeds (default: skip them, so a running "
                         "campaign cannot leak partial cells into the bundle)")
    args = ap.parse_args()
    roots = [Path(r).resolve() for r in args.results]
    dest = Path(args.dest)
    for root in roots:
        if not root.is_dir():
            sys.exit(f"no {root}/ dir here - run from the nr-rl-handover example dir")

    rows = []
    n_files = 0
    skipped = []
    dupes = []
    seen = set()
    candidates = []
    for root in roots:
        for ev in sorted(p for p in root.iterdir() if p.is_dir()
                         and not p.name.startswith("_") and p.name not in EXCLUDE):
            candidates.append((ev, sorted(p for p in ev.iterdir() if p.is_dir())))
    for ev, tags in candidates:
        for tag in tags:
            if (ev.name, tag.name) in seen:
                dupes.append(f"{ev.name}/{tag.name}")
                continue
            agg, raw = tag / "aggregate.csv", tag / "raw.csv"
            if not (agg.exists() and raw.exists()):
                continue
            params = load_params(tag / "params.yaml")
            n_seeds = sum(1 for _ in raw.open()) - 1  # valid seeds (raw rows)
            n_failed = sum(1 for s in tag.glob("seed_*") if s.is_dir()
                           and (s / "run-info.yaml").exists()
                           and "status: failed"
                           in (s / "run-info.yaml").read_text())
            # Guard: a cell whose run is still in flight (or was cut short) must
            # not enter the bundle as a smaller-than-configured cell.
            try:
                expected = int(float(params.get("n_seeds", "")))
            except (TypeError, ValueError):
                expected = None
            if (expected is not None and n_seeds < expected
                    and not args.allow_partial):
                skipped.append(f"{ev.name}/{tag.name} ({n_seeds}/{expected})")
                continue
            row = {"evaluation": ev.name, "tag": tag.name,
                   "block": block_for(ev.name), "n_seeds": n_seeds,
                   "n_failed": n_failed}
            for key in MANIFEST_KEYS:
                if key in ("evaluation", "tag", "block", "n_seeds", "n_failed"):
                    continue
                val = fmt_cell(params.get(key, ""))
                if key == "checkpoint" and val:
                    val = Path(val).name
                row[key] = val
            if not row["sim_time"]:
                metas = sorted(tag.glob("seed_*/meta.yaml"))
                if metas:
                    m = load_params(metas[0])
                    row["sim_time"] = fmt_cell(m.get("simDuration", ""))
            # beamformingMethod is written by the scenario into each seed's
            # meta.yaml, not into the cell params.yaml, so fall back the same way.
            if not row.get("beamformingMethod"):
                metas = sorted(tag.glob("seed_*/meta.yaml"))
                if metas:
                    m = load_params(metas[0])
                    row["beamformingMethod"] = fmt_cell(m.get("beamformingMethod", ""))
            seen.add((ev.name, tag.name))
            rows.append(row)
            for f in ("aggregate.csv", "raw.csv", "params.yaml",
                      "feature-importance.csv", "feature-importance-summary.md"):
                src = tag / f
                if not src.exists():
                    continue
                dst = dest / ev.name / tag.name / f
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
                n_files += 1
            if write_action_distribution(tag, dest / ev.name / tag.name):
                n_files += 1

    dest.mkdir(parents=True, exist_ok=True)
    with open(dest / "manifest.csv", "w", newline="") as f:
        # Single source of truth: the same key list the rows are built from.
        # (The previous hardcoded list omitted beamformingMethod, which
        # MANIFEST_KEYS adds, so writing the manifest raised ValueError.)
        w = csv.DictWriter(f, fieldnames=MANIFEST_KEYS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    print(f"synced {len(rows)} cells ({n_files} files) -> {dest}")
    if skipped:
        print(f"skipped {len(skipped)} incomplete cell(s): " + ", ".join(skipped))
    if dupes:
        print(f"ignored {len(dupes)} cell(s) already taken from an earlier tree: "
              + ", ".join(dupes))
    for block, c in sorted(Counter(r["block"] for r in rows).items()):
        print(f"  {block or '(unmapped)'}: {c} cells")


if __name__ == "__main__":
    main()
