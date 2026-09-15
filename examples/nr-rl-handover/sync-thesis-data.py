#!/usr/bin/env python3
"""sync-thesis-data.py - mirror thesis-grade CSVs into the EvaluationData bundle.

Copies aggregate.csv, raw.csv and params.yaml per cell (~360 KB) and writes
manifest.csv (cell -> narrative block, mode, seed count, regime). Reads
./results and skips _archive/ and the diagnostic probes.

Usage: python3 sync-thesis-data.py [--dest /path/to/Master-Thesis-Overleaf/EvaluationData]
"""

import argparse
import csv
import shutil
import sys
from pathlib import Path

RESULTS = Path("results")

# Diagnostic probes kept in results/ for the KB but not part of the thesis
# bundle (no narrative block; few seeds, variant-specific seed sets).
EXCLUDE = {"delack-race", "buffer-2d-sweep", "rlc-buffer-sweep"}

BLOCKS = {
    "capacity-probe": "B0",
    "transport-comparison-load-matched": "B1x",
    "transport-comparison": "B1",
    "tcp-variants": "B1",
    "a3-sweep": "B2",
    "agent-eval-": "B3",
    "nr-rl-handover-a3-baseline-30mhz-tcp-ul": "B3",
    "nr-rl-handover-a3-baseline-30mhz-quic-ul": "B3",
    "topology-hexgrid": "B3b",
    "mtx-ul": "B4",
    "a3-extremes-alt": "B4",
}

MANIFEST_KEYS = [
    "evaluation", "tag", "block", "mode", "n_seeds", "algorithm",
    "bandwidthMhz", "numerology", "transportProtocol", "trafficRateMbps",
    "ueSpeed", "startHeight", "endHeight", "addInterferingUes",
    "a3HysteresisDb", "a3TttMs", "tcpVariant", "checkpoint", "sim_time",
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
    ap.add_argument("--allow-partial", action="store_true",
                    help="Also sync cells that hold fewer valid seeds than their "
                         "params.yaml n_seeds (default: skip them, so a running "
                         "campaign cannot leak partial cells into the bundle)")
    args = ap.parse_args()
    dest = Path(args.dest)
    if not RESULTS.is_dir():
        sys.exit(f"no {RESULTS}/ dir here — run from the nr-rl-handover example dir")

    rows = []
    n_files = 0
    skipped = []
    evals = sorted(p for p in RESULTS.iterdir() if p.is_dir()
                   and not p.name.startswith("_") and p.name not in EXCLUDE)
    for ev in evals:
        ev_params = load_params(ev / "params.yaml")
        for tag in sorted(p for p in ev.iterdir() if p.is_dir()):
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
            rows.append(row)
            for f in ("aggregate.csv", "raw.csv", "params.yaml"):
                src = tag / f
                if not src.exists():
                    continue
                dst = dest / ev.name / tag.name / f
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
                n_files += 1

    dest.mkdir(parents=True, exist_ok=True)
    with open(dest / "manifest.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["evaluation", "tag", "block", "mode",
                                          "n_seeds", "n_failed", "algorithm",
                                          "bandwidthMhz", "numerology",
                                          "transportProtocol", "trafficRateMbps",
                                          "ueSpeed", "startHeight", "endHeight",
                                          "addInterferingUes", "a3HysteresisDb",
                                          "a3TttMs", "tcpVariant", "checkpoint",
                                          "sim_time"])
        w.writeheader()
        w.writerows(rows)

    print(f"synced {len(rows)} cells ({n_files} files) -> {dest}")
    if skipped:
        print(f"skipped {len(skipped)} incomplete cell(s): " + ", ".join(skipped))
    from collections import Counter
    for block, c in sorted(Counter(r["block"] for r in rows).items()):
        print(f"  {block or '(unmapped)'}: {c} cells")


if __name__ == "__main__":
    main()
