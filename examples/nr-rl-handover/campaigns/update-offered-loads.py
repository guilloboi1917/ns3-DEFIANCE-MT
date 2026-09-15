#!/usr/bin/env python3
"""Set the offered loads of a transport-comparison config from a capacity probe.

Per band, takes the median probe goodput as the measured ceiling and rewrites
trafficRateMbps to max(ceil(1.1 * ceiling), ceiling + 5), so every transport is
offered strictly more than the path carries. The first write backs up the
config to <config>.orig.

Usage: python3 campaigns/update-offered-loads.py --probe-results <dir> --config <yaml> [--dry-run]
"""

import argparse
import csv
import re
import statistics
import sys
from pathlib import Path


def median_goodput_mbps(seed_dir: Path, sim_time: float) -> float | None:
    p = seed_dir / "sink-packets.csv"
    if not p.exists():
        return None
    total = 0.0
    with open(p) as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) >= 2:
                try:
                    total += float(row[1])
                except ValueError:
                    pass
    return total * 8.0 / 1e6 / sim_time


def probe_ceilings(probe_root: Path) -> dict[int, float]:
    """{bandwidthMhz: median delivered Mbps} from the probe result dirs."""
    out: dict[int, float] = {}
    for tag_dir in sorted(p for p in probe_root.iterdir() if p.is_dir()):
        m = re.match(r"bw(\d+)-", tag_dir.name)
        if not m:
            continue
        band = int(m.group(1))
        # sim_time from the tag's own meta.yaml, so the helper does not depend
        # on the probe config being read here
        sim_time = 50.0
        meta = tag_dir / "seed_1" / "meta.yaml"
        if meta.exists():
            for line in meta.read_text().splitlines():
                if line.startswith("simDuration:"):
                    sim_time = float(line.split(":", 1)[1])
        vals = [v for v in (median_goodput_mbps(d, sim_time)
                            for d in sorted(tag_dir.glob("seed_*"))) if v]
        if vals:
            out[band] = statistics.median(vals)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-results", required=True, type=Path)
    ap.add_argument("--config", required=True, type=Path)
    ap.add_argument("--factor", type=float, default=1.1)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if not args.probe_results.is_dir():
        sys.exit(f"probe results not found: {args.probe_results}")
    ceilings = probe_ceilings(args.probe_results)
    if not ceilings:
        sys.exit(f"no probe cells with sink traces under {args.probe_results}")

    print("measured ceilings (median delivered goodput):")
    offered: dict[int, int] = {}
    for band, ceiling in sorted(ceilings.items()):
        rate = max(int(ceiling * args.factor + 0.999), int(ceiling) + 5)
        offered[band] = rate
        print(f"  bw{band:<3} ceiling {ceiling:6.2f} Mbps -> offered {rate} Mbps "
              f"({rate / ceiling:.2f}x)")

    text = args.config.read_text()
    changes = []

    def rewrite(match: re.Match) -> str:
        line, band = match.group(0), int(match.group(1))
        if band not in offered:
            return line
        new = re.sub(r"trafficRateMbps:\s*[\d.]+",
                     f"trafficRateMbps: {offered[band]}", line)
        if new != line:
            changes.append(f"  bw{band}: {line.strip()}  ->  {new.strip()}")
        return new

    text = re.sub(r"\{[^\n]*bandwidthMhz:\s*(\d+)[^\n]*\}", rewrite, text)

    if not changes:
        print("no changes needed (config already matches the probe)")
    else:
        print("offered-load updates:")
        print("\n".join(changes))

    if args.dry_run:
        print("dry-run: config not written")
        return 0

    backup = args.config.with_suffix(args.config.suffix + ".orig")
    if not backup.exists():
        backup.write_text(args.config.read_text())
        print(f"backup written: {backup.name}")
    args.config.write_text(text)
    print(f"updated {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
