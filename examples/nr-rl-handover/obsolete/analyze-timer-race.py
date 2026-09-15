#!/usr/bin/env python3
"""
analyze-timer-race.py — timer-race and stall diagnostics for a result family.

Complements analyze-evaluations.py (which aggregates the KPI columns): this
script answers the mechanism questions of BUGS-ISSUES #34 with per-seed
evidence:

  * stall state     delivery rate after t = 20 s (pkt/s) and total MB;
                    a rate of 3.4-3.6 pkt/s is the 1-segment RTO attractor
  * RTO cycles      CA_OPEN -> CA_LOSS event count and interval distribution.
                    In the trap this interval equals the MinRto floor to the
                    millisecond; once the timer race is broken it is either
                    absent or much larger
  * timer race      CA_LOSS -> CA_OPEN latency (the retransmission round trip)
  * trigger intact  NrRlc::TxDrop count (should stay ~400/seed at a 180 kB cap
                    even when the fatality is fixed)
  * RTT             nr-rl-rtt.csv mean/median

Usage:
    python3 analyze-timer-race.py results/delack-race [--stall-threshold 500]
"""

import argparse
import csv
import statistics
import sys
from pathlib import Path

STALL_RATE = 500.0  # pkt/s after t = 20 s; the trap sits at ~3.6


def read_csv(path):
    if not path.exists():
        return []
    with open(path) as f:
        rows = list(csv.reader(f))
    return [r for r in rows[1:] if len(r) >= 2] if rows else []


def seed_stats(seed_dir, stall_rate=STALL_RATE):
    st = {}

    sink = read_csv(seed_dir / "sink-packets.csv")
    if not sink:
        return None
    times = [float(r[0]) for r in sink]
    bytes_ = [float(r[1]) for r in sink]
    st["totalMB"] = sum(bytes_) / 1e6
    tail = [(t, b) for t, b in zip(times, bytes_) if t >= 20.0]
    span = max(times[-1] - 20.0, 1e-9)
    st["tailRate"] = len(tail) / span
    st["stalled"] = st["tailRate"] < stall_rate
    st["lastPkt"] = times[-1]

    drops = read_csv(seed_dir / "nr-rl-rlc-tx-drop.csv")
    st["drops"] = len(drops)
    st["firstDrop"] = float(drops[0][0]) if drops else float("nan")

    cong = read_csv(seed_dir / "nr-rl-congestion.csv")
    open_to_loss = []
    loss_to_open = []
    t_open = None
    t_loss = None
    for row in cong:
        t, a, b = float(row[0]), row[1], row[2]
        if a == "CA_OPEN" and b == "CA_LOSS":
            if t_open is not None:
                open_to_loss.append(t - t_open)
            t_loss = t
        elif a == "CA_LOSS" and b == "CA_OPEN":
            if t_loss is not None:
                loss_to_open.append(t - t_loss)
            t_open = t
    st["rtoCount"] = len(open_to_loss)
    st["rtoP50ms"] = statistics.median(open_to_loss) * 1000 if open_to_loss else float("nan")
    # intervals within 2.5x of the smallest => the deterministic floor cycle
    if open_to_loss:
        floor = min(open_to_loss)
        st["rtoAtFloor"] = sum(1 for x in open_to_loss if x <= 2.5 * floor)
    else:
        st["rtoAtFloor"] = 0
    st["recP50ms"] = statistics.median(loss_to_open) * 1000 if loss_to_open else float("nan")

    rtt = [float(r[1]) for r in read_csv(seed_dir / "nr-rl-rtt.csv")]
    st["rttAvgMs"] = statistics.mean(rtt) if rtt else float("nan")
    st["rttP50Ms"] = statistics.median(rtt) if rtt else float("nan")
    st["rttN"] = len(rtt)
    return st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("family", type=Path, help="results/<name> directory")
    ap.add_argument("--stall-threshold", type=float, default=STALL_RATE)
    ap.add_argument("--csv", type=Path, default=None,
                    help="also write per-seed rows (arm, seed, metrics) for plotting")
    args = ap.parse_args()
    stall_rate = args.stall_threshold

    tags = sorted(d for d in args.family.iterdir() if d.is_dir()) if args.family.is_dir() else []
    csv_rows = []
    if not tags:
        sys.exit(f"no scenario tags under {args.family}")

    for tag in tags:
        seeds = sorted(tag.glob("seed_*"), key=lambda p: int(p.name.split("_")[1]))
        rows = {s.name: seed_stats(s, stall_rate) for s in seeds}
        rows = {k: v for k, v in rows.items() if v}
        for name, r in rows.items():
            csv_rows.append(dict(arm=tag.name, seed=name.replace("seed_", ""), **r))
        if not rows:
            continue
        vals = list(rows.values())

        def med(key):
            v = [r[key] for r in vals if r[key] == r[key]]  # skip NaN
            return statistics.median(v) if v else float("nan")

        dead = [k for k, r in rows.items() if r["stalled"]]
        print(f"=== {tag.name}  ({len(rows)} seeds)")
        print(f"    median total   {med('totalMB'):8.1f} MB     "
              f"median tail {med('tailRate'):8.1f} pkt/s     stalled {len(dead)}/{len(rows)}"
              + (f"  [{', '.join(d.replace('seed_', 's') for d in dead)}]" if dead else ""))
        print(f"    RTO cycles     n={med('rtoCount'):6.0f}   p50={med('rtoP50ms'):7.1f} ms   "
              f"at-floor {med('rtoAtFloor'):6.0f}   CA_LOSS->CA_OPEN p50={med('recP50ms'):6.1f} ms")
        print(f"    RLC TX drops   {med('drops'):8.0f}   first {med('firstDrop'):6.2f} s   "
              f"RTT avg/p50 {med('rttAvgMs'):6.1f} / {med('rttP50Ms'):6.1f} ms")

    if args.csv:
        cols = ["arm", "seed", "totalMB", "tailRate", "stalled", "drops", "firstDrop",
                "rtoCount", "rtoP50ms", "rtoAtFloor", "recP50ms", "rttAvgMs", "rttP50Ms", "rttN", "lastPkt"]
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in sorted(csv_rows, key=lambda x: (x["arm"], int(x["seed"]))):
                w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
        print(f"\nwrote {args.csv} ({len(csv_rows)} rows)")

    # per-seed detail for the smallest family size (the mechanism evidence)
    print("\nper-seed detail (total MB | tail pkt/s | RTO n / p50 ms | drops):")
    for tag in tags:
        seeds = sorted(tag.glob("seed_*"), key=lambda p: int(p.name.split("_")[1]))
        cells = []
        for s in seeds:
            r = seed_stats(s, stall_rate)
            if not r:
                continue
            cells.append(f"s{s.name.split('_')[1]}:{r['totalMB']:.0f}MB/"
                         f"{r['tailRate']:.0f}p/{r['rtoCount']}x{r['rtoP50ms']:.0f}ms/{r['drops']}d")
        print(f"  {tag.name:26s} " + " ".join(cells))


if __name__ == "__main__":
    main()
