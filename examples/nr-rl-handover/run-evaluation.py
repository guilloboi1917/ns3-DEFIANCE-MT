#!/usr/bin/env python3
"""
run-evaluation.py — Batch evaluation runner for defiance-nr-rl-handover.

Usage:
    cd /path/to/ns-3-dev
    python3 contrib/defiance/examples/nr-rl-handover/run-evaluation.py scenarios.yaml

Reads a YAML scenario matrix, runs ns-3 for each (scenario, seed) via
./ns3 run with --logging=true and per-seed output directories, then parses
the generated CSV files to extract evaluation metrics. Produces per-scenario
raw.csv (per-seed) and aggregate.csv (statistics across seeds).

See TODO-PLANNING.md §21 for the metric set.
"""

import argparse
import csv
import os
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


# ---------------------------------------------------------------------------
#   Constants
# ---------------------------------------------------------------------------

NS3_BIN = "defiance-nr-rl-handover"
RESULTS_DIR = Path("results")


# ---------------------------------------------------------------------------
#   Helpers
# ---------------------------------------------------------------------------

def build_ns3_cmd(scenario, common, eval_defaults, seed, output_dir):
    """Build the shell command list for ns3 run (ns3 must be on PATH)."""
    params = dict(eval_defaults)
    params.update(common)
    params.update(scenario)
    algo = params.pop("algorithm", "agent")

    args = f"{NS3_BIN}"
    args += f" --simDuration={params['sim_time']}"
    args += f" --seed={seed}"
    args += f" --runId={seed}"
    args += f" --topology={params.get('topology', 'hexgrid')}"
    args += f" --flowDirection={params.get('flowDirection', 'ul')}"
    args += f" --uavMobility={params.get('uavMobility', 'random-waypoint')}"
    args += f" --tcpVariant={params.get('tcpVariant', 'TcpBbr')}"
    args += f" --startHeight={params.get('startHeight', 80)}"
    args += f" --endHeight={params.get('endHeight', 300)}"
    args += f" --addInterferingUes={params.get('addInterferingUes', 0)}"
    args += f" --aerialUeRatio={params.get('aerialUeRatio', 0.0)}"
    args += f" --transportProtocol={params.get('transportProtocol', 'tcp')}"
    args += " --logging=true"
    args += " --parallel=0"
    args += f" --outputDir={output_dir}"

    if algo == "agent":
        args += " --rlMode=true"
        args += " --handoverAlgorithm=agent"
        args += f" --stepTime={params.get('stepTime', 480)}"
        args += " --handoverMargin=-999"
    else:
        args += " --rlMode=false"
        args += f" --handoverAlgorithm={algo}"
        args += f" --handoverMargin={params.get('handoverMargin', 3.0)}"

    return ["ns3", "run", args]


def parse_stdout(stdout: str) -> dict:
    """Extract end-of-sim metrics from stdout."""
    result = {}
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("Simulation time:"):
            result["simTime"] = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Total handovers:"):
            result["handovers"] = int(line.split(":")[1].strip())
        elif line.startswith("Total received:"):
            result["goodputMbps_stdout"] = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Total retransmissions:"):
            result["retransmissions"] = int(line.split(":")[1].strip())
        elif line.startswith("Total RLF:"):
            result["rlfCount"] = int(line.split(":")[1].strip())
        elif line.startswith("Average RTT:"):
            result["rttMs_stdout"] = float(line.split(":")[1].strip().split()[0])
    return result


def parse_sink_packets(path: Path) -> float:
    """Compute total goodput (Mbps) from sink-packets.csv.

    Format: time,packetSize (one row per received packet).
    goodput = sum(packetSizes) * 8 / 1e6 / simDuration
    """
    if not path.exists():
        return 0.0
    df = pd.read_csv(path, header=None, names=["time", "bytes"])
    return int(df["bytes"].sum())


def parse_rtt(path: Path) -> tuple:
    """Parse nr-rl-rtt.csv. Format: time,rttMs.

    Returns (rtt_array, first_rtt_time).
    """
    if not path.exists():
        return np.array([]), None
    df = pd.read_csv(path, header=None, names=["time", "rttMs"])
    if len(df) == 0:
        return np.array([]), None
    return df["rttMs"].to_numpy(), df["time"].iloc[0]


def parse_dl_sinr(path: Path) -> np.ndarray:
    """Parse dl_sinr.csv. Format: time,cellId,rnti,sinrDb.

    Returns array of SINR values in dB.
    """
    if not path.exists():
        return np.array([])
    df = pd.read_csv(path, header=None, names=["time", "cellId", "rnti", "sinrDb"])
    return df["sinrDb"].to_numpy()


def parse_ue_meas(path: Path) -> tuple:
    """Parse ue_meas_report.csv. Format: time,cellId,rnti,rsrp,rsrq,isServingCell.

    Returns (rsrp_array, rsrq_array) for serving cell only.
    """
    if not path.exists():
        return np.array([]), np.array([])
    df = pd.read_csv(path, header=None,
                     names=["time", "cellId", "rnti", "rsrp", "rsrq", "isServingCell"])
    serving = df[df["isServingCell"] == 1]
    return serving["rsrp"].to_numpy(), serving["rsrq"].to_numpy()


def parse_handovers(path: Path) -> tuple:
    """Parse nr-rl-handovers.csv. Format: time,cellId.

    Returns (count, times_array, cells_array).
    """
    if not path.exists():
        return 0, np.array([]), np.array([])
    df = pd.read_csv(path, header=None, names=["time", "cellId"])
    return len(df), df["time"].to_numpy(), df["cellId"].to_numpy()


def count_ping_pong(times: np.ndarray, cells: np.ndarray, window_s: float = 2.0) -> int:
    """Count ping-pong sequences: A -> B -> A within window_s.

    A ping-pong is three consecutive handovers where:
      cell[t] == cell[t+2] != cell[t+1]  and  times[t+2] - times[t] < window_s.
    """
    if len(times) < 3:
        return 0
    mask = (
        (cells[:-2] == cells[2:])
        & (cells[:-2] != cells[1:-1])
        & (times[2:] - times[:-2] < window_s)
    )
    return int(mask.sum())


def aggregate_results(rows: list) -> dict:
    """Compute aggregate statistics (mean, std, p5, p50, p95) per metric."""
    if not rows:
        return {}
    metrics = [k for k in rows[0].keys() if k != "seed"]
    agg = {}
    for m in metrics:
        values = sorted([r[m] for r in rows if m in r])
        if not values:
            continue
        agg[m] = {
            "mean": float(np.mean(values)),
            "std": float(np.std(values)),
            "p5": float(np.percentile(values, 5)),
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
        }
    return agg


def write_raw_csv(path: Path, rows: list):
    """Write per-seed raw data with headers."""
    if not rows:
        return
    fieldnames = [
        "seed", "goodputMbps", "handovers", "pingPongCount", "rlfCount",
        "rttMs_avg", "rttMs_p50",
        "sinrDb_avg", "sinrDb_p50",
        "rsrpServingDbm_avg", "rsrqServingDb_avg",
        "retransmissions", "tcpConnectTime", "simTime"
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


UNIT_MAP = {
    "goodputMbps": "Mbps",
    "handovers": "count",
    "pingPongCount": "count",
    "rlfCount": "count",
    "rttMs": "ms",
    "sinrDb": "dB",
    "rsrpServingDbm": "dBm",
    "rsrqServingDb": "dB",
    "retransmissions": "count",
    "tcpConnectTime": "s",
    "simTime": "s",
}


def write_aggregate_csv(path: Path, aggregate: dict):
    """Write aggregated statistics with headers."""
    if not aggregate:
        return
    fieldnames = ["metric", "mean", "std", "p5", "p50", "p95", "unit"]
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fieldnames)
        for metric, stats in sorted(aggregate.items()):
            unit = UNIT_MAP.get(metric, "")
            writer.writerow([
                metric,
                f"{stats['mean']:.4f}",
                f"{stats['std']:.4f}",
                f"{stats['p5']:.4f}",
                f"{stats['p50']:.4f}",
                f"{stats['p95']:.4f}",
                unit,
            ])


# ---------------------------------------------------------------------------
#   Per-seed execution (runs inside ThreadPoolExecutor workers)
# ---------------------------------------------------------------------------

def run_one_seed(sc, common, eval_defaults, seed, tag_dir, sim_time):
    """Run one seed: execute ns-3 subprocess, parse CSVs, return result row."""
    seed_dir = tag_dir / f"seed_{seed}"
    seed_dir.mkdir(parents=True, exist_ok=True)

    cmd = build_ns3_cmd(sc, common, eval_defaults, seed, str(seed_dir))

    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=600,
        )
    except subprocess.TimeoutExpired:
        return {"seed": seed, "error": "TIMEOUT"}
    except FileNotFoundError:
        return {"seed": seed, "error": "ns3 command not found on PATH"}

    if proc.returncode != 0:
        stderr_preview = proc.stderr.strip()[:200] if proc.stderr else "(none)"
        return {"seed": seed, "error": f"FAILED rc={proc.returncode}: {stderr_preview}"}

    stdout_metrics = parse_stdout(proc.stdout)

    total_bytes = parse_sink_packets(seed_dir / "sink-packets.csv")
    goodput_mbps = total_bytes * 8 / 1e6 / sim_time if sim_time > 0 else 0.0

    rtt_samples, first_rtt_time = parse_rtt(seed_dir / "nr-rl-rtt.csv")
    rtt_avg = float(np.mean(rtt_samples)) if rtt_samples.size > 0 else 0.0
    rtt_p50 = float(np.median(rtt_samples)) if rtt_samples.size > 0 else 0.0

    sinr_vals = parse_dl_sinr(seed_dir / "dl_sinr.csv")
    sinr_avg = float(np.mean(sinr_vals)) if sinr_vals.size > 0 else 0.0
    sinr_p50 = float(np.median(sinr_vals)) if sinr_vals.size > 0 else 0.0

    rsrp_vals, rsrq_vals = parse_ue_meas(seed_dir / "ue_meas_report.csv")
    rsrp_avg = float(np.mean(rsrp_vals)) if rsrp_vals.size > 0 else 0.0
    rsrq_avg = float(np.mean(rsrq_vals)) if rsrq_vals.size > 0 else 0.0

    ho_count, ho_times, ho_cells = parse_handovers(seed_dir / "nr-rl-handovers.csv")
    ping_pong = count_ping_pong(ho_times, ho_cells)
    tcp_connect = first_rtt_time if first_rtt_time else 0.0

    return {
        "seed": seed,
        "goodputMbps": round(goodput_mbps, 4),
        "handovers": ho_count,
        "pingPongCount": ping_pong,
        "rlfCount": stdout_metrics.get("rlfCount", 0),
        "rttMs_avg": round(rtt_avg, 2),
        "rttMs_p50": round(rtt_p50, 2),
        "sinrDb_avg": round(sinr_avg, 2),
        "sinrDb_p50": round(sinr_p50, 2),
        "rsrpServingDbm_avg": round(rsrp_avg, 2),
        "rsrqServingDb_avg": round(rsrq_avg, 2),
        "retransmissions": stdout_metrics.get("retransmissions", 0),
        "tcpConnectTime": round(tcp_connect, 4),
        "simTime": round(stdout_metrics.get("simTime", 0.0), 4),
    }


# ---------------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Batch evaluation runner for defiance-nr-rl-handover"
    )
    parser.add_argument("scenarios_yaml", type=str,
                        help="Path to YAML scenario matrix")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output directory (default: ./results/<eval_name>)")
    parser.add_argument("--jobs", "-j", type=int, default=1,
                        help="Number of seeds to run concurrently (default: 1)")
    parser.add_argument("--dry-run", "-n", action="store_true",
                        help="Print commands without executing")
    args = parser.parse_args()

    # Load YAML
    yaml_path = Path(args.scenarios_yaml)
    if not yaml_path.exists():
        print(f"[ERROR] Scenario file not found: {yaml_path}")
        sys.exit(1)

    with open(yaml_path) as f:
        config = yaml.safe_load(f)

    eval_cfg = config.get("evaluation", config)
    eval_name = eval_cfg.get("name", "evaluation")
    n_seeds = eval_cfg.get("n_seeds", 10)
    common = eval_cfg.get("common", {})
    eval_defaults = {
        "sim_time": eval_cfg.get("sim_time", 30),
    }
    scenarios = eval_cfg.get("scenarios", [])

    if not scenarios:
        print("[ERROR] No scenarios defined in YAML")
        sys.exit(1)

    # Determine output directory (resolve to absolute — ns3 run may change cwd)
    if args.output:
        base_dir = Path(args.output).resolve()
    else:
        base_dir = (RESULTS_DIR / eval_name).resolve()

    print(f"Evaluation: {eval_name}")
    print(f"  Seeds:      1..{n_seeds}")
    print(f"  Scenarios:  {len(scenarios)}")
    print(f"  Output:     {base_dir}")
    print()

    total_runs = len(scenarios) * n_seeds
    run_count = 0
    fail_count = 0
    start_wall = time.time()

    for sc in scenarios:
        tag = sc.get("tag", "untagged")
        algorithm = sc.get("algorithm", "agent")
        tag_dir = base_dir / tag
        tag_dir.mkdir(parents=True, exist_ok=True)

        # Write scenario params for reference
        params_snapshot = dict(eval_defaults)
        params_snapshot.update(common)
        params_snapshot.update(sc)
        params_snapshot["n_seeds"] = n_seeds
        with open(tag_dir / "params.yaml", "w") as f:
            yaml.dump(params_snapshot, f, default_flow_style=False)

        sim_time = eval_defaults["sim_time"]

        # Run each seed (parallel via ThreadPoolExecutor)
        rows = []
        run_count_scenario = 0
        fail_count_scenario = 0
        print(f"[{tag}] Running {n_seeds} seeds with {args.jobs} workers...")

        if not args.dry_run and args.jobs > 1:
            with ThreadPoolExecutor(max_workers=args.jobs) as executor:
                futures = {
                    executor.submit(
                        run_one_seed, sc, common, eval_defaults, seed,
                        tag_dir, sim_time
                    ): seed for seed in range(1, n_seeds + 1)
                }
                for future in as_completed(futures):
                    seed = futures[future]
                    try:
                        result = future.result()
                    except Exception as e:
                        print(f"  seed={seed}: EXCEPTION: {e}")
                        fail_count_scenario += 1
                        continue
                    if "error" in result:
                        print(f"  seed={seed}: {result['error']}")
                        fail_count_scenario += 1
                    else:
                        rows.append(result)
                    run_count_scenario += 1
                    elapsed = time.time() - start_wall
                    done_global = run_count + run_count_scenario
                    pct = done_global / total_runs * 100
                    print(f"  [{done_global}/{total_runs} {pct:.0f}%] "
                          f"seed={seed}, {elapsed:.0f}s elapsed")
        else:
            for seed in range(1, n_seeds + 1):
                if args.dry_run:
                    seed_dir = tag_dir / f"seed_{seed}"
                    cmd = build_ns3_cmd(sc, common, eval_defaults, seed, str(seed_dir))
                    print(f"  seed={seed}: {' '.join(str(c) for c in cmd)}")
                    continue

                result = run_one_seed(sc, common, eval_defaults, seed,
                                      tag_dir, sim_time)
                if "error" in result:
                    print(f"  seed={seed}: {result['error']}")
                    fail_count_scenario += 1
                else:
                    rows.append(result)
                run_count_scenario += 1
                elapsed = time.time() - start_wall
                done_global = run_count + run_count_scenario
                pct = done_global / total_runs * 100
                print(f"  [{done_global}/{total_runs} {pct:.0f}%] "
                      f"seed={seed}, {elapsed:.0f}s elapsed")

        run_count += run_count_scenario
        fail_count += fail_count_scenario

        if args.dry_run:
            print(f"  -> (dry-run, skipped)")
            continue

        # Write results for this scenario
        if rows:
            write_raw_csv(tag_dir / "raw.csv", rows)

            # Build a clean aggregate dict (metric name -> stats)
            agg_data = {}
            for r in rows:
                for key in ["goodputMbps", "handovers", "pingPongCount", "rlfCount",
                            "rttMs_avg", "rttMs_p50", "sinrDb_avg", "sinrDb_p50",
                            "rsrpServingDbm_avg", "rsrqServingDb_avg",
                            "retransmissions", "tcpConnectTime", "simTime"]:
                    if key not in agg_data:
                        agg_data[key] = []
                    agg_data[key].append(r.get(key, 0))

            aggregate = {}
            for metric, vals in agg_data.items():
                sorted_vals = sorted(vals)
                aggregate[metric] = {
                    "mean": float(np.mean(sorted_vals)),
                    "std": float(np.std(sorted_vals)),
                    "p5": float(np.percentile(sorted_vals, 5)),
                    "p50": float(np.percentile(sorted_vals, 50)),
                    "p95": float(np.percentile(sorted_vals, 95)),
                }

            write_aggregate_csv(tag_dir / "aggregate.csv", aggregate)

            g = aggregate.get("goodputMbps", {})
            h = aggregate.get("handovers", {})
            r = aggregate.get("rlfCount", {})
            print(f"  -> goodput: p50={g.get('p50','?'):>6.2f} "
                  f"p5={g.get('p5','?'):>6.2f} p95={g.get('p95','?'):>6.2f} Mbps | "
                  f"handovers: p50={h.get('p50','?'):>5.1f} | "
                  f"RLF: p50={r.get('p50','?'):>4.1f}")
        else:
            print(f"  -> No successful runs")
        print()

    # Final summary
    elapsed = time.time() - start_wall
    print(f"Done. {run_count} runs, {fail_count} failed, "
          f"{elapsed:.0f}s elapsed ({elapsed/60:.1f} min)")
    print(f"Results: {base_dir.resolve()}")


if __name__ == "__main__":
    main()
