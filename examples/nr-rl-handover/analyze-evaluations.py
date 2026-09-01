#!/usr/bin/env python3
"""analyze-evaluations.py — unified analysis of defiance evaluation results.

Consumes any per-seed result directory produced by run-evaluations.py (or the
legacy evaluate-agent.py / run-evaluation.py / run-matrix-ul.py layouts, which
share the same per-(scenario, seed) structure). For each scenario it reads the
per-seed trace files (direction-aware), computes the full metric set with
units, and writes:

    raw.csv          one row per seed (metric columns)
    aggregate.csv    metric, unit, mean, std, p5, p50, p95 across seeds
    sources.md       which source files were present/missing per seed

Missing or empty sources yield NaN (not 0) so "not applicable" is visible in
the tables. TCP-only and RL-only metrics are NaN for runs where they do not
apply. The per-seed walltime comes from run-info.yaml (written by
run-evaluations.py); legacy dirs lack it -> NaN.

Metric set (units): goodput [Mbps], handovers/completions/ping-pong/RLF/
retransmissions/actions [count], hoRate [1/s], e2e loss [%] (FlowMonitor
tx-rx gap of the UAV data flow), HARQ corrupt [%] + TBLER + per-TB SINR/MCS
(from nr-rl-{dl,ul}-rx-sinr.csv), direction SINR avg/p50 [dB], serving RSRP
avg/p50 [dBm] and RSRQ [dB], RTT avg/p50 [ms], FlowMonitor delay/jitter [ms],
TCP connect time [s], RL reward/action metrics, simTime and walltime [s].

Usage:
    python3 analyze-evaluations.py <results-dir-or-scenario-dir> [--dry-run]
"""

import argparse
import csv
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


# ---------------------------------------------------------------------------
# Metric catalog (canonical name -> unit, description). Direction-specific
# SINR keys get a dl/ul prefix at parse time.
# ---------------------------------------------------------------------------

BASE_METRICS = [
    ("goodputMbps", "Mbps", "achieved goodput from sink bytes"),
    ("goodputMbps_obs_mean", "Mbps", "mean per-step goodput from rl_reward.csv (RL)"),
    ("reward_total", "-", "total episode reward (RL)"),
    ("reward_mean", "-", "mean per-step reward (RL)"),
    ("handovers", "count", "completed handovers (HandoverEndOk)"),
    ("handovers_completed", "count", "completed handovers (RL, alias of handovers)"),
    ("handover_steps", "count", "reward steps inside the hangover window (RL)"),
    ("pingPongCount", "count", "A->B->A ping-pong patterns"),
    ("pingpong_steps", "count", "reward steps flagged as ping-pong (RL)"),
    ("hoRate", "1/s", "completed handovers per second"),
    ("rlfCount", "count", "radio link failures"),
    ("e2eLossPct", "%", "end-to-end IP loss of the UAV data flow (FlowMonitor)"),
    ("corruptPct", "%", "per-TB CRC corruption (rx-sinr corrupt flag)"),
    ("tbler", "-", "mean transport-block error rate (rx-sinr)"),
    ("perTbSinrDb_avg", "dB", "mean per-TB SINR (rx-sinr)"),
    ("perTbSinrDb_p50", "dB", "median per-TB SINR (rx-sinr)"),
    ("mcs_p50", "-", "median MCS (rx-sinr)"),
    ("SinrDb_avg", "dB", "mean direction SINR (dl_sinr / ul_sinr_srs)"),
    ("SinrDb_p50", "dB", "median direction SINR"),
    ("rsrpServingDbm_avg", "dBm", "mean serving-cell RSRP (ue_meas_report)"),
    ("rsrpServingDbm_p50", "dBm", "median serving-cell RSRP"),
    ("rsrqServingDb_avg", "dB", "mean serving-cell RSRQ"),
    ("rttMs_avg", "ms", "mean RTT (nr-rl-rtt)"),
    ("rttMs_p50", "ms", "median RTT"),
    ("delayMs_avg", "ms", "mean one-way delay of the UAV flow (FlowMonitor)"),
    ("jitterMs_avg", "ms", "mean jitter of the UAV flow (FlowMonitor)"),
    ("tcpConnectTime", "s", "time of the first RTT sample (TCP connect; UDP: NaN)"),
    ("retransmissions", "count", "TCP retransmission events (UDP: NaN)"),
    ("actions_executed", "count", "handover actions executed (RL)"),
    ("actions_noop", "count", "noop actions (RL)"),
    ("actions_blocked", "count", "blocked actions, all reasons (RL)"),
    ("actions_blocked_same_cell", "count", "blocked same-cell actions (RL)"),
    ("actions_blocked_in_progress", "count", "blocked in-progress actions (RL)"),
    ("steps", "count", "reward steps logged (RL)"),
    ("simTime", "s", "simulation duration"),
    ("walltime_s", "s", "per-seed walltime (run-info.yaml; legacy runs: NaN)"),
]

METRIC_UNITS = {name: unit for name, unit, _ in BASE_METRICS}
SINR_SUFFIXES = ("SinrDb_avg", "SinrDb_p50")


def metric_units(flow: str) -> dict:
    """Metric name -> unit, with the direction prefix materialized for SINR."""
    units = dict(METRIC_UNITS)
    for suffix in SINR_SUFFIXES:
        units.pop(suffix)
        units[f"{flow}{suffix}"] = METRIC_UNITS[suffix]
    return units


# ---------------------------------------------------------------------------
# Per-file parsers (all return NaN-friendly values on missing/empty files)
# ---------------------------------------------------------------------------

def _nan() -> float:
    return float("nan")


def read_rows(path: Path, names: list, skiprows=1) -> pd.DataFrame:
    """Read a scenario CSV; returns an empty DataFrame on any failure."""
    if not path.exists():
        return pd.DataFrame()
    try:
        return pd.read_csv(path, skiprows=skiprows, header=None, names=names)
    except Exception:
        return pd.DataFrame()


def parse_sink_goodput(path: Path, sim_time: float) -> float:
    df = read_rows(path, ["time", "bytes"])
    if df.empty or sim_time <= 0:
        return _nan()
    return float(df["bytes"].sum()) * 8.0 / 1e6 / sim_time


def parse_rtt(path: Path):
    """(avg, p50, first_time) in ms; NaN tuple when absent."""
    df = read_rows(path, ["time", "rttMs"])
    if df.empty:
        return _nan(), _nan(), _nan()
    vals = df["rttMs"].to_numpy(dtype=float)
    return float(np.mean(vals)), float(np.median(vals)), float(df["time"].iloc[0])


def parse_sinr(path: Path):
    """Direction SINR (last column positionally: DL 4-col, UL SRS 3-col).
    The files carry a header, so read with pandas auto-detection and take
    the last column."""
    if not path.exists():
        return _nan(), _nan()
    try:
        df = pd.read_csv(path)
    except Exception:
        return _nan(), _nan()
    if df.empty:
        return _nan(), _nan()
    vals = df.iloc[:, -1].to_numpy(dtype=float)
    return float(np.mean(vals)), float(np.median(vals))


def parse_ue_meas(path: Path):
    """(rsrp_avg, rsrp_p50, rsrq_avg) of the serving cell."""
    df = read_rows(path, ["time", "cellId", "rnti", "rsrp", "rsrq", "isServingCell"])
    if df.empty:
        return _nan(), _nan(), _nan()
    serving = df[df["isServingCell"] == 1]
    if serving.empty:
        return _nan(), _nan(), _nan()
    rsrp = serving["rsrp"].to_numpy(dtype=float)
    rsrq = serving["rsrq"].to_numpy(dtype=float)
    return (float(np.mean(rsrp)), float(np.median(rsrp)), float(np.mean(rsrq)))


def parse_handovers(path: Path):
    df = read_rows(path, ["time", "cellId"])
    if df.empty:
        return 0, np.array([]), np.array([])
    return len(df), df["time"].to_numpy(dtype=float), df["cellId"].to_numpy()


def count_ping_pong(times: np.ndarray, cells: np.ndarray, window_s: float = 2.0) -> int:
    if len(times) < 3:
        return 0
    mask = ((cells[:-2] == cells[2:]) & (cells[:-2] != cells[1:-1])
            & (times[2:] - times[:-2] < window_s))
    return int(mask.sum())


def parse_rx_sinr(path: Path):
    """Per-TB RX stats: corrupt %, TBLER, per-TB SINR avg/p50, MCS p50."""
    df = read_rows(path, ["time", "cellId", "rnti", "sinrDb", "mcs", "tbSize",
                          "corrupt", "tbler"])
    if df.empty:
        return {k: _nan() for k in
                ("corruptPct", "tbler", "perTbSinrDb_avg", "perTbSinrDb_p50", "mcs_p50")}
    sinr = df["sinrDb"].to_numpy(dtype=float)
    corrupt = df["corrupt"].to_numpy(dtype=float)
    tbler = df["tbler"].to_numpy(dtype=float)
    mcs = df["mcs"].to_numpy(dtype=float)
    return {
        "corruptPct": float(np.mean(corrupt) * 100.0),
        "tbler": float(np.mean(tbler)),
        "perTbSinrDb_avg": float(np.mean(sinr)),
        "perTbSinrDb_p50": float(np.median(sinr)),
        "mcs_p50": float(np.median(mcs)),
    }


def parse_flowmon(path: Path):
    """e2e loss %, mean delay and jitter of the UAV data flow (max txPackets)."""
    if not path.exists():
        return {"e2eLossPct": _nan(), "delayMs_avg": _nan(), "jitterMs_avg": _nan()}
    try:
        root = ET.parse(path).getroot()
    except Exception:
        return {"e2eLossPct": _nan(), "delayMs_avg": _nan(), "jitterMs_avg": _nan()}
    flows = []
    for el in root.iter("Flow"):
        try:
            tx = int(el.get("txPackets", 0))
            rx = int(el.get("rxPackets", 0))
            delay_sum = float(el.get("delaySum", "0").replace("+", "").replace("ns", ""))
            jitter_sum = float(el.get("jitterSum", "0").replace("+", "").replace("ns", ""))
        except Exception:
            continue
        if tx > 0:
            flows.append((tx, rx, delay_sum, jitter_sum))
    if not flows:
        return {"e2eLossPct": _nan(), "delayMs_avg": _nan(), "jitterMs_avg": _nan()}
    tx, rx, delay_sum, jitter_sum = max(flows, key=lambda f: f[0])
    return {
        "e2eLossPct": float(100.0 * (1.0 - rx / tx)),
        "delayMs_avg": float(delay_sum / tx / 1e6),
        "jitterMs_avg": float(jitter_sum / max(tx - 1, 1) / 1e6),
    }


def parse_reward(path: Path):
    df = read_rows(path, ["t", "goodput", "ref", "min", "normG_raw", "normG",
                          "R_G", "I_ho", "R_H", "pingPong", "reward"])
    if df.empty:
        return {"reward_total": _nan(), "reward_mean": _nan(),
                "goodputMbps_obs_mean": _nan(), "handover_steps": _nan(),
                "pingpong_steps": _nan(), "steps": _nan()}
    return {
        "reward_total": float(df["reward"].sum()),
        "reward_mean": float(df["reward"].mean()),
        "goodputMbps_obs_mean": float(df["goodput"].mean()),
        "handover_steps": float(df["I_ho"].sum()),
        "pingpong_steps": float(df["pingPong"].sum()),
        "steps": float(len(df)),
    }


def parse_actions(path: Path):
    df = read_rows(path, ["t", "a", "target", "current", "outcome"])
    if df.empty:
        return {"actions_executed": _nan(), "actions_noop": _nan(),
                "actions_blocked": _nan(), "actions_blocked_same_cell": _nan(),
                "actions_blocked_in_progress": _nan()}
    out = df["outcome"].value_counts().to_dict()
    same = float(out.get("blocked-same-cell", 0))
    inprog = float(out.get("blocked-in-progress", 0))
    return {
        "actions_executed": float(out.get("executed", 0)),
        "actions_noop": float(out.get("noop", 0)),
        "actions_blocked": same + inprog,
        "actions_blocked_same_cell": same,
        "actions_blocked_in_progress": inprog,
    }


def parse_count(path: Path) -> float:
    """Row count of a time-ordered event CSV (RLF, retransmissions)."""
    if not path.exists():
        return _nan()
    try:
        with open(path) as f:
            return float(sum(1 for _ in f) - 1)
    except Exception:
        return _nan()


def read_meta(seed_dir: Path) -> dict:
    """flowDirection (and friends) from the sim's meta.yaml."""
    p = seed_dir / "meta.yaml"
    if p.exists():
        try:
            with open(p) as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}
    return {}


# ---------------------------------------------------------------------------
# Per-seed + per-scenario assembly
# ---------------------------------------------------------------------------

def _is_failed(seed_dir: Path) -> bool:
    """True if run-info.yaml marks this seed as failed (partial data)."""
    ri = seed_dir / "run-info.yaml"
    if ri.exists():
        try:
            with open(ri) as f:
                return (yaml.safe_load(f) or {}).get("status") == "failed"
        except Exception:
            return False
    return False


def analyze_seed(seed_dir: Path, sim_time: float) -> dict:
    meta = read_meta(seed_dir)
    flow = meta.get("flowDirection", "ul")
    seed = seed_dir.name.replace("seed_", "")

    sinr_csv = "ul_sinr_srs.csv" if flow == "ul" else "dl_sinr.csv"
    rx_csv = f"nr-rl-{flow}-rx-sinr.csv"

    goodput = parse_sink_goodput(seed_dir / "sink-packets.csv", sim_time)
    rtt_avg, rtt_p50, first_rtt = parse_rtt(seed_dir / "nr-rl-rtt.csv")
    sinr_avg, sinr_p50 = parse_sinr(seed_dir / sinr_csv)
    rsrp_avg, rsrp_p50, rsrq_avg = parse_ue_meas(seed_dir / "ue_meas_report.csv")
    ho_count, ho_times, ho_cells = parse_handovers(seed_dir / "nr-rl-handovers.csv")
    rx = parse_rx_sinr(seed_dir / rx_csv)
    fm = parse_flowmon(seed_dir / "nr-rl.flowmonitor")
    rew = parse_reward(seed_dir / "rl_reward.csv")
    act = parse_actions(seed_dir / "rl_actions_full.csv")

    retrans = parse_count(seed_dir / "retransmissions.csv")
    rlf = parse_count(seed_dir / "nr-rl-rlf.csv")

    walltime = _nan()
    ri = seed_dir / "run-info.yaml"
    if ri.exists():
        try:
            with open(ri) as f:
                walltime = float((yaml.safe_load(f) or {}).get("walltime_s", _nan()))
        except Exception:
            walltime = _nan()

    row = {
        "seed": int(seed) if seed.isdigit() else seed,
        "goodputMbps": goodput,
        "goodputMbps_obs_mean": rew["goodputMbps_obs_mean"],
        "reward_total": rew["reward_total"],
        "reward_mean": rew["reward_mean"],
        "handovers": float(ho_count),
        "handovers_completed": float(ho_count),
        "handover_steps": rew["handover_steps"],
        "pingPongCount": float(count_ping_pong(ho_times, ho_cells)),
        "pingpong_steps": rew["pingpong_steps"],
        "steps": rew["steps"],
        "hoRate": float(ho_count) / sim_time if sim_time > 0 else _nan(),
        "rlfCount": rlf,
        "e2eLossPct": fm["e2eLossPct"],
        "corruptPct": rx["corruptPct"],
        "tbler": rx["tbler"],
        "perTbSinrDb_avg": rx["perTbSinrDb_avg"],
        "perTbSinrDb_p50": rx["perTbSinrDb_p50"],
        "mcs_p50": rx["mcs_p50"],
        f"{flow}SinrDb_avg": sinr_avg,
        f"{flow}SinrDb_p50": sinr_p50,
        "rsrpServingDbm_avg": rsrp_avg,
        "rsrpServingDbm_p50": rsrp_p50,
        "rsrqServingDb_avg": rsrq_avg,
        "rttMs_avg": rtt_avg,
        "rttMs_p50": rtt_p50,
        "delayMs_avg": fm["delayMs_avg"],
        "jitterMs_avg": fm["jitterMs_avg"],
        "tcpConnectTime": first_rtt if not np.isnan(first_rtt) else _nan(),
        "retransmissions": retrans,
        "actions_executed": act["actions_executed"],
        "actions_noop": act["actions_noop"],
        "actions_blocked": act["actions_blocked"],
        "actions_blocked_same_cell": act["actions_blocked_same_cell"],
        "actions_blocked_in_progress": act["actions_blocked_in_progress"],
        "simTime": sim_time,
        "walltime_s": walltime,
    }
    return row


def _fmt(v: float) -> str:
    return "" if v is None or (isinstance(v, float) and np.isnan(v)) else f"{v:.6g}"


def analyze_scenario(tag_dir: Path) -> None:
    seed_dirs = sorted(tag_dir.glob("seed_*"),
                       key=lambda p: int(p.name.split("_")[1]) if p.name.split("_")[1].isdigit() else 0)
    if not seed_dirs:
        print(f"[{tag_dir.name}] no seed_N/ dirs — skipped")
        return

    failed = [d for d in seed_dirs if _is_failed(d)]
    seed_dirs = [d for d in seed_dirs if not _is_failed(d)]
    if not seed_dirs:
        print(f"[{tag_dir.name}] all seeds failed — skipped")
        return

    sim_time = _nan()
    meta0 = read_meta(seed_dirs[0])
    sim_time = float(meta0.get("simDuration", _nan()))
    if np.isnan(sim_time):
        p = tag_dir / "params.yaml"
        if p.exists():
            try:
                sim_time = float((yaml.safe_load(p) or {}).get("sim_time", _nan()))
            except Exception:
                pass

    flow = meta0.get("flowDirection", "ul")
    rows = [analyze_seed(d, sim_time) for d in seed_dirs]
    units = metric_units(flow)

    # raw.csv — one row per seed, all metric columns in catalog order (NaN
    # where a source is missing / not applicable -> stable schema across runs).
    columns = ["seed"] + [c for c in units]
    with open(tag_dir / "raw.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: _fmt(v) for k, v in r.items()})

    # aggregate.csv — metric, unit, mean, std, p5, p50, p95 across seeds
    with open(tag_dir / "aggregate.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "unit", "mean", "std", "p5", "p50", "p95"])
        for metric in columns[1:]:
            vals = [r[metric] for r in rows
                    if isinstance(r.get(metric), (int, float))
                    and not np.isnan(r[metric])]
            if not vals:
                w.writerow([metric, units.get(metric, "-"),
                            "nan", "nan", "nan", "nan", "nan"])
                continue
            arr = np.array(vals, dtype=float)
            w.writerow([metric, units.get(metric, "-"),
                        _fmt(np.mean(arr)), _fmt(np.std(arr)),
                        _fmt(np.percentile(arr, 5)), _fmt(np.percentile(arr, 50)),
                        _fmt(np.percentile(arr, 95))])

    # sources.md — which files were present across seeds
    expected = ["meta.yaml", "sink-packets.csv", "nr-rl-rtt.csv", "ue_meas_report.csv",
                "nr-rl-handovers.csv", f"nr-rl-{flow}-rx-sinr.csv",
                "nr-rl.flowmonitor", "rl_reward.csv", "rl_actions_full.csv",
                "retransmissions.csv", "nr-rl-rlf.csv", "run-info.yaml",
                ("ul_sinr_srs.csv" if flow == "ul" else "dl_sinr.csv")]
    with open(tag_dir / "sources.md", "w") as f:
        f.write(f"# Sources — {tag_dir.name} (flowDirection={flow}, sim_time={sim_time:g})\n\n")
        f.write("| file | present |\n|---|---|\n")
        for name in expected:
            present = sum(1 for d in seed_dirs if (d / name).exists())
            f.write(f"| {name} | {present}/{len(seed_dirs)} |\n")

    print(f"[{tag_dir.name}] {len(seed_dirs)} seeds -> raw.csv, aggregate.csv, "
          f"sources.md ({len(columns) - 1} metrics)"
          + (f", {len(failed)} failed excluded" if failed else ""))


def main():
    parser = argparse.ArgumentParser(
        description="Unified analysis of defiance evaluation result directories")
    parser.add_argument("input", type=str,
                        help="Results dir (scenario subdirs), a single scenario "
                             "dir, or a scenario-matrix YAML")
    parser.add_argument("--dry-run", "-n", action="store_true",
                        help="List the scenario dirs that would be analyzed")
    args = parser.parse_args()

    inp = Path(args.input)
    if not inp.exists():
        print(f"[ERROR] input not found: {inp}")
        sys.exit(1)

    if inp.is_file():  # matrix yaml -> results/<name>
        try:
            cfg = yaml.safe_load(inp) or {}
        except Exception:
            cfg = {}
        name = (cfg.get("evaluation", cfg)).get("name", inp.stem)
        inp = Path("results") / name
        if not inp.exists():
            print(f"[ERROR] results dir not found: {inp}")
            sys.exit(1)

    # A directory is a single scenario if it contains seed_* dirs directly.
    if list(inp.glob("seed_*")):
        targets = [inp]
    else:
        targets = sorted(d for d in inp.iterdir()
                         if d.is_dir() and list(d.glob("seed_*")))

    if not targets:
        print(f"[ERROR] no scenario dirs (with seed_N/) found under {inp}")
        sys.exit(1)

    for t in targets:
        if args.dry_run:
            print(f"[dry-run] would analyze: {t}")
        else:
            analyze_scenario(t)


if __name__ == "__main__":
    main()
