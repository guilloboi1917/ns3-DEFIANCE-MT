#!/usr/bin/env python3
"""
evaluate-agent.py — Multi-seed batch evaluation of a TRAINED RL agent policy.

Mirrors run-evaluation.py (A3 baselines) but launches `run-agent infer` per
(seed, scenario) so the trained policy (best_checkpoint) drives the handovers.
Produces per-scenario raw.csv (per-seed) and aggregate.csv (mean/std/p5/p50/p95)
for the thesis tables.

Usage:
    cd /path/to/ns-3-dev
    python3 contrib/defiance/examples/nr-rl-handover/evaluate-agent.py agent-scenarios.yaml
    python3 .../evaluate-agent.py agent-scenarios.yaml --jobs 4 -o results/agent-eval
    # Point at a different checkpoint / run without editing the YAML:
    python3 .../evaluate-agent.py agent-scenarios.yaml -a ~/ray_results/PPO_2026-08-04_.../ -s 10 --sim-time 60

YAML format (see agent-scenarios.yaml):
    evaluation:
      name: agent-eval
      checkpoint: /home/.../SAC_2026-08-03_19-06-55/   # exp dir or best_checkpoint dir
      n_seeds: 5
      sim_time: 30
      common: {transportProtocol: udp, flowDirection: dl}
      scenarios:
        - tag: triangle-no-if
          topology: triangle
          addInterferingUes: 0
        - tag: triangle-4if
          topology: triangle
          addInterferingUes: 4
          aerialUeRatio: 1.0

A run-agent meta.yaml is also accepted directly — the policy is then
    evaluated in the exact environment it was trained in:
        python3 .../evaluate-agent.py ~/ray_results/SAC_2026-08-05_16-17-55/meta.yaml
    (checkpoint defaults to the run's best_checkpoint/, sim time to the
    training simDuration; -a/-s/--sim-time still override).

Metrics per seed: goodput (stdout + rl_reward.csv mean), handovers, ping-pong,
RLF, RTT, SINR (dl_sinr.csv), episode reward + per-step action stats
(rl_actions_full.csv: executed / blocked-same-cell / noop, action distribution).
"""

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


# ---------------------------------------------------------------------------
#   Constants
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[4]  # ns-3-dev
RESULTS_DIR = Path("results")
ENV_NAME = "defiance-nr-rl-handover"


# ---------------------------------------------------------------------------
#   Command construction
# ---------------------------------------------------------------------------

# Settings that are per-worker / training-only in a run-agent meta.yaml and
# must not leak into the evaluation scenario (build_infer_cmd forces its own
# values for most of these).
_TRAINING_ONLY_KEYS = frozenset({
    "parallel", "seed", "runId", "trial_name", "outputDir", "logging",
    "rlMode", "handoverAlgorithm", "useActionMasking",
    "visualize", "statsDir",
})


# Settings that change the reward or observation semantics. If the eval
# config does not forward them, the ns-3 default silently applies, which can
# differ from the checkpoint's training config. (rlMode/handoverAlgorithm are
# excluded: build_infer_cmd() force-sets them to true/agent for the eval.)
_REWARD_ENV_KEYS = frozenset({
    "rlRewardComposition", "rlBetaHandover", "rlAlphaGoodput",
    "rlBetaGoodput", "useTbsObservation", "stepTime", "handoverMargin",
    "rlPingPongMultiplier", "rlHandoverRatePenalty", "rlHandoverRateWindowMs",
    "rlHandoverRateBudget", "rlHandoverRateLambda", "rlTopN",
    "rlHandoverDebounceMs", "rlRewardRefMbps", "rlRewardGoodputShape",
    "rlRewardGoodputAlpha", "rlRewardGoodputP",
})


def _find_training_meta(checkpoint: Path) -> Path | None:
    """Locate the run meta.yaml from a checkpoint path (exp dir,
    best_checkpoint dir, or a checkpoint_N dir) by walking up."""
    if checkpoint.is_file():
        checkpoint = checkpoint.parent
    for p in (checkpoint, *checkpoint.parents):
        meta = p / "meta.yaml"
        if meta.exists():
            return meta
    return None


def _warn_reward_env_mismatch(checkpoint: str, common: dict, scenarios: list) -> None:
    """Warn if reward/obs-affecting settings from the training config are
    absent from the eval config (the ns-3 default would then be used)."""
    meta = _find_training_meta(Path(checkpoint))
    if meta is None:
        return
    try:
        with open(meta) as f:
            training = (yaml.safe_load(f) or {}).get("ns3_settings", {})
    except Exception:
        return
    if not training:
        return
    forwarded = {**common}
    for sc in scenarios:
        forwarded.update({k: v for k, v in sc.items() if k != "tag"})
    for k in sorted(_REWARD_ENV_KEYS):
        if k not in forwarded and k in training:
            print(f"[warn] `{k}` not in the eval config — ns-3 default applies; "
                  f"training used {k}={training[k]} (see the checkpoint run's "
                  f"meta.yaml). Add it to `common:` if the reward/obs must "
                  f"match training.")


def _training_policy_settings(checkpoint: str) -> dict:
    """Policy-semantics settings (reward/obs-affecting) from the checkpoint's
    training run meta.yaml. Used as the BASE for the eval env so the reward
    matches training; the scenario matrix then only varies the
    scenario-defining keys (topology, interference, heights, mobility, ...).
    Returns {} when the training meta cannot be found or read."""
    meta = _find_training_meta(Path(checkpoint))
    if meta is None:
        return {}
    try:
        with open(meta) as f:
            training = (yaml.safe_load(f) or {}).get("ns3_settings", {})
    except Exception:
        return {}
    return {k: v for k, v in training.items() if k in _REWARD_ENV_KEYS}


def _eval_cfg_from_run_meta(yaml_path: Path, config: dict) -> dict | None:
    """Convert a run-agent meta.yaml into an evaluation config dict.

    Detects the run-agent schema (has both `type` and `ns3_settings`).
    Returns None for regular evaluate-agent scenario YAMLs. The converted
    config evaluates the policy in the same environment it was trained in
    (topology, interference, transport, stepTime, rlRewardComposition,
    useTbsObservation, ...), defaulting the checkpoint to the run's
    best_checkpoint and the sim time to the training duration.
    """
    if "ns3_settings" not in config or "type" not in config:
        return None

    # Locate the run root: meta.yaml lives in the trial dir and in the
    # experiment root; walk up until a best_checkpoint / experiment_state
    # marks the run root.
    run_root = yaml_path.resolve().parent
    for p in (run_root, *run_root.parents):
        if (p / "best_checkpoint").exists() or list(p.glob("experiment_state-*.json")):
            run_root = p
            break

    ns3 = {k: v for k, v in config.get("ns3_settings", {}).items()
           if k not in _TRAINING_ONLY_KEYS}
    sim_time = float(ns3.pop("simDuration", 30))
    tag = re.sub(r"[^A-Za-z0-9_-]+", "-", run_root.name) or "run"
    checkpoint = (str(run_root / "best_checkpoint")
                  if (run_root / "best_checkpoint").exists() else None)
    return {
        "name": tag,
        "checkpoint": checkpoint,
        "sim_time": sim_time,
        "common": {},
        "scenarios": [{"tag": tag, **ns3}],
    }


def build_infer_cmd(checkpoint, scenario, common, seed, output_dir, run_agent, sim_time):
    """Build the `run-agent infer` command for one (scenario, seed)."""
    params = dict(common)
    params.update(scenario)
    params.pop("tag", None)       # scenario metadata, not an ns-3 setting
    params["rlMode"] = "true"
    params["handoverAlgorithm"] = "agent"
    params["logging"] = "true"
    params["parallel"] = "0"          # keep seed == --seed exactly
    params["seed"] = str(seed)
    params["runId"] = "1"
    params["simDuration"] = str(sim_time)
    # Unique shared-memory segment per run: segName = ns3-ai_<trial_name>.
    # Concurrent --jobs runs MUST NOT share the default "inference" segment.
    params["trial_name"] = f"eval-{scenario.get('tag', 'run')}-s{seed}"
    params["outputDir"] = str(output_dir)

    settings = [f"{k}={v}" for k, v in params.items()]
    # NOTE: `-c` is nargs="*" (ParseKwargs) — each k=v MUST be its own argv
    # element. A single joined string would be split on "=" into N parts and
    # crash ParseKwargs ("dictionary update sequence element has length N").
    return [run_agent, "infer", "-n", ENV_NAME, "-a", checkpoint, "-c", *settings]


# ---------------------------------------------------------------------------
#   Parsers (current scenario outputs only)
# ---------------------------------------------------------------------------

def parse_stdout(stdout: str) -> dict:
    """Extract end-of-sim metrics from stdout."""
    result = {}
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("Simulation time:"):
            result["simTime"] = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Total handovers:"):
            result["handovers_stdout"] = int(line.split(":")[1].strip())
        elif line.startswith("Total received:"):
            # "Total received: 126.056 Mbit"
            result["mbit"] = float(line.split(":")[1].strip().split()[0])
        elif line.startswith("Total RLF:"):
            result["rlfCount"] = int(line.split(":")[1].strip())
        elif line.startswith("Average RTT:"):
            result["rttMs_stdout"] = float(line.split(":")[1].strip().split()[0])
    return result


def parse_reward(path: Path, sim_time: float) -> dict:
    """Parse rl_reward.csv.

    Columns: time,goodputMbps,dynamicRef,dynamicMin,normG_raw,normG,
             R_G,I_ho,R_H,pingPong,reward
    Returns per-step mean goodput, total episode reward, handover steps,
    ping-pong steps, and goodput from sink bytes (Mbit).
    """
    if not path.exists():
        return {}
    df = pd.read_csv(path, header=None)
    df.columns = ["t", "goodput", "ref", "min", "normG_raw", "normG",
                  "R_G", "I_ho", "R_H", "pingPong", "reward"][:df.shape[1]]
    if len(df) == 0:
        return {}
    return {
        "reward_total": float(df["reward"].sum()),
        "reward_mean": float(df["reward"].mean()),
        "goodputMbps_obs_mean": float(df["goodput"].mean()),
        "handover_steps": int(df["I_ho"].sum()),
        "pingpong_steps": int(df["pingPong"].sum()),
        "steps": int(len(df)),
    }


def parse_actions(path: Path) -> dict:
    """Parse rl_actions_full.csv (time,actionIndex,targetCellId,currentCellId,outcome)."""
    if not path.exists():
        return {}
    df = pd.read_csv(path, header=None,
                     names=["t", "a", "target", "current", "outcome"])
    if len(df) == 0:
        return {}
    out = df["outcome"].value_counts().to_dict()
    dist = df["a"].value_counts().sort_index()
    return {
        "actions_executed": int(out.get("executed", 0)),
        "actions_blocked_same_cell": int(out.get("blocked-same-cell", 0)),
        "actions_blocked_in_progress": int(out.get("blocked-in-progress", 0)),
        "actions_blocked_debounce": int(out.get("blocked-debounce", 0)),
        "actions_noop": int(out.get("noop", 0)),
        "action_dist": {int(k): int(v) for k, v in dist.items()},
    }


def parse_dl_sinr(path: Path) -> np.ndarray:
    """dl_sinr.csv: time,cellId,rnti,sinrDb (serving cell only)."""
    if not path.exists():
        return np.array([])
    df = pd.read_csv(path, header=None, names=["time", "cellId", "rnti", "sinrDb"])
    return df["sinrDb"].to_numpy()


def parse_handovers(path: Path) -> tuple:
    """nr-rl-handovers.csv: time,cellId -> (count, times, cells)."""
    if not path.exists():
        return 0, np.array([]), np.array([])
    df = pd.read_csv(path, header=None, names=["time", "cellId"])
    return len(df), df["time"].to_numpy(), df["cellId"].to_numpy()


def count_ping_pong(times: np.ndarray, cells: np.ndarray, window_s: float = 2.0) -> int:
    """A->B->A within window_s: cell[t] == cell[t+2] != cell[t+1]."""
    if len(times) < 3:
        return 0
    mask = (
        (cells[:-2] == cells[2:])
        & (cells[:-2] != cells[1:-1])
        & (times[2:] - times[:-2] < window_s)
    )
    return int(mask.sum())


def parse_meta(path: Path) -> dict:
    """meta.yaml — parameters snapshot for reproducibility."""
    if not path.exists():
        return {}
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


# ---------------------------------------------------------------------------
#   Aggregation / CSV writing
# ---------------------------------------------------------------------------

def aggregate_results(rows: list) -> dict:
    if not rows:
        return {}
    metrics = [k for k in rows[0].keys() if k not in ("seed", "action_dist", "error")]
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


UNIT_MAP = {
    "goodputMbps": "Mbps",
    "goodputMbps_obs_mean": "Mbps",
    "reward_total": "score",
    "reward_mean": "score",
    "handovers": "count",
    "handovers_completed": "count",
    "handover_steps": "steps",
    "pingPongCount": "count",
    "pingpong_steps": "steps",
    "rlfCount": "count",
    "rttMs_avg": "ms",
    "sinrDb_avg": "dB",
    "sinrDb_p50": "dB",
    "actions_executed": "count",
    "actions_blocked_same_cell": "count",
    "actions_blocked_in_progress": "count",
    "actions_blocked_debounce": "count",
    "actions_noop": "count",
    "steps": "count",
    "simTime": "s",
}

RAW_FIELDS = [
    "seed", "goodputMbps", "goodputMbps_obs_mean", "reward_total", "reward_mean",
    "handovers", "handovers_completed", "handover_steps",
    "pingPongCount", "pingpong_steps",
    "rlfCount", "rttMs_avg", "sinrDb_avg", "sinrDb_p50",
    "actions_executed", "actions_blocked_same_cell", "actions_blocked_in_progress",
    "actions_blocked_debounce",
    "actions_noop", "steps", "simTime",
]


def write_raw_csv(path: Path, rows: list):
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RAW_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_aggregate_csv(path: Path, aggregate: dict):
    if not aggregate:
        return
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "mean", "std", "p5", "p50", "p95", "unit"])
        for metric, stats in sorted(aggregate.items()):
            unit = UNIT_MAP.get(metric, "")
            writer.writerow([
                metric,
                f"{stats['mean']:.4f}", f"{stats['std']:.4f}",
                f"{stats['p5']:.4f}", f"{stats['p50']:.4f}",
                f"{stats['p95']:.4f}", unit,
            ])


# ---------------------------------------------------------------------------
#   Per-seed execution
# ---------------------------------------------------------------------------

def run_one_seed(scenario, common, checkpoint, seed, tag_dir, sim_time,
                 run_agent, timeout):
    seed_dir = tag_dir / f"seed_{seed}"
    seed_dir.mkdir(parents=True, exist_ok=True)

    cmd = build_infer_cmd(checkpoint, scenario, common, seed, seed_dir, run_agent, sim_time)

    env = os.environ.copy()
    env.setdefault("NS3_HOME", str(REPO_ROOT))

    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
            cwd=REPO_ROOT, env=env,
        )
    except subprocess.TimeoutExpired:
        return {"seed": seed, "error": "TIMEOUT"}
    except FileNotFoundError:
        return {"seed": seed, "error": f"run-agent not found: {run_agent}"}

    if proc.returncode != 0:
        stderr_preview = proc.stderr.strip()[-400:] if proc.stderr else "(none)"
        return {"seed": seed, "error": f"FAILED rc={proc.returncode}: {stderr_preview}"}

    out = parse_stdout(proc.stdout)
    rew = parse_reward(seed_dir / "rl_reward.csv", sim_time)
    act = parse_actions(seed_dir / "rl_actions_full.csv")
    sinr = parse_dl_sinr(seed_dir / "dl_sinr.csv")
    # Handover metrics. In RL mode `g_totalHandovers` (stdout) counts act-app
    # handover REQUESTS; nr-rl-handovers.csv counts COMPLETIONS (HandoverEndOk),
    # which can be 0 when the policy re-triggers before/without the procedure
    # completing — a finding in itself, not a parse failure.
    ho_req = out.get("handovers_stdout", 0) or act.get("actions_executed", 0)
    ho_count, ho_times, ho_cells = parse_handovers(seed_dir / "nr-rl-handovers.csv")
    ping_pong = count_ping_pong(ho_times, ho_cells)

    goodput_mbps = out.get("mbit", 0.0) / sim_time if sim_time > 0 else 0.0

    row = {
        "seed": seed,
        "goodputMbps": round(goodput_mbps, 4),
        "goodputMbps_obs_mean": round(rew.get("goodputMbps_obs_mean", 0.0), 4),
        "reward_total": round(rew.get("reward_total", 0.0), 4),
        "reward_mean": round(rew.get("reward_mean", 0.0), 4),
        "handovers": ho_req,
        "handovers_completed": ho_count,
        "handover_steps": rew.get("handover_steps", 0),
        "pingPongCount": ping_pong,
        "pingpong_steps": rew.get("pingpong_steps", 0),
        "rlfCount": out.get("rlfCount", 0),
        "rttMs_avg": round(out.get("rttMs_stdout", 0.0), 2),
        "sinrDb_avg": round(float(np.mean(sinr)), 2) if sinr.size else 0.0,
        "sinrDb_p50": round(float(np.median(sinr)), 2) if sinr.size else 0.0,
        "actions_executed": act.get("actions_executed", 0),
        "actions_blocked_same_cell": act.get("actions_blocked_same_cell", 0),
        "actions_blocked_in_progress": act.get("actions_blocked_in_progress", 0),
        "actions_blocked_debounce": act.get("actions_blocked_debounce", 0),
        "actions_noop": act.get("actions_noop", 0),
        "steps": rew.get("steps", 0),
        "simTime": round(out.get("simTime", 0.0), 4),
        "action_dist": act.get("action_dist", {}),
    }
    return row


# ---------------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Multi-seed batch evaluation of a trained RL agent policy"
    )
    parser.add_argument("scenarios_yaml", type=str,
                        help="Path to YAML scenario matrix, a run-agent meta.yaml, "
                        "or a run/experiment directory (its meta.yaml is used "
                        "automatically)")
    parser.add_argument("--checkpoint", "-a", type=str, default=None,
                        help="Checkpoint path override: an experiment dir (auto-appends "
                        "best_checkpoint/) or a direct checkpoint dir (e.g. "
                        ".../checkpoint_000030). Overrides the `checkpoint` field in "
                        "the YAML. Accepts ~ expansion.")
    parser.add_argument("--n-seeds", "-s", type=int, default=None,
                        help="Override the number of seeds from the YAML.")
    parser.add_argument("--sim-time", type=float, default=None,
                        help="Override simDuration (s) from the YAML.")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output directory (default: ./results/<name>)")
    parser.add_argument("--jobs", "-j", type=int, default=1,
                        help="Seeds to run concurrently (default: 1)")
    parser.add_argument("--timeout", type=float, default=1200.0,
                        help="Per-seed timeout in seconds (default 1200)")
    parser.add_argument("--run-agent", type=str, default=None,
                        help="Path to the run-agent binary (default: from PATH)")
    parser.add_argument("--dry-run", "-n", action="store_true",
                        help="Print commands without executing")
    args = parser.parse_args()

    run_agent = args.run_agent or shutil.which("run-agent")
    if not run_agent:
        print("[ERROR] `run-agent` not found on PATH. Activate the ns-defiance "
              "poetry env (eval $(poetry env activate)) or pass --run-agent.")
        sys.exit(1)

    yaml_path = Path(args.scenarios_yaml)
    if yaml_path.is_dir():
        # Accept a run/experiment directory directly (the run's meta.yaml is
        # written to both the experiment root and the trial dir).
        meta = yaml_path / "meta.yaml"
        if not meta.exists():
            trials = sorted(yaml_path.glob("*/meta.yaml"))
            meta = trials[0] if trials else None
        if meta is not None:
            print(f"[info] {yaml_path} is a run directory — using {meta.name} from it")
            yaml_path = meta
        else:
            print(f"[ERROR] No meta.yaml found in directory: {yaml_path}")
            sys.exit(1)
    elif not yaml_path.exists():
        print(f"[ERROR] Scenario file not found: {yaml_path}")
        sys.exit(1)
    with open(yaml_path) as f:
        config = yaml.safe_load(f)

    # Accept a run-agent meta.yaml directly: evaluate the policy in its own
    # training environment (checkpoint defaults to the run's best_checkpoint,
    # sim time to the training duration).
    run_meta_cfg = _eval_cfg_from_run_meta(yaml_path, config)
    if run_meta_cfg is not None:
        print(f"[info] {yaml_path.name} is a run-agent meta.yaml — evaluating "
              f"the policy in its training environment.")
        config = run_meta_cfg

    eval_cfg = config.get("evaluation", config)
    eval_name = eval_cfg.get("name", "agent-evaluation")
    checkpoint = args.checkpoint or eval_cfg.get("checkpoint")
    n_seeds = args.n_seeds or eval_cfg.get("n_seeds", 5)
    sim_time = args.sim_time or eval_cfg.get("sim_time", 30)
    scenarios = eval_cfg.get("scenarios", [])

    if not checkpoint:
        print("[ERROR] `checkpoint` not set in YAML")
        sys.exit(1)
    checkpoint = os.path.expanduser(checkpoint)
    if not os.path.exists(checkpoint):
        print(f"[ERROR] Checkpoint not found: {checkpoint}")
        sys.exit(1)

    # Base the eval environment on the checkpoint's training settings for all
    # reward/obs-affecting keys (reward composition, betaHandover, stepTime,
    # useTbsObservation, ...) so the reward matches training; the scenario
    # matrix then only varies the scenario-defining keys. Explicit `common:`
    # values still override the inherited base.
    if run_meta_cfg is None:
        training_policy = _training_policy_settings(checkpoint)
        if training_policy:
            print(f"[info] inherited {len(training_policy)} reward/obs settings from "
                  f"the checkpoint run's meta.yaml: {training_policy}")
        common = {**training_policy, **eval_cfg.get("common", {})}
        if args.sim_time is None and "sim_time" not in eval_cfg:
            meta_sim = _find_training_meta(Path(checkpoint))
            if meta_sim is not None:
                try:
                    with open(meta_sim) as f:
                        td = (yaml.safe_load(f) or {}).get("ns3_settings", {})
                    if "simDuration" in td:
                        sim_time = float(td["simDuration"])
                        print(f"[info] sim_time defaults to the training duration "
                              f"({sim_time:.0f}s) — override with --sim-time")
                except Exception:
                    pass
    else:
        common = eval_cfg.get("common", {})

    # Guard: warn when a reward/obs-affecting setting present in the
    # checkpoint's training config (meta.yaml) is NOT forwarded by the eval
    # config — the ns-3 default would then apply, silently changing the reward
    # scale (e.g. additive vs multiplicative, betaHandover). With the
    # inheritance above this fires only when the training meta is missing.
    if run_meta_cfg is None:
        _warn_reward_env_mismatch(checkpoint, common, scenarios)

    if not scenarios:
        print("[ERROR] No scenarios defined in YAML")
        sys.exit(1)

    if args.output:
        base_dir = Path(args.output).resolve()
    else:
        base_dir = (RESULTS_DIR / eval_name).resolve()

    print(f"Agent evaluation: {eval_name}")
    print(f"  Checkpoint: {checkpoint}")
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
        tag_dir = base_dir / tag
        tag_dir.mkdir(parents=True, exist_ok=True)

        params_snapshot = dict(common)
        params_snapshot.update(sc)
        params_snapshot["checkpoint"] = checkpoint
        params_snapshot["n_seeds"] = n_seeds
        params_snapshot["sim_time"] = sim_time
        with open(tag_dir / "params.yaml", "w") as f:
            yaml.dump(params_snapshot, f, default_flow_style=False)

        print(f"[{tag}] Running {n_seeds} seeds with {args.jobs} workers...")
        rows = []
        fail_scenario = 0

        def _run(seed):
            return run_one_seed(sc, common, checkpoint, seed, tag_dir,
                                sim_time, run_agent, args.timeout)

        if not args.dry_run and args.jobs > 1:
            with ThreadPoolExecutor(max_workers=args.jobs) as executor:
                futures = {executor.submit(_run, s): s for s in range(1, n_seeds + 1)}
                for future in as_completed(futures):
                    seed = futures[future]
                    try:
                        result = future.result()
                    except Exception as e:  # noqa: BLE001
                        print(f"  seed={seed}: EXCEPTION: {e}")
                        fail_scenario += 1
                        continue
                    if "error" in result:
                        print(f"  seed={seed}: {result['error']}")
                        fail_scenario += 1
                    else:
                        rows.append(result)
                    run_count += 1
                    elapsed = time.time() - start_wall
                    print(f"  [{run_count}/{total_runs} {run_count/total_runs*100:.0f}%] "
                          f"seed={seed}, {elapsed:.0f}s elapsed")
        else:
            for seed in range(1, n_seeds + 1):
                if args.dry_run:
                    seed_dir = tag_dir / f"seed_{seed}"
                    cmd = build_infer_cmd(checkpoint, sc, common, seed, seed_dir, run_agent, sim_time)
                    print(f"  seed={seed}: {' '.join(cmd)}")
                    continue
                result = _run(seed)
                if "error" in result:
                    print(f"  seed={seed}: {result['error']}")
                    fail_scenario += 1
                else:
                    rows.append(result)
                run_count += 1
                elapsed = time.time() - start_wall
                print(f"  [{run_count}/{total_runs} {run_count/total_runs*100:.0f}%] "
                      f"seed={seed}, {elapsed:.0f}s elapsed")

        fail_count += fail_scenario
        if args.dry_run:
            print("  -> (dry-run, skipped)")
            continue

        if rows:
            write_raw_csv(tag_dir / "raw.csv", rows)
            aggregate = aggregate_results(rows)
            write_aggregate_csv(tag_dir / "aggregate.csv", aggregate)

            g = aggregate.get("goodputMbps", {})
            h = aggregate.get("handovers", {})
            r = aggregate.get("reward_total", {})
            rlf = aggregate.get("rlfCount", {})
            # action distribution across seeds (all actions ever taken)
            all_actions = Counter()
            for row in rows:
                all_actions.update(row.get("action_dist", {}))
            dist_str = ", ".join(f"{a}:{c}" for a, c in sorted(all_actions.items()))
            print(f"  -> goodput: p50={g.get('p50','?'):>6.2f} "
                  f"(p5={g.get('p5','?'):>6.2f} p95={g.get('p95','?'):>6.2f}) Mbps | "
                  f"handovers: p50={h.get('p50','?'):>5.1f} | "
                  f"reward: p50={r.get('p50','?'):>8.1f} | RLF: p50={rlf.get('p50','?'):>4.1f}")
            print(f"  -> action distribution (all seeds): {dist_str or '(none)'}")
        else:
            print("  -> No successful runs")
        print()

    elapsed = time.time() - start_wall
    print(f"Done. {run_count} runs, {fail_count} failed, "
          f"{elapsed:.0f}s elapsed ({elapsed/60:.1f} min)")
    print(f"Results: {base_dir.resolve()}")


if __name__ == "__main__":
    main()
