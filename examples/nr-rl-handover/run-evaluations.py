#!/usr/bin/env python3
"""run-evaluations.py - run a scenario matrix x seeds in parallel.

Each (scenario, seed) writes results/<name>/<tag>/seed_N/.
Analysis is separate: analyze-evaluations.py.

Usage: python3 run-evaluations.py <matrix.yaml> [--jobs N] [--analyze] [--dry-run]
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]   # ns-3-dev root
RESULTS_DIR = Path("results")
NS3_BIN = "defiance-nr-rl-handover"
ENV_NAME = "defiance-nr-rl-handover"

# Run-agent / sim keys that are set by the runner itself, never by the YAML.
RUNNER_KEYS = frozenset({
    "tag", "mode", "algorithm", "checkpoint", "n_seeds", "sim_time",
    "simDuration", "seed", "runId", "trial_name", "outputDir", "logging",
    "parallel", "rlMode", "visualize", "statsDir",
})

# Reward/obs-affecting settings that must match the checkpoint's training run.
# Forwarded from the training meta.yaml to the eval command (explicit `common:`
# values override). Keep in sync whenever a reward/obs param is added.
REWARD_ENV_KEYS = frozenset({
    "rlRewardComposition", "rlBetaHandover", "rlAlphaGoodput",
    "rlBetaGoodput", "stepTime", "rlPingPongMultiplier", "rlPingPongWindowMs",
    "rlHandoverHangoverLength", "rlHandoverRateWindowMs", "rlRewardRefMbps",
    "rlRewardGoodputShape", "rlRewardGoodputAlpha", "rlRewardGoodputP",
    "obsStackFrames",
})

# Defaults matching the current campaign (results dirs / meta.yaml as ground
# truth); YAML `common:`/scenario values override these.
NS3_DEFAULTS = {
    "topology": "triangle",
    "flowDirection": "ul",
    "uavMobility": "random-waypoint",
    "tcpVariant": "TcpBbr",
    "startHeight": 50,
    "endHeight": 200,
    "addInterferingUes": 0,
    "aerialUeRatio": 0.0,
    "interfererMobility": "static",
    "transportProtocol": "udp",
    "bandwidthMhz": 30,
    "rlRewardRefMbps": 35.0,
}


# ---------------------------------------------------------------------------
# Checkpoint / training-meta helpers (RL mode)
# ---------------------------------------------------------------------------

def find_training_meta(checkpoint: str) -> Path | None:
    """Locate the run meta.yaml from a checkpoint path (experiment dir,
    best_checkpoint dir, or a checkpoint_N dir) by walking up."""
    p = Path(checkpoint)
    if p.is_file():
        p = p.parent
    for q in (p, *p.parents):
        meta = q / "meta.yaml"
        if meta.exists():
            return meta
    return None


def training_policy_settings(checkpoint: str) -> dict:
    """Reward/obs-affecting settings from the checkpoint's training meta.yaml,
    used as the BASE for the eval environment so the reward matches training.
    Returns {} when the training meta cannot be read."""
    meta = find_training_meta(checkpoint)
    if meta is None:
        return {}
    try:
        with open(meta) as f:
            training = (yaml.safe_load(f) or {}).get("ns3_settings", {})
    except Exception:
        return {}
    return {k: v for k, v in training.items() if k in REWARD_ENV_KEYS}


def training_sim_time(checkpoint: str) -> float | None:
    """The training simDuration from the checkpoint's meta.yaml, if readable."""
    meta = find_training_meta(checkpoint)
    if meta is None:
        return None
    try:
        with open(meta) as f:
            td = (yaml.safe_load(f) or {}).get("ns3_settings", {})
        return float(td["simDuration"])
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Command builders
# ---------------------------------------------------------------------------

def _settings(scenario: dict, common: dict) -> dict:
    """Merge common + scenario into one ns-3 settings dict (meta keys removed)."""
    params = dict(NS3_DEFAULTS)
    params.update(common)
    params.update(scenario)
    for k in RUNNER_KEYS:
        params.pop(k, None)
    return params


def build_ns3_cmd(scenario: dict, common: dict, sim_time: float, seed: int,
                  output_dir: Path) -> list:
    """Build the `ns3 run` command for one A3-baseline seed."""
    params = _settings(scenario, common)
    algo = params.pop("handoverAlgorithm", "a3")
    args = f"{NS3_BIN}"
    args += f" --simDuration={sim_time}"
    args += f" --seed={seed}"
    args += " --runId=1"
    for k, v in params.items():
        args += f" --{k}={v}"
    args += " --logging=true"
    args += " --parallel=0"
    args += " --rlMode=false"
    args += f" --handoverAlgorithm={algo}"
    args += f" --outputDir={output_dir}"
    return ["ns3", "run", args]


def build_infer_cmd(scenario: dict, common: dict, checkpoint: str, seed: int,
                    output_dir: Path, run_agent: str, sim_time: float) -> list:
    """Build the `run-agent infer` command for one RL-policy seed."""
    params = _settings(scenario, common)
    params["rlMode"] = "true"
    params["handoverAlgorithm"] = "agent"
    params["logging"] = "true"
    params["parallel"] = "0"
    params["seed"] = str(seed)
    params["runId"] = "1"
    params["simDuration"] = str(sim_time)
    params["trial_name"] = f"eval-{scenario.get('tag', 'run')}-s{seed}"
    params["outputDir"] = str(output_dir)
    settings = [f"{k}={v}" for k, v in params.items()]
    return [run_agent, "infer", "-n", ENV_NAME, "-a", checkpoint, "-c", *settings]


# ---------------------------------------------------------------------------
# Per-seed execution
# ---------------------------------------------------------------------------

def run_one_seed(mode: str, scenario: dict, common: dict, checkpoint: str | None,
                 seed: int, tag_dir: Path, sim_time: float, run_agent: str,
                 timeout: float) -> dict:
    """Run one seed, write run-info.yaml with walltime, return {seed} or
    {seed, error}."""
    seed_dir = tag_dir / f"seed_{seed}"
    seed_dir.mkdir(parents=True, exist_ok=True)

    if mode == "rl":
        cmd = build_infer_cmd(scenario, common, checkpoint, seed, seed_dir,
                              run_agent, sim_time)
        env = os.environ.copy()
        env.setdefault("NS3_HOME", str(REPO_ROOT))
        cwd = REPO_ROOT
    else:
        cmd = build_ns3_cmd(scenario, common, sim_time, seed, seed_dir)
        cwd, env = None, None

    start = time.perf_counter()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout, cwd=cwd, env=env)
    except subprocess.TimeoutExpired:
        return {"seed": seed, "error": "TIMEOUT"}
    except FileNotFoundError:
        return {"seed": seed, "error": f"command not found: {cmd[0]}"}
    walltime = time.perf_counter() - start

    if proc.returncode != 0:
        stderr_preview = (proc.stderr or "").strip()[-400:] or "(none)"
        error = f"FAILED rc={proc.returncode}: {stderr_preview}"
        with open(seed_dir / "run-info.yaml", "w") as f:
            yaml.dump({"seed": seed, "mode": mode, "status": "failed",
                       "walltime_s": round(walltime, 3), "error": error},
                      f, default_flow_style=False)
        return {"seed": seed, "error": error}

    info = {
        "seed": seed,
        "mode": mode,
        "status": "ok",
        "walltime_s": round(walltime, 3),
        "command": [str(c) for c in cmd],
    }
    with open(seed_dir / "run-info.yaml", "w") as f:
        yaml.dump(info, f, default_flow_style=False)
    return {"seed": seed}


# ---------------------------------------------------------------------------
# Matrix loading
# ---------------------------------------------------------------------------

def eval_cfg_from_run_meta(yaml_path: Path, config: dict) -> dict | None:
    """Convert a run-agent meta.yaml into an evaluation config (RL mode in the
    training environment). Returns None for regular scenario-matrix YAMLs."""
    if "ns3_settings" not in config or "type" not in config:
        return None
    run_root = yaml_path.resolve().parent
    for p in (run_root, *run_root.parents):
        if (p / "best_checkpoint").exists() or list(p.glob("experiment_state-*.json")):
            run_root = p
            break
    ns3 = {k: v for k, v in config.get("ns3_settings", {}).items()
           if k not in RUNNER_KEYS}
    sim_time = float(ns3.pop("simDuration", 30))
    tag = re.sub(r"[^A-Za-z0-9_-]+", "-", run_root.name) or "run"
    checkpoint = (str(run_root / "best_checkpoint")
                  if (run_root / "best_checkpoint").exists() else None)
    return {
        "name": tag,
        "mode": "rl",
        "checkpoint": checkpoint,
        "sim_time": sim_time,
        "common": {},
        "scenarios": [{"tag": tag, **ns3}],
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Unified multi-seed runner (A3 baselines and RL evals) "
                    "for defiance-nr-rl-handover")
    parser.add_argument("matrix_yaml", type=str,
                        help="Scenario-matrix YAML, a run-agent meta.yaml, or a "
                             "run/experiment directory (its meta.yaml is used)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output directory (default: ./results/<name>)")
    parser.add_argument("--jobs", "-j", type=int, default=1,
                        help="Seeds to run concurrently (default: 1)")
    parser.add_argument("--n-seeds", "-s", type=int, default=None,
                        help="Override the number of seeds from the YAML")
    parser.add_argument("--seed-start", type=int, default=1,
                        help="First seed to run (default: 1). Use with --n-seeds "
                             "to add seeds without re-running the existing ones")
    parser.add_argument("--tags", type=str, default=None,
                        help="Comma-separated scenario tags to run (default: all)")
    parser.add_argument("--sim-time", type=float, default=None,
                        help="Override simDuration (s) from the YAML")
    parser.add_argument("--checkpoint", "-a", type=str, default=None,
                        help="Checkpoint override for RL mode (default: YAML field)")
    parser.add_argument("--timeout", type=float, default=1200.0,
                        help="Per-seed timeout in seconds (default 1200)")
    parser.add_argument("--dry-run", "-n", action="store_true",
                        help="Print commands without executing")
    parser.add_argument("--analyze", action="store_true",
                        help="Run analyze-evaluations.py after all seeds finish")
    args = parser.parse_args()

    yaml_path = Path(args.matrix_yaml)
    if yaml_path.is_dir():
        meta = yaml_path / "meta.yaml"
        if not meta.exists():
            trials = sorted(yaml_path.glob("*/meta.yaml"))
            meta = trials[0] if trials else None
        if meta is None:
            print(f"[ERROR] No meta.yaml found in directory: {yaml_path}")
            sys.exit(1)
        yaml_path = meta
    if not yaml_path.exists():
        print(f"[ERROR] Matrix file not found: {yaml_path}")
        sys.exit(1)
    with open(yaml_path) as f:
        config = yaml.safe_load(f)

    run_meta_cfg = eval_cfg_from_run_meta(yaml_path, config)
    if run_meta_cfg is not None:
        print(f"[info] {yaml_path.name} is a run-agent meta.yaml — evaluating "
              f"the policy in its training environment.")
        config = run_meta_cfg

    eval_cfg = config.get("evaluation", config)
    eval_name = eval_cfg.get("name", "evaluation")
    mode = eval_cfg.get("mode", "a3")
    checkpoint = args.checkpoint or eval_cfg.get("checkpoint")
    n_seeds = args.n_seeds or eval_cfg.get("n_seeds", 5)
    seed_start = args.seed_start
    sim_time = args.sim_time or eval_cfg.get("sim_time", 30)
    jobs = args.jobs
    scenarios = eval_cfg.get("scenarios", [])
    if args.tags:
        wanted = {t.strip() for t in args.tags.split(",") if t.strip()}
        scenarios = [s for s in scenarios if s.get("tag") in wanted]
        if not scenarios:
            print(f"[ERROR] No scenario tag matched --tags {args.tags}")
            sys.exit(1)

    if mode == "rl":
        if not checkpoint:
            print("[ERROR] mode=rl requires `checkpoint` in the YAML (or -a)")
            sys.exit(1)
        checkpoint = os.path.expanduser(checkpoint)
        if not os.path.exists(checkpoint) and not args.dry_run:
            print(f"[ERROR] Checkpoint not found: {checkpoint}")
            sys.exit(1)
        run_agent = shutil.which("run-agent")
        if not run_agent and not args.dry_run:
            print("[ERROR] `run-agent` not found on PATH. Activate the "
                  "ns-defiance poetry env or pass --run-agent.")
            sys.exit(1)
        # Base the eval env on the checkpoint's training reward/obs settings;
        # explicit `common:` values override the inherited base.
        inherited = training_policy_settings(checkpoint)
        if inherited:
            print(f"[info] inherited {len(inherited)} reward/obs settings from "
                  f"the checkpoint's training meta.yaml")
        common = {**inherited, **eval_cfg.get("common", {})}
        if args.sim_time is None and "sim_time" not in eval_cfg:
            ts = training_sim_time(checkpoint)
            if ts is not None:
                sim_time = ts
                print(f"[info] sim_time defaults to the training duration "
                      f"({sim_time:.0f}s) — override with --sim-time")
    else:
        run_agent = None
        common = eval_cfg.get("common", {})

    if not scenarios:
        print("[ERROR] No scenarios defined in the matrix")
        sys.exit(1)

    base_dir = (Path(args.output).resolve() if args.output
                else (RESULTS_DIR / eval_name).resolve())
    total_runs = len(scenarios) * max(0, n_seeds - seed_start + 1)
    print(f"Evaluation: {eval_name} (mode={mode}"
          + (f", checkpoint={checkpoint}" if mode == "rl" else "") + ")")
    print(f"  Seeds: {seed_start}..{n_seeds} | Scenarios: {len(scenarios)} | "
          f"Jobs: {jobs} | Output: {base_dir}")

    run_count = fail_count = 0
    start_wall = time.time()

    for sc in scenarios:
        sc_mode = sc.get("mode", mode)
        tag = sc.get("tag", "untagged")
        tag_dir = base_dir / tag

        def _run(seed):
            return run_one_seed(sc_mode, sc, common, checkpoint, seed, tag_dir,
                                sim_time, run_agent if sc_mode == "rl" else "",
                                args.timeout)

        if args.dry_run:
            for seed in range(seed_start, n_seeds + 1):
                seed_dir = tag_dir / f"seed_{seed}"
                cmd = (build_infer_cmd(sc, common, checkpoint, seed, seed_dir,
                                       run_agent, sim_time) if sc_mode == "rl"
                       else build_ns3_cmd(sc, common, sim_time, seed, seed_dir))
                print(f"  seed={seed}: {' '.join(str(c) for c in cmd)}")
            print(f"[{tag}] (dry-run, skipped)")
            continue

        tag_dir.mkdir(parents=True, exist_ok=True)
        with open(tag_dir / "params.yaml", "w") as f:
            yaml.dump({"mode": sc_mode, "n_seeds": n_seeds, "sim_time": sim_time, "checkpoint": checkpoint,
                       **common, **sc}, f, default_flow_style=False)

        if jobs > 1:
            with ThreadPoolExecutor(max_workers=jobs) as executor:
                futures = {executor.submit(
                    _run, s): s for s in range(seed_start, n_seeds + 1)}
                for future in as_completed(futures):
                    seed = futures[future]
                    try:
                        result = future.result()
                    except Exception as e:  # noqa: BLE001
                        print(f"  seed={seed}: EXCEPTION: {e}")
                        fail_count += 1
                        continue
                    if "error" in result:
                        print(f"  seed={seed}: {result['error']}")
                        fail_count += 1
                    else:
                        run_count += 1
                    print(f"  [{run_count + fail_count}/{total_runs} "
                          f"{(run_count + fail_count) / total_runs * 100:.0f}%] "
                          f"seed={seed}, {time.time() - start_wall:.0f}s elapsed")
        else:
            for seed in range(seed_start, n_seeds + 1):
                result = _run(seed)
                if "error" in result:
                    print(f"  seed={seed}: {result['error']}")
                    fail_count += 1
                else:
                    run_count += 1
                print(f"  [{run_count + fail_count}/{total_runs} "
                      f"{(run_count + fail_count) / total_runs * 100:.0f}%] "
                      f"seed={seed}, {time.time() - start_wall:.0f}s elapsed")

    print(f"\nDone: {run_count} succeeded, {fail_count} failed "
          f"({time.time() - start_wall:.0f}s wall)")
    if args.analyze:
        analyze = Path(__file__).with_name("analyze-evaluations.py")
        subprocess.run([sys.executable, str(analyze), str(base_dir)])
    sys.exit(1 if fail_count else 0)


if __name__ == "__main__":
    main()
