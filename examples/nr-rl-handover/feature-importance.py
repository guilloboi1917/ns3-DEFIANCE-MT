#!/usr/bin/env python3
"""Permutation feature importance for a trained handover policy.

Measures how much each observation dimension influences the policy's
action choice. For each feature, the column is randomly shuffled across the
batch and the ACTION CHANGE RATE (fraction of argmax actions that flip,
mean(perturbed_actions != ref_actions)) is recorded. The change rate is used
rather than the mean absolute action-index shift: the Discrete(N) index has no
ordinal meaning for handovers (stay<->handover is more significant than
switching handover targets), so a binary flip measure is the defensible one
(2026-08-11). Features the policy relies on show high importance; unused
features show ~0.

Requires:
  - An old-API-stack policy checkpoint (SAC/DQN/D3QN): either an experiment
    dir under ~/ray_results with best_checkpoint/, or a direct checkpoint dir.
  - An observation CSV (rl_obs.csv) with columns: time, obs[0..N].
    Produced by the obs app when --logging=true.

Usage:
    python3 feature-importance.py \\
        --checkpoint ~/ray_results/SAC_2026-07-31_00-18-10 \\
        --obs-file output/inference-agent/rl_obs.csv \\
        --min-time 1.0 \\
        --repeat 5

Outputs (default: next to --obs-file):
  - feature-importance.csv : feature index, name, importance (sorted desc)
  - feature-importance.png : labeled horizontal bar chart
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# 30-dim Top-N observation layout — must match BuildObservation() in
# nr-rl-handover-obs-app.cc
FEATURE_NAMES = [
    # Final obs layout (32 dims, freeze 2026-08-13), see NR-RL-DESIGN.md §3.
    "serving_rsrp", "serving_rsrq",
    "slot_rsrp_0", "slot_rsrp_1", "slot_rsrp_2",
    "rsrp_delta_0", "rsrp_delta_1", "rsrp_delta_2",
    "dl_sinr", "time_since_ho", "norm_goodput", "ho_count_10s",
    # UL block: serving-cell UL SRS SINR, RB utilization, scheduled-UE count.
    "ul_sinr", "ul_rb_util", "ul_sched_ue",
    # Time-delta block (1 s trends).
    "d_serving_rsrp", "d_serving_sinr", "d_serving_rsrq", "d_norm_goodput", "d_margin_best",
    "d_slot_rsrp_0", "d_slot_rsrp_1", "d_slot_rsrp_2",
    # Heading + trajectory (pos and position 2 s ahead, normalized by ISD).
    "heading_x", "heading_y", "heading_z",
    "pos_x", "pos_y", "pos_z",
    "pos2s_x", "pos2s_y", "pos2s_z",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Permutation feature importance for a trained handover policy.")
    p.add_argument("--checkpoint", required=True,
                   help="experiment dir (~/ray_results/<exp>) or direct checkpoint dir")
    p.add_argument("--obs-file", required=True,
                   help="rl_obs.csv, or a directory containing it "
                        "(e.g. an eval seed dir): columns time, obs[0..N]")
    p.add_argument("--out-dir", default=None,
                   help="output dir for csv/png (default: obs-file's directory)")
    p.add_argument("--min-time", type=float, default=None,
                   help="drop observations before this sim time (s), e.g. 1.0 to skip pre-data blind zone")
    p.add_argument("--max-time", type=float, default=None,
                   help="drop observations after this sim time (s)")
    p.add_argument("--repeat", type=int, default=5,
                   help="number of permutation repeats per feature (averaged)")
    p.add_argument("--limit", type=float, default=1.0,
                   help="fraction of rows to use (random subsample) for speed")
    p.add_argument("--seed", type=int, default=0,
                   help="RNG seed for permutations and subsampling")
    p.add_argument("--batch-size", type=int, default=1024,
                   help="rows per compute_actions call")
    return p.parse_args()


def resolve_policy_dir(checkpoint_arg: str) -> tuple[Path, str]:
    """Resolve checkpoint arg to the per-agent policy/module directory.

    Accepts: <exp>/best_checkpoint, <exp>/<trial>/checkpoint_N, or
    <exp> (auto: best_checkpoint, else newest checkpoint_N).

    Returns (dir, stack): stack == "old" for the policies/<agent> layout
    (Policy.from_checkpoint) or "new" for the RLModule layout
    (learner_group/learner/rl_module/<module>, RLModule.from_checkpoint).
    """
    base = Path(checkpoint_arg).expanduser()
    if not base.exists():
        raise FileNotFoundError(f"checkpoint path does not exist: {base}")

    ckpt = base / "best_checkpoint" if (base / "best_checkpoint").is_dir() else base

    # New RLModule API stack: learner_group/learner/rl_module/<module_id>
    modules = sorted((ckpt / "learner_group" / "learner" / "rl_module").glob("*") \
                     if (ckpt / "learner_group" / "learner" / "rl_module").is_dir() else [])
    if not modules:
        # maybe base is the experiment root: <exp>/<trial>/checkpoint_N
        candidates = sorted(ckpt.glob("*/checkpoint_*"))
        if candidates:
            ckpt = candidates[-1]
            modules = sorted((ckpt / "learner_group" / "learner" / "rl_module").glob("*"))
    if modules:
        module_dir = next(d for d in modules if d.is_dir())
        return module_dir, "new"

    # Old API stack: policies/<agent_id>
    if not (ckpt / "policies").is_dir():
        candidates = sorted(ckpt.glob("*/checkpoint_*"))
        if not candidates:
            raise FileNotFoundError(
                f"no policies/ or learner_group/ dir found under {base} "
                f"(looked in best_checkpoint and */checkpoint_*)")
        ckpt = candidates[-1]
    policies = ckpt / "policies"
    agent_dirs = sorted(d for d in policies.iterdir() if d.is_dir())
    if not agent_dirs:
        raise FileNotFoundError(f"no agent policy dirs under {policies}")
    return agent_dirs[0], "old"


def load_observations(obs_file: str, min_time: float | None, max_time: float | None,
                      limit: float, seed: int, expected_dim: int,
                      names: list[str] = FEATURE_NAMES) -> np.ndarray:
    p = Path(obs_file).expanduser()
    if p.is_dir():
        p = p / "rl_obs.csv"
    if not p.exists():
        raise FileNotFoundError(f"no rl_obs.csv found at {p}")
    df = pd.read_csv(p, skiprows=1, header=None)
    n_obs_cols = df.shape[1] - 1
    if n_obs_cols < expected_dim:
        raise ValueError(
            f"eval obs file {p} has {n_obs_cols} obs dims but the policy expects "
            f"{expected_dim}: the eval likely ran with a stale ns-3 binary "
            f"(rl_obs.csv should have t + {expected_dim} columns). Re-run the "
            f"eval with the current build and retry.")
    if n_obs_cols > expected_dim:
        print(f"WARNING: eval obs file has {n_obs_cols} dims; using the first "
              f"{expected_dim} (policy layout).")
    t = df[0].to_numpy()
    mask = np.ones(len(df), dtype=bool)
    if min_time is not None:
        mask &= t >= min_time
    if max_time is not None:
        mask &= t <= max_time
    df = df[mask]
    if df.empty:
        raise ValueError("no observations left after time filtering")

    if limit < 1.0:
        df = df.sample(frac=limit, random_state=seed)

    obs = df.iloc[:, 1:1 + expected_dim].to_numpy(dtype=np.float32)
    if obs.shape[1] != len(names[:expected_dim]):
        raise ValueError(
            f"expected {len(names[:expected_dim])} obs columns, got {obs.shape[1]}")
    return obs


def main() -> None:
    args = parse_args()

    # ── Load policy ──────────────────────────────────────────────────
    policy_dir, stack = resolve_policy_dir(args.checkpoint)
    print(f"Loading policy from {policy_dir} (stack={stack})")

    if stack == "old":
        from ray.rllib.policy import Policy
        policy = Policy.from_checkpoint(str(policy_dir))

        # Validate the policy's flattened obs dim matches FEATURE_NAMES
        from ray.rllib.models.preprocessors import get_preprocessor
        prep = get_preprocessor(policy.observation_space)(policy.observation_space)
        flat_shape = tuple(prep.shape)
        if flat_shape[0] > len(FEATURE_NAMES):
            print(f"ERROR: policy obs shape {flat_shape} exceeds {len(FEATURE_NAMES)} "
                  f"(FEATURE_NAMES); extend the names list")
            sys.exit(1)
        elif flat_shape != (len(FEATURE_NAMES),):
            print(f"NOTE: policy obs shape {flat_shape} < {len(FEATURE_NAMES)} "
                  f"(FEATURE_NAMES); using the first {flat_shape[0]} names")

        def batched_compute(x: np.ndarray) -> np.ndarray:
            """Return argmax/exploit actions for a batch of flat obs."""
            outs = []
            for s in range(0, len(x), args.batch_size):
                acts = policy.compute_actions(x[s:s + args.batch_size], explore=False)[0]
                outs.append(np.asarray(acts))
            return np.concatenate(outs)
    else:
        # New RLModule API stack (e.g. PPO default): forward -> logits -> argmax.
        import torch
        from ray.rllib.core.rl_module.rl_module import RLModule
        module = RLModule.from_checkpoint(str(policy_dir))
        obs_space = getattr(module, "observation_space", None)
        if obs_space is None:
            import pickle
            d = pickle.load(open(policy_dir / "class_and_ctor_args.pkl", "rb"))
            obs_space = d["ctor_args_and_kwargs"][1]["observation_space"]
        flat_shape = tuple(int(s) for s in obs_space.shape)
        if flat_shape[0] > len(FEATURE_NAMES):
            print(f"ERROR: policy obs shape {flat_shape} exceeds {len(FEATURE_NAMES)} "
                  f"(FEATURE_NAMES); extend the names list")
            sys.exit(1)
        elif flat_shape != (len(FEATURE_NAMES),):
            print(f"NOTE: policy obs shape {flat_shape} < {len(FEATURE_NAMES)} "
                  f"(FEATURE_NAMES); using the first {flat_shape[0]} names")

        def batched_compute(x: np.ndarray) -> np.ndarray:
            """Return argmax actions for a batch of flat obs (module forward)."""
            outs = []
            for s in range(0, len(x), args.batch_size):
                batch = x[s:s + args.batch_size]
                with torch.no_grad():
                    out = module.forward({"obs": torch.from_numpy(batch).float()})
                logits = out["action_dist_inputs"].cpu().numpy()
                outs.append(np.asarray(logits).argmax(-1))
            return np.concatenate(outs)

    obs_dim = flat_shape[0]

    # ── Load observations ───────────────────────────────────────────
    obs = load_observations(args.obs_file, args.min_time, args.max_time,
                            args.limit, args.seed, expected_dim=obs_dim,
                            names=FEATURE_NAMES[:obs_dim])
    print(f"Loaded {len(obs)} observations "
          f"({args.min_time if args.min_time is not None else 'start'}.."
          f"{args.max_time if args.max_time is not None else 'end'} s)")

    np.random.seed(args.seed)

    # ── Reference actions ───────────────────────────────────────────
    ref = batched_compute(obs).astype(np.int64)
    print(f"Reference action distribution: "
          f"{dict(zip(*np.unique(ref, return_counts=True)))}")

    # ── Permutation importance ──────────────────────────────────────
    # Action CHANGE RATE per feature: fraction of argmax decisions that flip
    # when the feature column is shuffled (binary — the Discrete(N) action
    # index is not an interval scale for handovers).
    n_feat = obs.shape[1]
    importance = np.zeros((args.repeat, n_feat))
    for r in range(args.repeat):
        for i in range(n_feat):
            x = obs.copy()
            perm = np.random.permutation(len(x))
            x[:, i] = x[perm, i]
            act = batched_compute(x).astype(np.int64)
            importance[r, i] = np.mean(act != ref)
        print(f"repeat {r + 1}/{args.repeat} done")

    mean_imp = importance.mean(axis=0)
    std_imp = importance.std(axis=0)

    # ── Outputs ─────────────────────────────────────────────────────
    obs_path = Path(args.obs_file).expanduser()
    if obs_path.is_dir():
        obs_path = obs_path / "rl_obs.csv"
    out_dir = Path(args.out_dir) if args.out_dir else obs_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    order = np.argsort(mean_imp)[::-1]
    rows = [{"feature_index": i,
             "name": FEATURE_NAMES[i],
             "importance": mean_imp[i],
             "std": std_imp[i]}
            for i in order]
    df_out = pd.DataFrame(rows)
    csv_path = out_dir / "feature-importance.csv"
    df_out.to_csv(csv_path, index=False)
    print(f"Wrote {csv_path}")

    # ── Plot ────────────────────────────────────────────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 7))
    names = [FEATURE_NAMES[i] for i in order]
    vals = mean_imp[order]
    errs = std_imp[order]
    colors = ["tab:red" if v > 0.02 else "tab:gray" for v in vals]
    ax.barh(range(len(vals)), vals, xerr=errs, color=colors,
            tick_label=names, edgecolor="black", linewidth=0.5)
    ax.invert_yaxis()
    ax.set_xlabel("Mean |Δaction| under feature permutation")
    ax.set_title("Permutation Feature Importance — handover policy")
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    png_path = out_dir / "feature-importance.png"
    fig.savefig(png_path, dpi=150)
    print(f"Wrote {png_path}")

    # ── Top features ────────────────────────────────────────────────
    print("\nTop 10 features:")
    for r in df_out.head(10).itertuples():
        print(f"  {r.name:16s} {r.importance:.4f} ± {r.std:.4f}")


if __name__ == "__main__":
    main()
