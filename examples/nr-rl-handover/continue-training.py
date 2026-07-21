#!/usr/bin/env python3
"""Patch a Ray Tune experiment state file to allow continued training.

Ray 2.x Tuner.restore() ignores CLI --iterations because the stop condition
is baked into the experiment_state-*.json file.  This script edits that file
in-place so training can resume past the original iteration limit.

Usage:
    python continue_training.py ~/ray_results/PPO_2026-06-18_13-12-42/ --iterations 30
    python continue_training.py ~/ray_results/PPO_2026-06-18_13-12-42/ -i 30 --dry-run

After running this, use your normal train command with -a pointing to the same
directory.  The -i flag on the CLI is still ignored, but the experiment state
now has the desired limit.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any


def find_latest_state_file(exp_dir: Path) -> Path | None:
    """Return the newest experiment_state-*.json, or None."""
    state_files = sorted(exp_dir.glob("experiment_state-*.json"))
    if not state_files:
        return None
    # Prefer the one with the latest modification time.
    return max(state_files, key=lambda p: p.stat().st_mtime)


def patch_state_file(
    path: Path,
    new_iterations: int,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Patch status, done flag, and training_iteration in *path*."""
    with open(path) as f:
        data: dict[str, Any] = json.load(f)

    if "trial_data" not in data or not data["trial_data"]:
        raise ValueError(f"No trial_data in {path}")

    trial_raw = data["trial_data"][0][0]

    # --- Extract current iteration count for reporting ---
    m = re.search(r'"training_iteration"\s*:\s*(\d+)', trial_raw)
    old_iter = int(m.group(1)) if m else None

    # --- Patch ---
    changes = 0
    for old, new in [
        ('"status": "TERMINATED"', '"status": "PAUSED"'),
        ('"status": "ERROR"', '"status": "PAUSED"'),
        ('"done": true', '"done": false'),
    ]:
        if old in trial_raw:
            trial_raw = trial_raw.replace(old, new)
            changes += 1

    if old_iter is not None and old_iter != new_iterations:
        trial_raw = trial_raw.replace(
            f'"training_iteration": {old_iter}',
            f'"training_iteration": {new_iterations}',
        )
        changes += 1

    data["trial_data"][0][0] = trial_raw

    if not dry_run:
        with open(path, "w") as f:
            json.dump(data, f)

    return {
        "path": str(path),
        "old_iter": old_iter,
        "new_iter": new_iterations,
        "changes": changes,
        "dry_run": dry_run,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Patch Ray Tune experiment state to continue training."
    )
    parser.add_argument(
        "experiment_dir",
        type=Path,
        help="Path to the experiment directory (contains experiment_state-*.json).",
    )
    parser.add_argument(
        "--iterations",
        "-i",
        type=int,
        required=True,
        help="New total training iterations (e.g., 30 to continue from 20).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would change without writing.",
    )
    args = parser.parse_args()

    exp_dir = args.experiment_dir.expanduser().resolve()
    if not exp_dir.is_dir():
        print(f"ERROR: {exp_dir} is not a directory.", file=sys.stderr)
        sys.exit(1)

    # --- Find and clean up state files ---
    all_state_files = sorted(exp_dir.glob("experiment_state-*.json"))
    if not all_state_files:
        print(f"ERROR: No experiment_state-*.json found in {exp_dir}", file=sys.stderr)
        sys.exit(1)

    target = find_latest_state_file(exp_dir)
    if target is None:
        print(f"ERROR: Could not determine latest state file.", file=sys.stderr)
        sys.exit(1)

    # Remove older state files to avoid trial-id conflicts during restore.
    removed = []
    if not args.dry_run:
        for f in all_state_files:
            if f != target:
                f.unlink()
                removed.append(f.name)
    else:
        removed = [f.name for f in all_state_files if f != target]

    # --- Patch ---
    info = patch_state_file(target, args.iterations, dry_run=args.dry_run)

    # --- Report ---
    print(f"Experiment dir: {exp_dir}")
    print(f"State file:     {target.name}")
    print(f"Iterations:     {info['old_iter']} → {info['new_iter']}")
    if removed:
        print(f"Removed stale:  {', '.join(removed)}")
    if info["dry_run"]:
        print("DRY RUN — no files were modified.")
    else:
        print(f"Changes made:   {info['changes']}")
    print()
    print("Now run:")
    print(f"  run-agent train ... -a {exp_dir}/ -i {args.iterations}")


if __name__ == "__main__":
    main()
