#!/usr/bin/env python3
"""Plot training progress from a Ray result.json file.

Accepts either a direct path to result.json or an experiment directory
(e.g. ~/ray_results/PPO_2026-05-06_17-21-31) that contains a single trial
subdirectory with result.json inside.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

plt.rcParams["pdf.fonttype"] = 42


def _find_result_json(path: str) -> tuple[str, str]:
    """Return the result.json path and a label derived from the run."""
    p = Path(path).expanduser().resolve()

    # Direct path to a result.json
    if p.is_file():
        return str(p), p.parent.name

    # Experiment directory: expect one trial folder containing result.json
    if p.is_dir():
        trials = [d for d in p.iterdir() if d.is_dir() and d.name not in ("best_checkpoint",)]
        if len(trials) == 1:
            result_file = trials[0] / "result.json"
            if result_file.exists():
                return str(result_file), p.name
        # Fallback: search recursively
        candidates = list(p.rglob("result.json"))
        if candidates:
            return str(candidates[0]), p.name
        msg = f"No result.json found under {p}"
        raise FileNotFoundError(msg)

    msg = f"Path does not exist: {p}"
    raise FileNotFoundError(msg)


def _resolve_metric(history: pd.DataFrame, metric: str) -> pd.Series:
    """Extract a metric from the DataFrame.

    Supports nested paths like "env_runners/episode_return_mean" where
    "env_runners" is a column of dicts.
    """
    # Direct column match
    if metric in history.columns:
        return history[metric]

    # Nested path: split on first slash, traverse dict cells
    parts = metric.split("/", 1)
    if len(parts) == 2 and parts[0] in history.columns:
        col = history[parts[0]]
        if col.notna().any() and isinstance(col.dropna().iloc[0], dict):
            inner_key = parts[1]
            return col.apply(lambda cell: cell.get(inner_key) if isinstance(cell, dict) else None)

    # Fallback: old-style flat metrics
    fallbacks = ["episode_reward_mean", "episode_return_mean"]
    for fb in fallbacks:
        if fb in history.columns:
            print(f"Using fallback metric: {fb}")
            return history[fb]

    available = [col for col in history.columns if "reward" in col.lower() or "return" in col.lower()]
    msg = f"Metric '{metric}' not found. Available reward-related columns: {available}"
    raise KeyError(msg)


def plot_progress(json_file: str, metric: str, label: str, n_iterations: int) -> None:
    """Read a result.json and plot the given metric."""
    history = pd.read_json(json_file, lines=True)
    series = _resolve_metric(history, metric)
    series = series.head(n_iterations)
    # Drop NaN values (e.g. when episode_return_mean hasn't been recorded yet)
    series = series.dropna()
    plt.plot(series.index, series, label=label)


parser = argparse.ArgumentParser(
    description="Visualize training progress from a Ray result.json file or experiment directory."
)
parser.add_argument(
    "json_file",
    help="Path to result.json or experiment root directory (e.g. ~/ray_results/PPO_2026-05-06_17-21-31)",
)
parser.add_argument(
    "-l",
    "--label",
    default=None,
    help="Label for the curve (default: directory name)",
)
parser.add_argument(
    "-m",
    "--metric",
    default="env_runners/episode_return_mean",
    help="Metric column to plot, supports nested paths (default: env_runners/episode_return_mean)",
)
parser.add_argument(
    "-i",
    "--iterations",
    type=int,
    default=100,
    help="Number of iterations to plot",
)
parser.add_argument(
    "-o",
    "--output",
    default="training_progress.pdf",
    help="Output file path",
)
args = parser.parse_args()

# Find the result.json and derive a default label
result_path, auto_label = _find_result_json(args.json_file)
label = args.label or auto_label

print(f"Plotting {result_path}")
plot_progress(result_path, args.metric, label, args.iterations)

plt.xlabel("Iteration")
plt.ylabel("Mean reward per iteration")
plt.legend()
plt.grid(visible=True)
plt.savefig(args.output, bbox_inches="tight")
print(f"Saved to {args.output}")
plt.show()
plt.close()
