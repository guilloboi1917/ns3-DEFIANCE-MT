"""!Program code for the `run-agent` cli program."""

import logging
import sys
from argparse import Action, ArgumentParser, Namespace
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import yaml
from typing_extensions import assert_never, override

logging.basicConfig(level=logging.DEBUG)


class ParseKwargs(Action):
    """!Parse key=value cli parameters into a dictionary."""

    @override
    def __call__(
        self,
        parser: ArgumentParser,
        namespace: Namespace,
        values: str | Sequence[Any] | None,
        option_string: str | None = None,
    ) -> None:
        if values is None or isinstance(values, str):
            return
        setattr(namespace, self.dest, dict(value.split("=")
                for value in values))


arg_parser = ArgumentParser(
    "run-agent", description="cli tool to launch ns3 coupled with python agents.")
arg_parser.add_argument(
    "type",
    nargs="?",
    choices=["debug", "train", "infer", "random"],
    help="which python agent should interact with the environments "
    "(may come from --config instead)",
)
arg_parser.add_argument(
    "--env-name", "-n", type=str, default=None, help="name of the ns3 environment to start (config: env_name)"
)
arg_parser.add_argument(
    "--max-episode-steps", "-s", type=int, default=None, help="number of environment steps per iteration (config: max_episode_steps)"
)
arg_parser.add_argument("--iterations", "-i", type=int,
                        default=None, help="number of environment cycles (config: iterations)")
arg_parser.add_argument(
    "--single",
    "-sg",
    action="store_true",
    default=False,
    help="if enabled, assume a single agent environment instead of a multi-agent environment.",
)
arg_parser.add_argument(
    "--training-params",
    "-p",
    default={},
    action=ParseKwargs,
    nargs="*",
    help="key=value pairs overriding the training parameter",
)
arg_parser.add_argument(
    "--config",
    type=str,
    default=None,
    help="Path to a YAML config file (the per-run meta.yaml written by a "
    "previous training also works). Values in the config provide defaults; "
    "explicit CLI flags and -c key=value pairs override them per key. "
    "After merging, the effective config is written back to the run's "
    "trial directory as meta.yaml for reproducibility.",
)
arg_parser.add_argument(
    "--checkpoint-path", "-a", type=str, default=None, help="path to the checkpoint to load before training"
)

arg_parser.add_argument(
    "--ns3-settings",
    "-c",
    default={},
    action=ParseKwargs,
    nargs="*",
    help="key=value pairs specifying the ns3 settings",
)

arg_parser.add_argument(
    "--enable-wandb",
    action="store_true",
    default=False,
    help="enable wandb logging. By default, project and key will be read from WANDB_API_KEY and WANDB_PROJECT_NAME",
)

arg_parser.add_argument(
    "--wandb-project",
    "-wp",
    type=str,
    required=False,
    help="enable wandb logging to this project",
)

arg_parser.add_argument(
    "--wandb-key",
    "-wk",
    type=str,
    required=False,
    help="enable wandb logging with this api key",
)
arg_parser.add_argument(
    "--trainable", "-t", type=str, default=None, help="The algorithm (trainable) to use for training (config: trainable)."
)
arg_parser.add_argument(
    "--rollout-fragment-length",
    "-rfl",
    type=int,
    default=None,
    help="The rollout fragment length to be used for training.",
)
arg_parser.add_argument(
    "--sample-timeout",
    "-st",
    type=float,
    default=None,
    help="Timeout (seconds) for rollout worker samples. Increase if workers "
    "keep timing out during long episodes (default 30s in RLlib).",
)

arg_parser.add_argument(
    "--evaluation-num-env-runners",
    "-enr",
    type=int,
    default=None,
    help="Number of dedicated evaluation EnvRunners (each runs its own ns-3 "
    "sim). 0 disables evaluation entirely.",
)
arg_parser.add_argument(
    "--evaluation-interval",
    "-ei",
    type=int,
    default=None,
    help="Run evaluation every N training iterations. Default 2 (denser eval "
    "curve + more robust best-checkpoint selection than the old 5; the eval "
    "episode runs in parallel with training, so the cost is ~50%% of one extra "
    "ns-3 sim).",
)
arg_parser.add_argument(
    "--evaluation-duration",
    "-ed",
    type=str,
    default=None,
    help="Episodes per eval worker per evaluation, or 'auto' (run as long as "
    "the parallel training step takes).",
)
arg_parser.add_argument(
    "--evaluation-sample-timeout",
    "-est",
    type=float,
    default=None,
    help="Timeout (seconds) for evaluation workers to return an episode. "
    "Default 600 (RLlib default is 120, which is too low for ns-3 eval "
    "episodes that take 90-400s wall under load; a timeout -> NaN eval metrics).",
)

arg_parser.add_argument(
    "--train-batch-size-per-learner",
    "-tbs",
    type=int,
    default=None,
    help="The training batch size per learner (steps sampled from the replay "
    "buffer per gradient update). Default: computed as `parallel * "
    "simDuration/stepTime` (steps per iteration) so one update consumes ~all "
    "of the iteration's fresh data; a smaller value starves the learner (e.g. "
    "tbs=300 with 2100 steps/iter learns from only ~14%%).",
)

ns = arg_parser.parse_args()

# ── Single source of truth: --config YAML provides defaults, CLI overrides ──
# The per-run meta.yaml written by a previous training is a valid --config,
# so any run is reproducible with: run-agent train --config <run>/meta.yaml
cfg: dict[str, Any] = {}
if ns.config:
    with open(ns.config) as f:
        cfg = yaml.safe_load(f) or {}

def merged(base_key: str) -> dict[str, Any]:
    """Merge a key=value map: config file base, CLI -c/-p wins per key."""
    base = dict(cfg.get(base_key, {}) or {})
    cli = getattr(ns, base_key) or {}
    return {**base, **cli}

def pick(dest: str, cfg_key: str, default: Any) -> Any:
    """Explicit CLI value wins; else config file; else the default."""
    cli_val = getattr(ns, dest)
    if cli_val is not None:
        return cli_val
    return cfg.get(cfg_key, default)

ns.ns3_settings = merged("ns3_settings")
ns.training_params = merged("training_params")
ns.type = ns.type or cfg.get("type")
ns.env_name = pick("env_name", "env_name", "defiance-5g-gym")
ns.trainable = pick("trainable", "trainable", "PPO")
ns.iterations = pick("iterations", "iterations", 50)
ns.max_episode_steps = pick("max_episode_steps", "max_episode_steps", 100)
ns.checkpoint_path = pick("checkpoint_path", "checkpoint_path", None)
ns.rollout_fragment_length = pick("rollout_fragment_length", "rollout_fragment_length", None)
ns.sample_timeout = pick("sample_timeout", "sample_timeout", None)
ns.evaluation_num_env_runners = pick(
    "evaluation_num_env_runners", "evaluation_num_env_runners", 1)
ns.evaluation_interval = pick("evaluation_interval", "evaluation_interval", 2)
ns.evaluation_duration = pick("evaluation_duration", "evaluation_duration", "1")
ns.evaluation_sample_timeout = pick(
    "evaluation_sample_timeout", "evaluation_sample_timeout", 600.0)
ns.train_batch_size_per_learner = pick(
    "train_batch_size_per_learner", "train_batch_size_per_learner", None)

if ns.type is None:
    arg_parser.error("missing required positional 'type' (or 'type:' in --config)")

if "seed" not in ns.ns3_settings:
    ns.ns3_settings["seed"] = "1"
ns.ns3_settings["runId"] = int(ns.ns3_settings.get("runId", "1"))
ns.ns3_settings["parallel"] = int(ns.ns3_settings.get("parallel", "1"))

# Default -tbs to the steps sampled per iteration (parallel * steps per
# episode), so one gradient update consumes ~all of each iteration's fresh
# data (with batch_mode=complete_episodes). Without this, e.g. -tbs 300 with
# 6x70s/200ms (2100 steps/iter) learns from only ~14% of the data.
if ns.train_batch_size_per_learner is None:
    parallel = int(ns.ns3_settings.get("parallel", 1))
    sim_duration = float(ns.ns3_settings.get("simDuration", 30))
    step_ms = float(ns.ns3_settings.get("stepTime", 200))
    steps_per_iter = parallel * (sim_duration / (step_ms / 1000.0))
    ns.train_batch_size_per_learner = int(max(256, min(4096, round(steps_per_iter))))
    print(f"[tbs] computed default train batch size = {ns.train_batch_size_per_learner} "
          f"(parallel={parallel} x {sim_duration:.0f}s/{step_ms:.0f}ms steps)", flush=True)

# ── Effective metadata: everything needed to reproduce this exact run ──
def _build_command() -> str:
    cmd = ["run-agent", ns.type, "-n", ns.env_name, "-t", ns.trainable,
           "-i", str(ns.iterations), "-s", str(ns.max_episode_steps)]
    for k, v in ns.training_params.items():
        cmd += ["-p", f"{k}={v}"]
    for k, v in ns.ns3_settings.items():
        cmd += ["-c", f"{k}={v}"]
    if ns.checkpoint_path:
        cmd += ["-a", ns.checkpoint_path]
    if ns.rollout_fragment_length:
        cmd += ["-rfl", str(ns.rollout_fragment_length)]
    if ns.sample_timeout:
        cmd += ["-st", str(ns.sample_timeout)]
    if ns.train_batch_size_per_learner:
        cmd += ["-tbs", str(ns.train_batch_size_per_learner)]
    cmd += ["-enr", str(ns.evaluation_num_env_runners),
            "-ei", str(ns.evaluation_interval),
            "-ed", str(ns.evaluation_duration),
            "-est", str(ns.evaluation_sample_timeout)]
    return " ".join(cmd)

metadata: dict[str, Any] = {
    "type": ns.type,
    "config_file": ns.config,
    "env_name": ns.env_name,
    "trainable": ns.trainable,
    "iterations": ns.iterations,
    "max_episode_steps": ns.max_episode_steps,
    "training_params": ns.training_params,
    "ns3_settings": ns.ns3_settings,
    "checkpoint_path": ns.checkpoint_path,
    "rollout_fragment_length": ns.rollout_fragment_length,
    "sample_timeout": ns.sample_timeout,
    "train_batch_size_per_learner": ns.train_batch_size_per_learner,
    "evaluation_num_env_runners": ns.evaluation_num_env_runners,
    "evaluation_interval": ns.evaluation_interval,
    "evaluation_duration": ns.evaluation_duration,
    "evaluation_sample_timeout": ns.evaluation_sample_timeout,
    "command": _build_command(),
}
print(f"[config] effective command: {metadata['command']}", flush=True)

match ns.type:
    case "debug" | "random":
        if not ns.single:
            from .debug import make_debug_env, make_env, start_random_agent
        else:
            from .single.debug import make_debug_env, make_env, start_random_agent

        match ns.type:
            case "debug":
                env = make_debug_env(
                    ns.env_name, ns.max_episode_steps, ns.ns3_settings)
            case "random":
                env = make_env(
                    ns.env_name, ns.max_episode_steps, ns.ns3_settings)
            case _:
                assert_never(ns.type)

        start_random_agent(env, ns.iterations)
    case "train":
        if not ns.single:
            from .ray import create_example_training_config, start_training
        else:
            from .single.ray import create_example_training_config, start_training

        config = create_example_training_config(
            ns.env_name,
            ns.max_episode_steps,
            ns.training_params,
            ns.rollout_fragment_length,
            ns.train_batch_size_per_learner,
            ns.sample_timeout,
            ns.trainable,
            **({} if ns.single else {
                "evaluation_num_env_runners": ns.evaluation_num_env_runners,
                "evaluation_interval": ns.evaluation_interval,
                "evaluation_duration": ns.evaluation_duration,
                "evaluation_sample_timeout_s": ns.evaluation_sample_timeout,
            }),
            **ns.ns3_settings,
        )

        wandb_logger = None
        if ns.enable_wandb or ns.wandb_key or ns.wandb_project:
            from ray.air.integrations.wandb import WandbLoggerCallback

            wandb_logger = WandbLoggerCallback(
                project=ns.wandb_project, api_key=ns.wandb_key)
        if ns.single:
            start_training(ns.iterations, config, ns.trainable,
                           ns.checkpoint_path, wandb_logger)
        else:
            start_training(ns.iterations, config, ns.trainable,
                           ns.checkpoint_path, wandb_logger, metadata)
    case "infer":
        from .ray import start_inference

        start_inference(ns.env_name, ns.checkpoint_path, **ns.ns3_settings)
    case _:
        assert_never(ns.type)
sys.exit(0)
