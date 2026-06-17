"""!Program code for the `run-agent` cli program."""

import logging
import sys
from argparse import Action, ArgumentParser, Namespace
from collections.abc import Sequence
from typing import Any

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
        setattr(namespace, self.dest, dict(value.split("=") for value in values))


arg_parser = ArgumentParser("run-agent", description="cli tool to launch ns3 coupled with python agents.")
arg_parser.add_argument(
    "type",
    choices=["debug", "train", "infer", "random"],
    help="which python agent should interact with the environments",
)
arg_parser.add_argument(
    "--env-name", "-n", type=str, default="defiance-5g-gym", help="name of the ns3 environment to start"
)
arg_parser.add_argument(
    "--max-episode-steps", "-s", type=int, default=100, help="number of environment steps per iteration"
)
arg_parser.add_argument("--iterations", "-i", type=int, default=50, help="number of environment cycles")
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
    "--trainable", "-t", type=str, default="PPO", help="The algorithm (trainable) to use for training."
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

ns = arg_parser.parse_args()

if "seed" not in ns.ns3_settings:
    ns.ns3_settings["seed"] = "1"
ns.ns3_settings["runId"] = int(ns.ns3_settings.get("runId", "1"))
ns.ns3_settings["parallel"] = int(ns.ns3_settings.get("parallel", "1"))

match ns.type:
    case "debug" | "random":
        if not ns.single:
            from .debug import make_debug_env, make_env, start_random_agent
        else:
            from .single.debug import make_debug_env, make_env, start_random_agent

        match ns.type:
            case "debug":
                env = make_debug_env(ns.env_name, ns.max_episode_steps, ns.ns3_settings)
            case "random":
                env = make_env(ns.env_name, ns.max_episode_steps, ns.ns3_settings)
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
            ns.sample_timeout,
            ns.trainable,
            **ns.ns3_settings,
        )

        wandb_logger = None
        if ns.enable_wandb or ns.wandb_key or ns.wandb_project:
            from ray.air.integrations.wandb import WandbLoggerCallback

            wandb_logger = WandbLoggerCallback(project=ns.wandb_project, api_key=ns.wandb_key)
        start_training(ns.iterations, config, ns.trainable, ns.checkpoint_path, wandb_logger)
    case "infer":
        from .ray import start_inference

        start_inference(ns.env_name, ns.checkpoint_path, **ns.ns3_settings)
    case _:
        assert_never(ns.type)
sys.exit(0)
