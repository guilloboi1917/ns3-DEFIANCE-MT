"""!Agent leveraging ray to train a single agent for a certain ns3 environment."""

import logging
from typing import Any

import gymnasium as gym
import ray
from gymnasium.wrappers import TimeLimit
from ns3ai_gym_env.envs import Ns3Env
from ray.air.integrations.wandb import WandbLoggerCallback
from ray.rllib.algorithms import AlgorithmConfig
from ray.tune import Tuner, register_env
from ray.tune.impl.config import CheckpointConfig, RunConfig
from ray.tune.registry import get_trainable_cls

from defiance import NS3_HOME

logger = logging.getLogger(__name__)


def create_example_training_config(
    env_name: str,
    max_episode_steps: int,
    training_params: dict[str, Any],
    rollout_fragment_length: int,
    trainable: str = "PPO",
    **ns3_settings: str,
) -> AlgorithmConfig:
    """!Create an example algorithm config for use with single agent training."""

    def environment_creator(_config: dict[str, Any]) -> Ns3Env:
        import ns3ai_gym_env  # noqa: F401  # import again to register env

        return TimeLimit(
            gym.make("ns3ai_gym_env/Ns3-v0", targetName=env_name, ns3Path=NS3_HOME, ns3Settings=ns3_settings),
            max_episode_steps=max_episode_steps,
        )

    register_env("defiance", environment_creator)

    config: AlgorithmConfig = get_trainable_cls(trainable).get_default_config()

    return (
        config.environment(env="defiance")
        .training(**training_params)
        .resources(num_gpus=0)
        .env_runners(
            num_env_runners=1,
            num_envs_per_env_runner=1,
            create_env_on_local_worker=False,
            rollout_fragment_length=rollout_fragment_length or "auto",
        )
    )


def start_training(
    iterations: int,
    config: AlgorithmConfig,
    trainable: str = "PPO",
    load_checkpoint_path: str | None = None,
    wandb_logger: WandbLoggerCallback | None = None,
) -> None:
    """!Start a ray training session with the given algorithm config."""
    logger.info("Loading checkpoints is not supported for single agent: %s", load_checkpoint_path)
    try:
        ray.init()
        logger.info("Training...")
        Tuner(
            trainable,
            run_config=RunConfig(
                stop={"training_iteration": iterations},
                checkpoint_config=CheckpointConfig(checkpoint_at_end=True),
                callbacks=[wandb_logger] if wandb_logger else [],
            ),
            param_space=config.to_dict(),
        ).fit()

        ray.shutdown()

    except Exception:
        logger.exception("Exception occurred!")
