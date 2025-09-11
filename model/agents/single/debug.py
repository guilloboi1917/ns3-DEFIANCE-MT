"""!Debug agent. Interacts with the ns3 environment some random steps."""

import logging
from typing import Any

import gymnasium as gym
import ns3ai_gym_env  # noqa: F401  # import to register env
from gymnasium.wrappers import TimeLimit

from defiance import NS3_HOME

logger = logging.getLogger(__name__)


def make_debug_env(env_name: str, max_episode_steps: int, ns3_settings: dict[str, Any], **env_args: Any) -> gym.Env:
    """Make a configured ns3-ai gym env with debugging enabled by default."""
    return make_env(env_name, max_episode_steps, ns3_settings, **env_args, debug=True)


def make_env(env_name: str, max_episode_steps: int, ns3_settings: dict[str, Any], **env_args: Any) -> gym.Env:
    """Make a configured ns3-ai gym env."""
    return TimeLimit(
        gym.make("ns3ai_gym_env/Ns3-v0", targetName=env_name, ns3Path=NS3_HOME, ns3Settings=ns3_settings, **env_args),
        max_episode_steps=max_episode_steps,
    )


def start_random_agent(env: gym.Env, iterations: int) -> None:
    for iteration in range(iterations):
        truncated = False
        obs, info = env.reset()
        logger.info("initial observation: %s, info: %s", obs, info)
        while not truncated:
            obs, reward, _, truncated, info = env.step(env.action_space.sample())
            logger.info("got observation %s, reward %f, info %s", obs, reward, info)
        logger.info("iteration %i completed", iteration)
    env.close()
