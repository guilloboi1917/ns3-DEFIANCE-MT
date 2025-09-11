"""!Debug agent. Interacts with the ns3 environment some random steps."""

import logging
from typing import Any

import ns3ai_gym_env  # noqa: F401  # import to register env
from ns3ai_gym_env.envs import Ns3MultiAgentEnv

from defiance import NS3_HOME
from defiance.utils import first

logger = logging.getLogger(__name__)


def make_debug_env(
    env_name: str, max_episode_steps: int, ns3_settings: dict[str, Any], **env_args: Any
) -> Ns3MultiAgentEnv:
    """Make a configured ns3-ai gym env with debugging enabled by default."""
    return make_env(env_name, max_episode_steps, ns3_settings, **env_args, debug=True)


def make_env(env_name: str, max_episode_steps: int, ns3_settings: dict[str, Any], **env_args: Any) -> Ns3MultiAgentEnv:
    """Make a configured ns3-ai gym env."""
    logger.info("max_episode_steps %s not supported for multi-agent!", max_episode_steps)
    return Ns3MultiAgentEnv(targetName=env_name, ns3Path=NS3_HOME, ns3Settings=ns3_settings, **env_args)


def start_random_agent(env: Ns3MultiAgentEnv, iterations: int) -> None:
    """Run iterations on the env, each using random steps until truncated or terminated."""
    for iteration in range(iterations):
        truncated = False
        terminated = False
        reset = env.reset()
        agent, observation = first(reset[0].items())
        _, info = first(reset[1].items())
        logger.info("initial observation: %s, info: %s", observation, info)
        i = 0
        while not terminated and not truncated:
            i += 1
            action = env.action_spaces[agent].sample()
            logger.info("taking action %s", action)
            states = env.step({agent: action})
            logger.info("got states %s", states)
            agent = first(states[0])
            logger.info([state[agent] for state in states])
            observation, reward, terminated, truncated, info = (state[agent] for state in states)
            logger.info(
                "got observation %s, reward %f, terminated %s, truncated %s, info %s",
                observation,
                reward,
                terminated,
                truncated,
                info,
            )
        logger.info("iteration %i completed", iteration)
    env.close()
