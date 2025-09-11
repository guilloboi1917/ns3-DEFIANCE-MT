import json
import logging
import os
from typing import Any, SupportsFloat

import gymnasium as gym
import ns3ai_gym_env  # noqa: F401  # import to register env
import pandas as pd
from gymnasium.wrappers import TimeLimit
from ray.tune.registry import register_env

NS3_HOME = os.getenv("NS3_HOME") or "/code/ns3"
logger = logging.getLogger(__name__)


def env_creator(_: str) -> gym.Env:
    return env


settings = {
    "optional": {
        "log_component": "",
    },  # always include the optional dict for every arg that is not needed in c++ side
    "topology": "simple",
    "propagationModel": "ns3::FriisPropagationLossModel",
    "handoverAlgorithm": "ns3::A3RsrpHandoverAlgorithm",  # "ns3::A2A4RsrqHandoverAlgorithm"
    "speed": 100,
}

if __name__ == "__main__":
    global_info: dict[str, list[list[Any]]] = {
        "action": [],
        "sinrs": [],
        "distances": [],
    }

    env: gym.Env = TimeLimit(
        gym.make(
            "ns3ai_gym_env/Ns3-v0",
            targetName="defiance-lte-learning",
            ns3Path=NS3_HOME,
            ns3Settings=settings,
        ),
        max_episode_steps=100,
    )
    register_env("defiance", env_creator)

    try:
        obs, info = env.reset()
        logger.info("Initial Observations after Env was reset %s", obs)
        logger.info("Initial Info after Env was reset %s", info)

        reward: SupportsFloat = 0
        done = False

        for steps in range(2500):
            if steps % 100 == 0:
                logger.info("Progress %s", steps)
                action = [10]

            global_info["action"].append(action)

            obs, reward, done, _, info = env.step(action)
            info = json.loads(info["info"])
            global_info["sinrs"].append(info["sinrs"])
            global_info["distances"].append(info["distances"])

    except Exception:
        logger.exception("Exception occurred")

    finally:
        logger.info("Finally exiting...")
        env.close()

    pd.DataFrame(global_info).to_csv(NS3_HOME + "/contrib/defiance/experiments/results/test0.csv")
