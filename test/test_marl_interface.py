"""Python part to marl-interface-test. Typical testcases: Start a Ns3MultiAgentEnv and assert communication was successful."""

from collections.abc import Collection, Iterator
from typing import TypeVar

import pytest
from _pytest.mark import ParameterSet
from ns3ai_gym_env.envs.ns3_multi_agent_environment import Ns3MultiAgentEnv

from defiance import NS3_HOME
from defiance.utils import first

NS3AI_TEST_SUITES = ["marl-interface", "marl-echo-action-interface", "marl-agent-interface"]

COUNTING_AGENT_APP_LIMIT = 10

T = TypeVar("T")


def only(it: Collection[T]) -> T:
    assert len(it) == 1
    return first(it)


@pytest.fixture()
def env(request: ParameterSet) -> Iterator[Ns3MultiAgentEnv]:
    env = Ns3MultiAgentEnv(
        "test-runner",
        NS3_HOME,
        ns3Settings={
            "test-name": request.param,
            "stop-on-failure": None,
            "fullness": "QUICK",
            "verbose": None,
        },
        trial_name=None,
    )
    yield env
    env.close()
    del env


@pytest.mark.parametrize("env", NS3AI_TEST_SUITES, indirect=True)
def test_ai_interface(env: Ns3MultiAgentEnv) -> None:
    states = env.reset()
    assert all(isinstance(state, dict) for state in states)
    agent = only({only(state) for state in states})

    stop_requested = False
    stepped = False
    while not stop_requested:
        action = env.action_spaces[agent].sample()
        states = env.step({agent: action})
        assert all(isinstance(state, dict) for state in states)
        if states[2].pop("__all__") or states[3].pop("__all__"):
            break
        agent = only({only(state) for state in states})
        observation, reward, terminated, truncated, info = (only(state.values()) for state in states)
        stop_requested = terminated or truncated
        stepped = True
    assert stepped


@pytest.mark.parametrize("env", ["marl-echo-action-interface"], indirect=True)
def test_echo_interface(env: Ns3MultiAgentEnv) -> None:
    agent, observation = only(env.reset()[0].items())
    terminated = False
    while not terminated:
        action = env.action_spaces[agent].sample()
        states = env.step({agent: action})
        if states[2].pop("__all__") or states[3].pop("__all__"):
            break
        agent = only({only(state) for state in states})
        observation, reward, terminated, truncated, info = (only(state.values()) for state in states)
        if not terminated:
            assert all(action == observation)
            assert action[0] == reward


@pytest.mark.parametrize("env", ["marl-agent-interface"], indirect=True)
def test_counting_agent(env: Ns3MultiAgentEnv) -> None:
    agent, observation = only(env.reset()[0].items())
    for i in range(1, COUNTING_AGENT_APP_LIMIT):
        assert i == only(observation)
        action = env.action_spaces[agent].sample()
        states = env.step({agent: action})
        states[2].pop("__all__")
        states[3].pop("__all__")
        agent = only({only(state) for state in states})
        observation, _, _, _, _ = (only(state.values()) for state in states)
    assert only(observation) == COUNTING_AGENT_APP_LIMIT
