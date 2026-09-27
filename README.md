# DEFIANCE - RL/ML research framework for ns-3

DEFIANCE provides a framework for reinforcement-learning research in ns-3, built
on [ns3-ai](https://github.com/DEFIANCE-project/ns3-ai) for the shared-memory
Python interface. Upstream documentation: <https://DEFIANCE-project.github.io>.

This fork hosts the thesis example
[`examples/nr-rl-handover`](examples/nr-rl-handover/README.md): an aerial-UE
5G-LENA handover scenario with RL agents (PPO via RLlib) evaluated against an A3
baseline. See that README for the pinned forks, build steps and campaign usage.

## Install

1. Clone ns-3 and set `NS3_HOME` to it.
2. Clone ns3-ai and this repo into `contrib/` as `contrib/ai` and
   `contrib/defiance`.
3. `poetry -C contrib/defiance install --without local`, activate the venv, then
   `poetry -C contrib/defiance install --with local`.
4. `./ns3 configure --enable-python --enable-examples --enable-tests`
5. `./ns3 build ai && ./ns3 build`

Then start a scenario with `run-agent train -n <scenario>` (see
`run-agent --help`). Alternatively use the prebuilt container
`ghcr.io/defiance-project/bake-defiance:full-latest`, or the local `Dockerfile`
(`--build-arg BUILD_NS3=False` skips the ns-3 build).

## Development tools

- `ruff check` / `ruff format` for Python lint and formatting.
- `mypy` for type checking (install extras with `poetry install --with dev`).
- ns-3 testsuites: `./test.py -s <suite>`; ns3-ai tests: `pytest contrib/defiance`.

## Frequent problems

A simulation that uses ns3-ai segfaults when no Python agent is attached. Run it
through `run-agent train|debug|random` instead of a bare `ns3 run`.
