# nr-rl-handover - setup and usage

Aerial-UE 5G-LENA handover scenario with RL agents (DEFIANCE + ns3-ai): the
scenario behind the thesis's RL-vs-A3 evaluation campaign. This guide mirrors
the machine the campaign ran on (Ubuntu 22.04 WSL2); a copy lives in the thesis
knowledge base.

## Pinned repositories

| path | origin | pin |
|---|---|---|
| `ns-3-dev/` | `git@gitlab.com:nisaak/ns-3-dev-mathesis.git` | tag `thesis-submission` |
| `contrib/nr` | `git@gitlab.com:nisaak/nr-mt.git` | tag `thesis-submission` |
| `contrib/ai` | `https://github.com/guilloboi1917/ns3-ai-MT` | tag `thesis-submission` |
| `contrib/defiance` | `https://github.com/guilloboi1917/ns3-DEFIANCE-MT` | tag `thesis-submission` |
| `contrib/quic` | `https://github.com/guilloboi1917/quic-ns-3` | tag `thesis-submission` |

All five repositories carry the same annotated tag `thesis-submission`; check it
out in each one. The tags are fixed snapshots, so later branch commits do not
change what the thesis refers to. Clone the ns-3 fork first; `contrib/ai` and
`contrib/defiance` must sit under `contrib/`. RL checkpoints are tied to one
exact build: results do not transfer across the nr v5.0 -> v5.1 change.

## Prerequisites (Ubuntu 22.04)

```bash
sudo apt-get install -y git build-essential cmake ninja-build \
  python3.10 python3.10-dev python3.10-venv libpython3.10-dev \
  libc6-dev libeigen3-dev sqlite3 libsqlite3-dev libpybind11-dev
curl -sSL https://install.python-poetry.org | python3.10 -
```

GPU is needed only for training (CUDA torch); evaluation is CPU-only.

## Build

Run the build inside the Poetry venv: `ns3 configure` detects Python at configure
time and falls back to the system 3.11 otherwise (see Troubleshooting).

```bash
cd contrib/defiance && poetry install          # creates the venv and run-agent
export NS3_HOME=/path/to/ns-3-dev
source ~/.cache/pypoetry/virtualenvs/ns-defiance-*/bin/activate
cd "$NS3_HOME"
./ns3 clean && ./ns3 configure --build-profile=optimized --out=build/optimized \
  --enable-python --enable-python-bindings --enable-examples
./ns3 build defiance-nr-rl-handover
```

## Run

```bash
export PATH="$NS3_HOME:$PATH"                                          # ns3 wrapper
export PATH="$HOME/.cache/pypoetry/virtualenvs/ns-defiance-*/bin:$PATH" # run-agent
rm -f /dev/shm/ns3-ai_*                                                # stale shared memory
cd contrib/defiance/examples/nr-rl-handover                            # results are cwd-relative
```

A3 smoke test:

```bash
./ns3 run "defiance-nr-rl-handover --simDuration=20 --topology=triangle \
  --flowDirection=ul --transportProtocol=udp --handoverAlgorithm=a3 \
  --bandwidthMhz=30 --numerology=1 --seed=1 --runId=1 --outputDir=/tmp/nr-smoke"
```

RL smoke test and training (a Python agent must be attached):

```bash
run-agent train -n defiance-nr-rl-handover -c parallel=1 simDuration=5 \
  rlMode=true handoverAlgorithm=agent -t PPO -st 600 -i 1
run-agent train --config training_configs/train-ppo-ul-no-if-tcp-beta5-fixed.yaml
```

Every run is reproduced from a YAML in `training_configs/` (see
`train-ppo-ul-template.yaml` for the keys); the effective config is written to
the run dir as `meta.yaml`. The checkpoints behind the thesis are mirrored in
`checkpoints/`.

## Evaluation campaign

Configs live in `evaluation-scenarios/{rl,baseline}/*.yaml` (`name` = results dir
= tag = EvaluationData dir; `mode: a3|rl`). Run everything through one runner:

```bash
python3 run-evaluations.py evaluation-scenarios/baseline/transport-comparison.yaml --jobs 4 --analyze
python3 run-evaluations.py evaluation-scenarios/rl/agent-eval-ul-if-tcp.yaml --jobs 4 --analyze
```

- A3 cells run the simulator; RL cells run `run-agent infer` (greedy) and inherit
  the reward/obs settings from the checkpoint's training meta.
- Seeds land in `results/<name>/<tag>/seed_N/`; the analyzer writes `raw.csv` and
  `aggregate.csv` per cell and excludes failed seeds.
- `sync-thesis-data.py` mirrors cells into the Overleaf `EvaluationData/` bundle.

## Layout

```
nr-rl-handover-scenario.{cc,h}      entry point + CLI
nr-rl-handover-scenario-setup.cc    scenario assembly (radio, apps, loggers)
handover-nr/                        obs / reward / action / agent applications
training_configs/                   PPO training configs + template
evaluation-scenarios/{rl,baseline}/ evaluation configs (baseline/run-baselines.sh)
run-evaluations.py                  evaluation runner
analysis/                           feature importance, flowmon parsing
plots/                              per-run stats and UAV-path figures
profiling/                          perf and per-step latency profiling
campaigns/                          local campaign drivers (not committed)
checkpoints/                        mirrored PPO policies (see README)
docs/                               local working notes (not committed)
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| pybind11 compile error (`PyFrameObject`) | configure picked system Python 3.11; activate the venv, `ns3 clean`, reconfigure, build |
| Segfault in an ns3-ai simulation | no Python agent attached; use `run-agent train/debug/random` |
| Deadlock after a crash | stale shared memory: `rm -f /dev/shm/ns3-ai_*` |
| Results in the wrong place | results are cwd-relative; run from the example dir |
| `Cannot TX while RX` fatal | 5G-LENA half-duplex edge case, seed-dependent; the runner marks the seed failed and the analyzer excludes it |

## Versions (campaign machine)

ns-3.47 + local commits; NR v5.1 fork (`a75668ae`); Ray/RLlib 2.55.1;
torch 2.6.0+cu124; Python 3.10.12 (Poetry venv); Ubuntu 22.04.5 WSL2;
i7-11700 / 16 GB RAM / RTX 3060 12 GB.
