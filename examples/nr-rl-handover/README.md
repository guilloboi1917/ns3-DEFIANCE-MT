# nr-rl-handover — Setup & Installation

Setup and installation guide for the **nr-rl-handover** example: an ns-3 /
5G-LENA aerial-UE handover scenario with RL agents (DEFIANCE + ns3-ai), the
scenario behind the RL-vs-A3 evaluation campaign of the thesis.

The guide mirrors the machine the campaign ran on (Ubuntu 22.04 WSL2, GPU).
Commands that must run from the ns-3 root or the example dir are marked.
A mirror of this file lives in the thesis knowledge base
(`setup-installation.md`); keep both in sync.

---

## 1. What you need to know first

- **The build must happen inside the Python virtual environment.** The
  ns-3/contrib CMake detects Python at configure time; without the venv it
  picks the system Python 3.11 and pybind11 2.9.1 fails to compile against
  it (see Troubleshooting). Always: activate venv -> `ns3 clean` ->
  `ns3 configure` -> `ns3 build`.
- **Reproduction is commit-pinned, not version-pinned.** The whole tree is a
  set of git forks with local changes. Freezing is planned via GitHub /
  GitLab releases (TODO at the end). RL checkpoints are bound to one exact
  build of the simulator: results do not transfer across the nr v5.0 ->
  v5.1 change.
- **Component roles:** `contrib/nr` (5G-LENA, the NR radio), `contrib/ai`
  (ns3-ai, shared-memory ML interface), `contrib/defiance` (RL harness),
  `contrib/quic` (QUIC module, thesis part 2 only), the example itself
  (`contrib/defiance/examples/nr-rl-handover`).

## 2. Repository layout (forks, pinned commits)

| path | origin (fork) | branch | pinned commit |
|---|---|---|---|
| `ns-3-dev/` | `git@gitlab.com:nisaak/ns-3-dev-mathesis.git` | master | `62ab99352` (ns-3.47 + 258 local commits) |
| `ns-3-dev/contrib/nr` | `git@gitlab.com:nisaak/nr-mt.git` | `nr-v5.1-local` | `v5.1-1-ga75668ae` (5G-LENA v5.1 + fixes) |
| `ns-3-dev/contrib/ai` | `https://github.com/guilloboi1917/ns3-ai-MT` | (fork head) | — |
| `ns-3-dev/contrib/defiance` | `https://github.com/guilloboi1917/ns3-DEFIANCE-MT` | `main` | — |
| `ns-3-dev/contrib/quic` | `https://github.com/guilloboi1917/quic-ns-3.47` | (fork head) | — |

Clone the ns-3 fork first, then add the contribs inside it (`contrib/ai` and
`contrib/defiance` must sit under `contrib/`; ns-3's build picks them up
automatically). `contrib/quic` is only needed for the transport part 2; the
B0-B4 evaluation campaign does not use it.

## 3. Prerequisites (Ubuntu 22.04)

```bash
sudo apt-get update
sudo apt-get install -y git build-essential cmake ninja-build \
  python3.10 python3.10-dev python3.10-venv libpython3.10-dev \
  libc6-dev libeigen3-dev sqlite3 libsqlite3-dev libpybind11-dev
curl -sSL https://install.python-poetry.org | python3.10 -
```

Notes:
- `libc6-dev` provides `semaphore.h` (needed by the NR module).
- `libeigen3-dev` enables the NR module's MIMO/channel-matrix features
  (the UMa-AV channel of this project relies on them).
- The venv used here runs Python 3.10.12. Do not let configure fall back to
  the system Python 3.11 (see Troubleshooting).
- GPU: the RLlib learner runs on CUDA (`torch 2.6.0+cu124`); the ns-3
  instances are CPU processes. CUDA-capable hardware + driver needed only
  for training; inference/evaluation runs CPU-only.
- WSL2: shared memory lives in `/dev/shm` (see step 5).

## 4. Step 1 — Python environment (poetry, from `contrib/defiance`)

```bash
cd ns-3-dev/contrib/defiance
poetry install            # creates the venv; installs ray[tune], gymnasium,
                          # torch, pandas/scipy/matplotlib, and the two
                          # ns3-ai editable packages (ns3ai-python-utils,
                          # ns3ai-gym-env) from contrib/ai
```

- `poetry install` exposes the `run-agent` console script (from
  `[tool.poetry.scripts]`) in the venv's `bin/`.
- The venv name on the campaign machine:
  `~/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10`.
- **Torch / GPU:** the pyproject pins a CPU torch wheel by default
  (source `pytorch-cpu`, `https://download.pytorch.org/whl/cpu`). The
  campaign machine runs `torch 2.6.0+cu124`; on a GPU machine replace the
  wheel after `poetry install`:

```bash
source ~/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin/activate
pip install "torch==2.6.0+cu124" --index-url https://download.pytorch.org/whl/cu124
```

## 5. Step 2 — Build ns-3 (venv FIRST, order matters)

```bash
export NS3_HOME=/path/to/ns-3-dev
cd "$NS3_HOME"
source ~/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin/activate
ns3 clean                         # removes build/ AND cmake-cache/
ns3 configure --build-profile=optimized --out=build/optimized \
  --enable-python --enable-python-bindings --enable-examples
ns3 build defiance-nr-rl-handover
```

- The configure line is `NS3OPT` from `build-command.template` (the
  campaign ran the optimized profile; `NS3DEFAULT` is the equivalent
  default-profile variant). With one profile configured, plain
  `./ns3 run` resolves it automatically.
- Build only the example target for quick iteration; `ns3 build` builds the
  whole tree (first build takes a while).

## 6. Step 3 — Runtime essentials

```bash
export NS3_HOME=/path/to/ns-3-dev        # consumed by DEFIANCE examples and run-evaluations
export PATH="$NS3_HOME:$PATH"            # exposes the `ns3` wrapper (repo root)
export PATH="$HOME/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin:$PATH"  # run-agent
rm -f /dev/shm/ns3-ai_*                  # clear stale shared memory before runs
```

- Results dirs resolve **relative to the cwd**. Run the runner/analyzer
  from the example dir so results land next to the other campaigns:
  `cd contrib/defiance/examples/nr-rl-handover`.
- ns3-ai shared-memory segments (`/dev/shm/ns3-ai_*`) are created per run;
  crashed sims can leave them behind and deadlock the next run — the cleanup
  above (or `smoke.sh`) is the fix. A simulation that uses ns3-ai needs a
  python agent attached: `run-agent train/debug/random` (the segfault in the
  Troubleshooting table is the unattached case).

## 7. Verification

A3 smoke (single 20 s A3 run, logs to `/tmp/nr-smoke`):

```bash
cd "$NS3_HOME"
./ns3 run "defiance-nr-rl-handover --simDuration=20 --topology=triangle \
  --flowDirection=ul --transportProtocol=udp --addInterferingUes=0 \
  --aerialUeRatio=0 --uavMobility=random-waypoint --startHeight=50 \
  --endHeight=200 --ueSpeed=20 --bandwidthMhz=30 --numerology=1 \
  --handoverAlgorithm=a3 --logging=true --seed=1 --runId=1 \
  --outputDir=/tmp/nr-smoke"
ls /tmp/nr-smoke/        # expect the trace CSVs (nr-rl-handovers.csv, ...)
```

RL smoke (from the example dir; the bundled `smoke.sh` does the same):

```bash
cd contrib/defiance/examples/nr-rl-handover
run-agent train -n defiance-nr-rl-handover -c parallel=1 simDuration=5 \
  topology=triangle rlMode=true handoverAlgorithm=agent transportProtocol=udp \
  flowDirection=ul addInterferingUes=0 stepTime=200 -t PPO -st 600 -i 1
```

Evaluation dry run (prints the exact commands, touches nothing):

```bash
python3 run-evaluations.py evaluation-scenarios/baseline/capacity-probe-canonical.yaml --dry-run
```

## 8. Training (RLlib PPO)

```bash
# venv active, NS3_HOME set; from the example dir
run-agent train --config training_configs/train-ppo-ul-no-if.yaml   # full run
run-agent train --config training_configs/train-ppo-ul-if.yaml -i 2 -c parallel=2  # smoke
```

- Every run is reproducible from one YAML (training_configs/); CLI flags
  (`-c`, `-p`, `-i`) override per key. The effective config is written to
  the run dir as `meta.yaml` — any past run is reproducible from its own
  meta. `~/ray_results/` holds the full Ray runs; the checkpoints behind the
  thesis results are mirrored into `checkpoints/<run>/` (best_checkpoint +
  meta.yaml + params.json + progress.csv, ~3.6 MB each) so evaluation does
  not depend on the Ray tree. See `checkpoints/README.md`. See the docs
  (REHYDRATION.md) and the training config comments for the meaning of every
  key.

## 9. Evaluation campaign

Configs: `evaluation-scenarios/{rl,baseline}/*.yaml` (each file: `name`
= results dir = tag = EvaluationData dir; `mode: a3|rl`; matrix cells under
`scenarios:`). Run everything through one runner:

```bash
cd contrib/defiance/examples/nr-rl-handover
python3 run-evaluations.py evaluation-scenarios/baseline/transport-comparison.yaml --jobs 4 --analyze
python3 run-evaluations.py evaluation-scenarios/rl/agent-eval-ul-if-tcp.yaml --jobs 4 --analyze
```

- A3 cells run the simulator directly (`ns3 run`); RL cells run
  `run-agent infer` against a checkpoint's `best_checkpoint` (greedy, one
  episode per seed). Reward/obs-affecting settings are inherited from the
  checkpoint's training meta so the eval reward matches training.
- Per-seed results land in `results/<name>/<tag>/seed_N/` (trace CSVs +
  `run-info.yaml` with walltime and `status: ok|failed`); the analyzer
  excludes failed seeds.
- `run-baselines.sh` (baseline dir) runs the A3 configs in order with
  analysis (default 5 jobs, `DRY_RUN=1` previews, per-config failures do
  not stop the campaign).
- Analysis: `analyze-evaluations.py` writes `raw.csv` (per seed) and
  `aggregate.csv` (mean/std/p5/p50/p95) per cell.
- Thesis handoff: `sync-thesis-data.py` mirrors the cell CSVs into the
  Overleaf `EvaluationData/` bundle and writes `manifest.csv`.

## 10. Project layout (example dir)

```
nr-rl-handover/
  nr-rl-handover-scenario.{cc,h}      sim entry point + CLI
  nr-rl-handover-scenario-setup.cc    scenario assembly (radio, apps, loggers)
  handover-nr/                        obs/reward/action/agent applications
  training_configs/train-ppo-ul-{no-if,if}[-tcp].yaml
  evaluation-scenarios/{rl,baseline}/*.yaml   (baseline/run-baselines.sh drives them)
  run-evaluations.py                  evaluation runner (entry point)
  analyze-evaluations.py              per-cell raw.csv / aggregate.csv
  sync-thesis-data.py                 mirror cells into the thesis EvaluationData
  smoke.sh                            environment smoke test
  analysis/                           feature importance, flowmon parsing
  plots/                              per-run stats and UAV-path figures
  profiling/                          perf and per-step latency profiling
  campaigns/                          canonical wave / transport drivers
  obsolete/                           retired scripts (see obsolete/README.md)
  docs/                               REHYDRATION.md, EVALUATION-RUN-PLAN.md,
                                      BUGS-ISSUES.md, CURRENT-NEXT-STEPS.md, ...
  results/                            campaign outputs (per-cell seed_N dirs)
```

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| pybind11 compile error (`PyFrameObject`, `frame->f_back`) after clean/reconfigure | Configure picked the system Python 3.11 (pybind11 2.9.1 predates 3.11 support). Fix: activate the venv, `ns3 clean` (clears the stale CMake cache), reconfigure, build — order matters |
| Training-time segfault | A python agent must be attached to the ns3-ai sim: use `run-agent train/debug/random`; a plain `ns3 run` of an `rlMode=true` sim segfaults |
| Deadlock / stuck run after a crash | Stale shared memory: `rm -f /dev/shm/ns3-ai_*` before rerunning |
| Results land in the wrong place | Results are cwd-relative; cd into the example dir before running the runner/analyzer |
| "Cannot TX while RX" fatal (rare, seed-dependent) | 5G-LENA half-duplex edge case (BUGS-ISSUES #29). The runner marks the seed failed; the analyzer excludes it — safe to ignore a single failed seed |
| Concurrent `run-agent infer` collisions | Each infer process needs a unique shm name: the runner sets `trial_name=eval-<tag>-s<seed>` automatically |

## 12. Versions (campaign machine, verified)

ns-3.47 + 258 local commits (`62ab99352`); NR v5.1 fork
(`v5.1-1-ga75668ae`); Ray/RLlib 2.55.1; torch 2.6.0+cu124; Python 3.10.12
(poetry venv); numpy 1.26.4 / scipy 1.15.3; Ubuntu 22.04.5 WSL2;
i7-11700 / 16 GB RAM / RTX 3060 12 GB; NVIDIA driver 595.95.

## 13. TODOs

- **Freeze the codebase with releases.** Plan: cut GitHub releases for the
  GitHub-hosted forks (ns3-DEFIANCE-MT, ns3-ai-MT, quic-ns-3.47) and GitLab
  releases for the GitLab-hosted forks (ns-3-dev-mathesis, nr-mt) once the
  write-up is final. Before tagging: (a) record the exact tag for each of
  the five repos in section 2 and pin the clone instructions to the tags;
  (b) re-verify the versions table on a fresh checkout from the tags; (c)
  note that RL checkpoints are bound to the pre-release build — evaluate
  reproducibility claims against the tagged tree before submission.
- Align the poetry torch source with the GPU wheel actually used
  (pyproject/lock drift: lock resolves a CPU torch 2.11.0, venv runs
  2.6.0+cu124) — fold into the release freeze.
- Keep this README and the thesis knowledge-base mirror in sync.
