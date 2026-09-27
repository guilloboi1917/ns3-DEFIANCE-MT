# checkpoints/ - the PPO policies behind the thesis results (2026-09-15; corrected runs added 2026-09-23)

Self-contained mirror of the checkpoints that the evaluation results in this
work trace back to. Evaluation no longer depends on the machine-local Ray
tree (`~/ray_results/`); the RL evaluation configs in
`evaluation-scenarios/rl/*.yaml` point here.

## What is mirrored per run

```
checkpoints/<run>/
  best_checkpoint/    RLlib checkpoint selected during training (21 files, 3.4 MB)
  meta.yaml           effective run config written by run-agent at training start
  params.json         RLlib algorithm config (all hyperparameters)
  progress.csv        per-iteration training metrics
```

`best_checkpoint/` is the only artifact needed to run a policy: `run-agent
infer -a <run dir>` resolves `<run dir>/best_checkpoint` itself
(`defiance/model/agents/ray.py`, `start_inference`). `meta.yaml` is the second
required file: `run-evaluations.py` walks up from the checkpoint to find it and
inherits the reward/observation settings from the training run, so the
evaluation reward matches training (`REWARD_ENV_KEYS` in `run-evaluations.py`).
`params.json` and `progress.csv` are records, not needed for inference.

## The runs

**2026-09-23: the quoted arm is now the two corrected runs** (`PPO_2026-09-20_13-59-15`
and `PPO_2026-09-21_12-07-15`, both `train-ppo-ul-*-beta5-fixed.yaml`, corrected
serving-cell interface and uniformly unsteered default). The three pre-fix runs
below are the pre-fix record: they were trained before the interface fix and
under the old `directpath` scenario default, they are kept for provenance only,
and no final number is quoted from them.

| run | role | config | iters | beta_H | interferers | selected checkpoint | `algorithm_state.pkl` md5 |
|---|---|---|---|---|---|---|---|
| `PPO_2026-09-20_13-59-15` | **quoted** clean-regime arm (corrected interface) | `train-ppo-ul-no-if-tcp-beta5-fixed.yaml` | 100 | 5.0 | 0 | `checkpoint_000054` | `bc2e2ff845358f982282721e4d8c660d` |
| `PPO_2026-09-21_12-07-15` | **quoted** in-regime arm (corrected interface) | `train-ppo-ul-if-tcp-beta5-fixed.yaml` | 100 | 5.0 | 4 | `checkpoint_000052` | `f3769307406e14f416a95f9ffa2ef1ad` |
| `PPO_2026-09-02_00-07-41` | pre-fix: no-if, beta 1.0 (reference + zero-shot transfer to if) | `train-ppo-ul-no-if-tcp.yaml` | 120 | 1.0 | 0 | `checkpoint_000118` | `bdeebcd966fac9d36eb78105e04d3efb` |
| `PPO_2026-09-07_13-52-01` | pre-fix: no-if, beta 5.0 (former headline no-if policy) | `train-ppo-ul-no-if-tcp-beta5.yaml` | 100 | 5.0 | 0 | `checkpoint_000051` | `d6581619104cedc179eaa88fe87b20be` |
| `PPO_2026-09-02_20-53-50` | pre-fix: if-native (interference regime) | `train-ppo-ul-if-tcp.yaml` | 100 | 1.0 | 4 | `checkpoint_000079` | `c15c8ba315e0d20f78b89238d73e8abd` |

Copied 2026-09-15 from `~/ray_results/<run>/`; each `best_checkpoint/` tree was
verified byte-identical to its source (recursive md5 of the tree: `784cd60609fe`,
`d97ed8779611`, `e8cb5a28059e` for the three pre-fix runs in table order above).
The two corrected runs were copied the same way on 2026-09-23; their
`best_checkpoint/` trees verify byte-identical to the sources (recursive tree md5
`f04ab00b3832170dabe3cf65ba442526` and `c0988ff4c2ed0108d341d3a068eabfcb`),
and their selected checkpoints match `checkpoint_000054` and `checkpoint_000052`
of the respective trial dirs.

## Configs that use them

All live RL eval configs under `evaluation-scenarios/rl/` that belong to the
quoted arm reference this directory (2026-09-23: `agent-eval-ul-no-if-tcp-beta5-fixed`,
`agent-eval-ul-if-tcp-beta5-fixed-transfer`, `agent-eval-ul-if-tcp-beta5-fixed-native`,
`agent-eval-ul-no-if-quic-beta5-fixed`, `agent-eval-ul-if-quic-beta5-fixed`,
`agent-eval-ul-if-quic-beta5-fixed-native` were repointed here from
`~/ray_results/`; the weights are identical, so no stored result changes).
The remaining pre-fix configs (`agent-eval-ul-{no-if,if}-{tcp,quic}*`,
`agent-eval-ul-if-tcp-transfer`, `rl-eval-hexgrid`, `rl-eval-extremes-alt`)
also point here.

Two legacy configs still point at `~/ray_results/`: `agent-eval-ul-no-if.yaml`
(placeholder path, UDP-era) and `agent-eval-ul-if.yaml` (the pre-canonical
`PPO_2026-08-31_14-53-00` run, which lives under
`~/ray_results/non-canonical/` and was not mirrored). Neither is part of the
canonical campaign.

## Refreshing

Re-copy with the same layout if a run is ever re-selected or re-trained:

```bash
src="$HOME/ray_results/<run>"; dst="checkpoints/<run>"
mkdir -p "$dst"
cp -r "$src/best_checkpoint" "$dst/"
cp "$src/meta.yaml" "$dst/meta.yaml"
sub=$(ls -d "$src"/PPO_defiance_*/ | head -1)
cp "$sub/params.json" "$sub/progress.csv" "$dst/"
```

The full Ray run dirs (~340-407 MB each: every `checkpoint_N`, the Tune
experiment state and tfevents) stay in `~/ray_results/`; they are only needed
to resume training or to pick a different iteration. The thesis-side
metric-only mirror is `Master-Thesis-Overleaf/TrainingData/`.
