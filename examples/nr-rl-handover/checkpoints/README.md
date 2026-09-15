# checkpoints/ - the PPO policies behind the thesis results (2026-09-15)

Self-contained mirror of the three checkpoints that every evaluation result in
this work traces back to. Evaluation no longer depends on the machine-local Ray
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

| run | role | config | iters | beta_H | interferers | selected checkpoint | `algorithm_state.pkl` md5 |
|---|---|---|---|---|---|---|---|
| `PPO_2026-09-02_00-07-41` | no-if, beta 1.0 (reference + zero-shot transfer to if) | `train-ppo-ul-no-if-tcp.yaml` | 120 | 1.0 | 0 | `checkpoint_000118` | `bdeebcd966fac9d36eb78105e04d3efb` |
| `PPO_2026-09-07_13-52-01` | no-if, beta 5.0 (headline no-if policy) | `train-ppo-ul-no-if-tcp-beta5.yaml` | 100 | 5.0 | 0 | `checkpoint_000051` | `d6581619104cedc179eaa88fe87b20be` |
| `PPO_2026-09-02_20-53-50` | if-native (interference regime) | `train-ppo-ul-if-tcp.yaml` | 100 | 1.0 | 4 | `checkpoint_000079` | `c15c8ba315e0d20f78b89238d73e8abd` |

Copied 2026-09-15 from `~/ray_results/<run>/`; each `best_checkpoint/` tree was
verified byte-identical to its source (recursive md5 of the tree: `784cd60609fe`,
`d97ed8779611`, `e8cb5a28059e` for the three runs in the table order above).

## Configs that use them

Nine live configs under `evaluation-scenarios/rl/` reference this directory
(`agent-eval-ul-{no-if,if}-{tcp,quic}*`, `agent-eval-ul-if-tcp-transfer`,
`rl-eval-hexgrid`, `rl-eval-extremes-alt`).

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
