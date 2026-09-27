# checkpoints/ - the PPO policies behind the thesis results

Self-contained mirror of the checkpoints the evaluation results trace back to,
so evaluation does not depend on the machine-local Ray tree (`~/ray_results/`).
The RL evaluation configs point here.

## Layout per run

```
checkpoints/<run>/
  best_checkpoint/    RLlib checkpoint (21 files, ~3.4 MB); the only artifact
                      needed for inference (`run-agent infer -a <run dir>`)
  meta.yaml           effective training config; run-evaluations.py inherits the
                      reward/obs settings from it (REWARD_ENV_KEYS)
  params.json         RLlib algorithm config (record)
  progress.csv        per-iteration training metrics (record)
```

## Runs

| run | role | config | iters | beta_H | interferers | selected ckpt | `algorithm_state.pkl` md5 |
|---|---|---|---|---|---|---|---|
| `PPO_2026-09-20_13-59-15` | quoted clean regime | `train-ppo-ul-no-if-tcp-beta5-fixed.yaml` | 100 | 5.0 | 0 | `checkpoint_000054` | `bc2e2ff845358f982282721e4d8c660d` |
| `PPO_2026-09-21_12-07-15` | quoted interfered regime | `train-ppo-ul-if-tcp-beta5-fixed.yaml` | 100 | 5.0 | 4 | `checkpoint_000052` | `f3769307406e14f416a95f9ffa2ef1ad` |
| `PPO_2026-09-02_00-07-41` | pre-fix, no-if beta 1.0 | `train-ppo-ul-no-if-tcp.yaml` | 120 | 1.0 | 0 | `checkpoint_000118` | `bdeebcd966fac9d36eb78105e04d3efb` |
| `PPO_2026-09-07_13-52-01` | pre-fix, no-if beta 5.0 | `train-ppo-ul-no-if-tcp-beta5.yaml` | 100 | 5.0 | 0 | `checkpoint_000051` | `d6581619104cedc179eaa88fe87b20be` |
| `PPO_2026-09-02_20-53-50` | pre-fix, interfered beta 1.0 | `train-ppo-ul-if-tcp.yaml` | 100 | 1.0 | 4 | `checkpoint_000079` | `c15c8ba315e0d20f78b89238d73e8abd` |

The two `-fixed` runs are the quoted arm (corrected serving-cell interface,
uniformly unsteered). The three pre-fix runs predate both and are kept for
provenance only; no final number is quoted from them. Each `best_checkpoint/`
tree was verified byte-identical to its `~/ray_results/` source: recursive tree
md5 `f04ab00b3832170dabe3cf65ba442526` and `c0988ff4c2ed0108d341d3a068eabfcb`
for the two quoted runs, `784cd60609fe`, `d97ed8779611` and `e8cb5a28059e` for
the pre-fix runs in table order.

All live RL eval configs in `evaluation-scenarios/rl/` reference this directory.
Two legacy configs still point at `~/ray_results/`: `agent-eval-ul-no-if.yaml`
(placeholder path) and `agent-eval-ul-if.yaml` (pre-canonical run, not mirrored).

## Refreshing

Re-copy with the same layout if a run is re-selected or re-trained:

```bash
src="$HOME/ray_results/<run>"; dst="checkpoints/<run>"; mkdir -p "$dst"
cp -r "$src/best_checkpoint" "$dst/"
cp "$src/meta.yaml" "$dst/meta.yaml"
sub=$(ls -d "$src"/PPO_defiance_*/ | head -1)
cp "$sub/params.json" "$sub/progress.csv" "$dst/"
```

The full Ray run dirs (~340-407 MB each) stay in `~/ray_results/`; they are
needed only to resume training or pick a different iteration. The thesis-side
metric-only mirror is `Master-Thesis-Overleaf/TrainingData/`.
