# obsolete/ - retired harness scripts

Files here are kept for reference but are no longer part of the active
workflow. They are moved (not deleted) so the evidence and methods behind
past decisions remain inspectable.

| file | retired | reason |
|---|---|---|
| `plot-evaluation.py` | 2026-09-11 | Superseded by `Master-Thesis-Overleaf/KnowledgeBase/plot_eval_results.py`, which reads the synced `EvaluationData` bundle and produces the thesis figures. This script expected the `{transport}-{direction}-{if_type}` tag format, which no result directory uses any more. |
| `plot-reward-stats.py` | 2026-09-11 | Reward-structure plotting; `plots/plot-nr-rl-stats.py` now draws a "Reward Components" panel from the same `rl_reward.csv`. Kept here for the finer R_G/R_H decomposition and distribution panels. |
| `feature-audit.py` | 2026-09-11 | A-priori observation-space screening. The observation space is frozen, so the pre-training audit phase is closed; the results are recorded in `docs/REHYDRATION.md` and the thesis KnowledgeBase. |
| `analyze-timer-race.py` | 2026-09-11 | Timer-race / RTO-attractor diagnostics for BUGS-ISSUES #34. That investigation concluded with the canonical buffer/timer regime (`KnowledgeBase/buffer-timer-regime.md`), so the tool is no longer part of the active analysis path. |
| `run-rlc-buffer-sweep.sh` | 2026-09-11 | RLC TX-buffer sensitivity probe (3 caps x 4 congestion controls x 4 stuck seeds). Superseded by the adopted canonical buffer/timer regime. |
| `run-buffer-2d-sweep.sh` | 2026-09-11 | Two-knob buffer/timer sweep (MinRto ablation + SndBuf x RLC grid). Same closed investigation. |
| `run-config-search.sh` | 2026-09-11 | Fixed-stuck-seed search over the three candidate fixes. Same closed investigation, no remaining references. |
| `plot-antenna-pattern.py` | 2026-09-15 | Early 3D visualization of the ParabolicAntennaModel. The scenario uses `ThreeGppAntennaModelOriented`; the script produced no thesis figure and had no remaining references. |
| `plot-three-gpp-antenna.py` | 2026-09-15 | Validation plot of the oriented 3GPP antenna pattern. No thesis figure and no remaining references. |
| `best-checkpoint.py` | 2026-09-15 | Checkpoint picker used while training was running. Training is finished and the chosen checkpoints are recorded, so it is no longer part of the workflow. |
| `continue-training.py` | 2026-09-15 | Ray Tune experiment-state patcher for resuming a run past its iteration limit. The training campaign is closed. |

`analyze-timer-race.py` was the only tool covering per-seed stall rate and
CA_OPEN -> CA_LOSS cycle counts; `analyze-evaluations.py` still does not
compute those, so restore it from here if that diagnostic is needed again.

The three `run-*.sh` probes were given a one-line path fix on archival (`HERE`
now resolves the parent directory), so they still find `NS3ROOT` and write to
the real `results/` directory if re-run from here.

## Reorganization 2026-09-15

The loose harness scripts in the example directory were moved into
purpose-named subdirectories. Nothing was deleted.

| from (example dir) | to |
|---|---|
| `feature-importance.py`, `feature-importance-summary.py`, `run-feature-importance.sh`, `flowmon-parse-results.py` | `analysis/` |
| `plot-nr-rl-stats.py`, `plot-uav-path.py` | `plots/` |
| `profile-perf.sh`, `profile-rl.sh`, `rl-step-latency.py` | `profiling/` |
| `run-canonical-wave1.sh`, `run-canonical-wave2.sh`, `run-canonical-transport-block.sh`, `update-offered-loads.py` | `campaigns/` |
| `run-evaluations.py`, `analyze-evaluations.py`, `sync-thesis-data.py`, `smoke.sh` | unchanged (entry points stay in the example dir) |
| `plot-antenna-pattern.py`, `plot-three-gpp-antenna.py`, `best-checkpoint.py`, `continue-training.py` | `obsolete/` (retired same day, see the table above) |

The subdirectory scripts resolve the example directory themselves, so they can
be run from anywhere; `python3 run-evaluations.py <cfg>` still runs from the
example directory. `evaluation-scenarios/baseline/run-baselines.sh` stays next
to the baseline configs.
