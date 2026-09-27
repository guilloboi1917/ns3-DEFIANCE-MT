# campaigns/ - local campaign drivers

These run drivers are **local scaffolding and are not committed** (this
directory is gitignored apart from this README). Every driver writes a separate
dataset; never merge them, because the effective gNB beamforming differs per
dataset and `directpath` means different things on the unpatched and patched nr
builds.

| dataset | nr build | effective gNB method | output |
|---|---|---|---|
| campaign (quoted until 2026-09-18) | `nr-v5.1-local` @ `a75668ae` | `directpath` while the UE stays on its first cell, quasi-omni after | `results/`, synced to `EvaluationData/` |
| uniformly unsteered rerun | `nr-v5.1-local` @ `a75668ae` | `quasiomni` throughout | `results-unsteered/` |
| beamforming ablation | `bf-handover-probe` @ `54caa12a` | per cell ON/OFF, uniform over the episode | `results/a3-bf-onoff/` |

Drivers: `run-canonical-wave{1,2}.sh`, `run-canonical-transport-block.sh`,
`run-unsteered-recheck.sh`, `run-quic-refresh.sh`, `run-seed-extension.sh`,
`run-fixed-arm-cells.sh`, `run-native-arm-cells.sh`, `run-native-cp-ladder.sh`
(exploratory), `update-offered-loads.py`.

`run-unsteered-recheck.sh` reruns the quoted cells with the whole episode
unsteered. It passes the canonical configs unchanged and reads the effective
`beamformingMethod` back from each seed's `meta.yaml`, so run it on the
unpatched `nr-v5.1-local` build. A full run takes ~3.5 h and is resume-safe: a
cell counts as complete only when every seed has a clean `seed_*/run-info.yaml`.
