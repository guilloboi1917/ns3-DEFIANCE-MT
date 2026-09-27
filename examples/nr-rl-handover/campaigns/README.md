# campaigns/ -- run drivers for the thesis datasets

Every driver here writes a *separate* dataset. Never merge them: the effective
gNB beamforming differs per dataset (see `docs/BUGS-ISSUES.md` #38), and
`directpath` means different things on the unpatched and patched builds.

| dataset | build (`contrib/nr`) | effective gNB method | output dir |
|---|---|---|---|
| 914-run campaign (quoted until 2026-09-18) | `nr-v5.1-local` @ `a75668ae` | `directpath`: steered only while the UE stays on the cell it attached to, quasi-omni after the first handover | `results/`, synced to `EvaluationData/` |
| uniformly unsteered rerun | `nr-v5.1-local` @ `a75668ae` | `quasiomni`: quasi-omni throughout | `results-unsteered/` (sibling of `results/` in this example dir) |
| beamforming ablation (120 runs, complete) | `bf-handover-probe` @ `54caa12a` | per cell: `directpath` / `directpath-both` = ON, `quasiomni` = OFF, uniform over the episode | `results/a3-bf-onoff/` |

## Uniformly unsteered rerun (prepared 2026-09-18)

`run-unsteered-recheck.sh` reruns the cells the thesis quotes with the whole
episode unsteered. It relies on the scenario default being `quasiomni` (set on
2026-09-18 in `nr-rl-handover-scenario.cc`), passes the canonical configs
unchanged, refuses any config that sets `beamformingMethod`, and reads the
effective method back from each seed's `meta.yaml`.

Prerequisite: `contrib/nr` on the unpatched revision and a fresh build.

    cd /home/nisaak/masterthesis-ns3/ns-3-dev
    git -C contrib/nr log --oneline -1          # expect a75668ae (nr-v5.1-local)
    ./ns3 build defiance-nr-rl-handover

Smoke test (one claim-bearing pair, ~4 min):

    cd /home/nisaak/masterthesis-ns3/ns-3-dev/contrib/defiance/examples/nr-rl-handover
    bash campaigns/run-unsteered-recheck.sh --only nr-rl-handover-a3-baseline-30mhz-tcp-ul --jobs-a3 10

Full run, detached (~3.5 h; the claim-bearing pairs land in the first ~40 min):

    nohup bash campaigns/run-unsteered-recheck.sh --jobs-a3 10 --jobs-rl 6 \
        > campaigns/unsteered-run.log 2>&1 &
    tail -f campaigns/unsteered-run.log

Resume after an interruption (a cell counts as complete only when every seed
finished with a clean `seed_*/run-info.yaml`; `meta.yaml` is written at
simulation setup and would make an interrupted seed look complete):

    bash campaigns/run-unsteered-recheck.sh --jobs-a3 10 --jobs-rl 6

Flags:

- `--dry-run` print the plan without running (45 cells / 765 runs by default)
- `--only <eval-name>` one config; `--tags a,b` restrict cells
- `--jobs-a3 N` (default 10), `--jobs-rl N` (default 6; each RL job needs a Raylet)
- `--drop-load-matched` drop `transport-comparison-load-matched` (42 cells / 705 runs)
- `--keep-bw30-probe` keep only the `bw30-n1` capacity cell (46 cells / 768 runs)

Output `results-unsteered/<eval>/<tag>/seed_N` (in this example dir, alongside
`results/`) plus `aggregate.csv`, `raw.csv`, `params.yaml` per cell; logs in
`campaigns/unsteered-logs/<eval>.log`. The driver
exits non-zero if any seed does not report `quasiomni`. Spot check:

    grep -h beamformingMethod results-unsteered/*/*/seed_1/meta.yaml | sort | uniq -c

## Steered ON arm (ablation) -- patched build only

Already complete (6 tags x 20 seeds). Re-run or extend it only for steered cells,
from the ns-3-dev root:

    cd /home/nisaak/masterthesis-ns3/ns-3-dev
    git -C contrib/nr switch bf-handover-probe && ./ns3 build
    python3 contrib/defiance/examples/nr-rl-handover/run-evaluations.py \
        contrib/defiance/examples/nr-rl-handover/evaluation-scenarios/baseline/a3-bf-onoff.yaml \
        --n-seeds 20 --jobs 10 --analyze
    # switch back, or `directpath` stops reproducing the campaign dataset
    git -C contrib/nr switch nr-v5.1-local && ./ns3 build

## Known exclusions in the unsteered dataset

`nr-rl-handover-a3-baseline-30mhz-tcp-ul/tcp-ul-if` seed 8 crashes with
`NS_FATAL: Cannot TX while RX` (`nr-spectrum-phy.cc:838`) - a deterministic,
seed-dependent half-duplex edge case (`docs/BUGS-ISSUES.md` #29) that the
`directpath` campaign did not hit. The analyzer excludes it, so that cell reports
19 seeds; paired A3-vs-RL tests for it use the 19-seed intersection. Do not bump
`runId` (it would break the per-seed pairing with the RL arms).

Completion semantics: a cell counts as complete when every seed has finished
(successful or not), so a deterministic crash does not force a re-run on resume.
Failures are reported per cell and excluded by the analyzer.
