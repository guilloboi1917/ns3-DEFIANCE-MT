# Evaluation scenario archive

Superseded scenario configurations, kept for provenance. The live campaign
configurations are `../baseline/*.yaml` and `../rl/*.yaml`; every one of them
starts with a `# REGIME: canonical ...` marker and carries the canonical knobs
explicitly (RLC TX 2 MiB, TCP SndBuf 1 MiB, MinRto 200 ms, DelAck 40 ms x2).

## canonical-precursor regime (pre-2026-09-11)

All pre-canonical campaign results were retired to
`results/_archive/canonical-precursor-20260911/` on the same date. They ran
with the old defaults: de-facto RLC TX cap 180 kB (the value is absent from
the run-info.yaml commands because the knobs did not exist yet), ns-3 TCP
socket buffer 128 kB, delayed ACK 200 ms x2. The delayed-ACK/RTO race made
TCP results (especially the loss-based congestion controls) an artifact of the
timer interaction; the RLC queue also set the UDP delay/loss regime.

### `pre-canonical-20260911/`

The configuration as it stood immediately before the canonical fold-in, i.e.
the live file with the canonical knob block removed. Mirrors the live tree.

| Pre-canonical file | Produced (now archived) results |
|---|---|
| `baseline/a3-extremes-alt.yaml` | `results/_archive/canonical-precursor-20260911/a3-extremes-alt/` |
| `baseline/a3-sweep.yaml` | `.../a3-sweep/` |
| `baseline/mtx-ul.yaml` | `.../mtx-ul/` |
| `baseline/nr-rl-handover-a3-baseline-30mhz-tcp-ul.yaml` | `.../nr-rl-handover-a3-baseline-30mhz-tcp-ul/` |
| `baseline/nr-rl-handover-a3-baseline-30mhz-tcp-ul-m5t256.yaml` | `.../nr-rl-handover-a3-baseline-30mhz-tcp-ul-m5t256/` |
| `baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul.yaml` | (no completed pre-canonical QUIC cell; the 2026-09-11 attempt hung, see `results/_archive/transport-canonical-quic-hung-20260911/`) |
| `baseline/nr-rl-handover-a3-baseline-30mhz-udp-ul.yaml` | not part of the 46-cell campaign |
| `baseline/nr-rl-handover-a3-baseline-30mhz-udp-dl.yaml` | not part of the 46-cell campaign (DL out of scope) |
| `baseline/tcp-variants.yaml` | `.../tcp-variants/` |
| `baseline/tcp-vs-udp.yaml` (now `transport-comparison.yaml`) | `.../tcp-vs-udp/` |
| `baseline/topology-hexgrid.yaml` | `.../topology-hexgrid/` |
| `rl/agent-eval-ul-no-if-tcp.yaml` | `.../agent-eval-ul-no-if-tcp/` |
| `rl/agent-eval-ul-no-if-tcp-beta5.yaml` | `.../agent-eval-ul-no-if-tcp-beta5/` |
| `rl/agent-eval-ul-if-tcp.yaml` | `.../agent-eval-ul-if-tcp/` |
| `rl/agent-eval-ul-if-tcp-beta5-transfer.yaml` | `.../agent-eval-ul-if-tcp-beta5-transfer/` |

Not archived, because no pre-canonical numbered config ever existed:

- `baseline/capacity-probe.yaml` (retired; superseded by
  `baseline/capacity-probe-canonical.yaml`, which produced the live B0 cell
  `results/capacity-probe-canonical/`, 3 seeds at 10/30/50 MHz)
- `rl/agent-eval-ul-if-tcp-transfer.yaml` (recreated 2026-09-11; the results
  dir `.../agent-eval-ul-if-tcp-transfer/` existed but its source config did
  not survive the earlier scenario-file cleanup. Settings mirror
  `agent-eval-ul-if-tcp.yaml` with only the checkpoint swapped back to the
  no-if `PPO_2026-09-02_00-07-41`, matching the recorded manifest row.)
- `rl/agent-eval-ul-no-if-quic.yaml` (new 2026-09-11; QUIC leg of the RL
  transport-robustness set, smoke-tested 2/2 seeds)

### `interim-canonical-copies-20260911/`

The `*-canonical.yaml` copies created 2026-09-08..11 to carry the canonical
knobs before the fold-in. Redundant once the knobs live in the base files and
the C++ CLI defaults. Includes `baseline/transport-comparison-canonical.yaml.orig`
(a stray editor backup) and `baseline/capacity-probe.yaml`.

Also retired here 2026-09-11, both strict subsets of the 3-cell
`transport-comparison-canonical.yaml` and never run under their own name:

- `baseline/transport-comparison-30mhz-canonical.yaml` (3 cells, identical to
  the 30 MHz half)
- `baseline/transport-comparison-30mhz-canonical-quic.yaml` (1 cell; the QUIC
  cell it describes was run into `results/transport-comparison-canonical/`)

Live canonical-only configs with no pre-canonical counterpart stay in the
parent tree: `baseline/capacity-probe-canonical.yaml` (B0) and
`baseline/transport-comparison-load-matched.yaml` (load-matched cross-check).

## B1 transport-table decision (2026-09-11)

B1 is `baseline/transport-comparison.yaml`, 30 MHz n1 only, three cells: UDP
offered 45 Mbps (saturating) plus TCP and QUIC capped at the 35 Mbps RL
operating point. `baseline/transport-comparison-load-matched.yaml` is the
load-matched cross-check (all three at 43 Mbps).

**RENAMED 2026-09-11:** `baseline/tcp-vs-udp.yaml` ->
`baseline/transport-comparison.yaml` (config, `name:` field, results dir and
`EvaluationData` dir), and `baseline/transport-comparison-canonical.yaml` ->
`baseline/transport-comparison-load-matched.yaml` (results dir moved with it).
Historical references to `tcp-vs-udp` in dated docs, in the pre-canonical
archive table above, and in `results/_archive/canonical-precursor-20260911/`
keep the old name on purpose: that is what the cells were called when they ran.
During the running wave 1 a temporary symlink `tcp-vs-udp.yaml ->
transport-comparison.yaml` kept the in-flight config list resolving; it was
removed once the wave finished.

The 50 MHz cells were dropped campaign-wide: 30 MHz n1 is the canonical radio
and a second band implied a second radio config in the table. Consequence for
the thesis: `resultsdiscussion.tex:75-76` (the two 50 MHz B1 rows) and the
50 MHz half of the `resultsdiscussion.tex:83` paragraph must be removed, and
`EvaluationData` will no longer contain the `bw50-n1_{tcp,udp}` cells.

## Out of campaign (legacy, not touched)

`rl/agent-eval-ul-no-if.yaml`, `rl/agent-eval-ul-if.yaml` (UDP RL legs from
the pre-TCP scope), `baseline/tcp-variants-repaired.yaml`,
`baseline/tcp-variants-classic-recovery.yaml` (delack/classic-recovery
investigation; the latter's results are archived), `baseline/delack-race.yaml`
(KB probe, still live evidence), `rl/rl-eval-extremes-alt.yaml`,
`rl/rl-eval-hexgrid.yaml`.
