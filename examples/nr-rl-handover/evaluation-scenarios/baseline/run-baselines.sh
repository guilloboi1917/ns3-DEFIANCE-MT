#!/usr/bin/env bash
# run-baselines.sh — run every A3 baseline config (RL-comparison UL, DL
# future-work reference, UDP matrix, TCP-vs-UDP) with analysis.
# Usage: ./run-baselines.sh [JOBS]   (default 5); DRY_RUN=1 to preview only.
# Failing seeds do not stop the campaign; the runner exits non-zero per
# config, so each config runs and reports its own status.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/../../../../../.." && pwd)"
export NS3_HOME="$ROOT"
export PATH="$ROOT:$PATH"
JOBS="${1:-5}"
RUNNER="$ROOT/contrib/defiance/examples/nr-rl-handover/run-evaluations.py"
CFGS="$ROOT/contrib/defiance/examples/nr-rl-handover/evaluation-scenarios/baseline"
EXTRA=""; [[ "${DRY_RUN:-0}" == "1" ]] && EXTRA="--dry-run"
# Results resolve relative to the cwd; run from the example dir so they land
# in .../nr-rl-handover/results/ next to the other campaigns.
cd "$ROOT/contrib/defiance/examples/nr-rl-handover"

for cfg in nr-rl-handover-a3-baseline-10mhz-udp-ul.yaml \
           nr-rl-handover-a3-baseline-10mhz-udp-dl.yaml \
           mtx-ul.yaml \
           tcp-vs-udp.yaml
do
    echo "=== $cfg ==="
    python3 "$RUNNER" "$CFGS/$cfg" --jobs "$JOBS" --analyze $EXTRA \
        && echo "[ok] $cfg" || echo "[FAILED rc=$?] $cfg"
done
