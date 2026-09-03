#!/usr/bin/env bash
# run-baselines.sh — A3 baseline campaign, B0-B4 in run order (narrative
# block map: EVALUATION-RUN-PLAN §0c / CURRENT-NEXT-STEPS). Each config
# runs with analysis; per-config failures do not stop the campaign.
# Usage: ./run-baselines.sh [JOBS]   (default 5); DRY_RUN=1 to preview only.
# The RL evals (agent-eval-*, rl-eval-*) are separate configs — run them
# with the same runner from CURRENT-NEXT-STEPS.
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

for cfg in capacity-probe.yaml \
           tcp-vs-udp.yaml \
           tcp-variants.yaml \
           a3-sweep.yaml \
           topology-hexgrid.yaml \
           mtx-ul.yaml \
           a3-extremes-alt.yaml
do
    echo "=== $cfg ==="
    python3 "$RUNNER" "$CFGS/$cfg" --jobs "$JOBS" --analyze $EXTRA \
        && echo "[ok] $cfg" || echo "[FAILED rc=$?] $cfg"
done
