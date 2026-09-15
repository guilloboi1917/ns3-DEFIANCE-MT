#!/usr/bin/env bash
# Regenerate the permutation feature-importance summaries for the canonical
# cells (one checkpoint at a time, so the transport/regime effect on the
# policy's information use is visible). Writes feature-importance.csv,
# feature-importance-summary.md and feature-importance-avg.png per cell.
#
# Usage: ./analysis/run-feature-importance.sh [SEEDS=5 REPEAT=2 LIMIT=0.2 ...]

set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT"

VENV="$(ls -d "$HOME"/.cache/pypoetry/virtualenvs/ns-defiance-*/ 2>/dev/null | head -1)"
[[ -n "$VENV" ]] || { echo "no ns-defiance poetry venv found"; exit 1; }
PY="$VENV/bin/python"

SEEDS="${SEEDS:-20}"
REPEAT="${REPEAT:-3}"
LIMIT="${LIMIT:-0.5}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export MKL_NUM_THREADS="$OMP_NUM_THREADS"

LOGDIR="results/_logs"
mkdir -p "$LOGDIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/feature-importance-$STAMP.log"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "feature importance -- started $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG   (seeds=$SEEDS repeat=$REPEAT limit=$LIMIT)"
echo "==================================================================="

run_cell() {
    local ckpt="$1" cell="$2" note="$3"
    if [[ ! -d "$cell" ]]; then
        echo
        echo "=== SKIP $cell (does not exist) ==="
        return
    fi
    echo
    echo "=== $cell   $(date '+%H:%M:%S') ==="
    "$PY" "$HERE/feature-importance-summary.py" \
        --checkpoint "$ckpt" --cell-dir "$cell" --note "$note" \
        --seeds "$SEEDS" --repeat "$REPEAT" --limit "$LIMIT" --python "$PY" \
        || echo "WARNING: $cell failed"
}

run_cell "$ROOT/checkpoints/PPO_2026-09-07_13-52-01" \
    "results/agent-eval-ul-no-if-tcp-beta5/triangle-ul-no-if-tcp-beta5" \
    "TCP, rlBetaHandover 5.0, clean regime (addInterferingUes=0)"
run_cell "$ROOT/checkpoints/PPO_2026-09-02_20-53-50" \
    "results/agent-eval-ul-if-tcp/triangle-ul-if-tcp" \
    "TCP, native interference regime (addInterferingUes=4)"
run_cell "$ROOT/checkpoints/PPO_2026-09-07_13-52-01" \
    "results/agent-eval-ul-no-if-quic/triangle-ul-no-if-quic" \
    "QUIC, rlBetaHandover 5.0, clean regime (addInterferingUes=0)"
run_cell "$ROOT/checkpoints/PPO_2026-09-07_13-52-01" \
    "results/agent-eval-ul-if-quic/triangle-ul-if-quic" \
    "QUIC, rlBetaHandover 5.0, interference regime (addInterferingUes=4)"

echo
echo "==================================================================="
echo "feature importance finished $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="
