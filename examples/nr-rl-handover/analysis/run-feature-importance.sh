#!/usr/bin/env bash
# Regenerate the permutation feature-importance summaries for the three native
# arms of the corrected (post-interface-fix, uniformly unsteered) RL evaluation:
#
#   clean TCP            PPO_2026-09-20_13-59-15   native clean regime
#   interfered TCP       PPO_2026-09-21_12-07-15   native interference regime
#   interfered QUIC      PPO_2026-09-21_12-07-15   native interference regime
#
# Writes feature-importance.csv, feature-importance-summary.md and
# feature-importance-avg.png per cell.
#
# 2026-09-23: repointed from the retired steered-campaign cells
# (results/agent-eval-ul-{no-if-tcp,if-tcp,no-if-quic,if-quic}-beta5) on the
# pre-fix checkpoint PPO_2026-09-07_13-52-01, which no longer exist. The old
# set held one checkpoint across four cells; the current set is native-per-cell
# (each cell evaluated with the checkpoint trained for it), so the columns are
# not a checkpoint-controlled comparison - that comparison is the zero-shot
# transfer cells, which are not part of this table.
#
# Usage: ./analysis/run-feature-importance.sh [SEEDS=20 REPEAT=3 LIMIT=0.5 ...]

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

run_cell "$HOME/ray_results/PPO_2026-09-20_13-59-15" \
    "results-unsteered/agent-eval-ul-no-if-tcp-beta5-fixed/triangle-ul-no-if-tcp-fixed" \
    "TCP, clean regime, native (checkpoint PPO_2026-09-20_13-59-15)"

run_cell "$HOME/ray_results/PPO_2026-09-21_12-07-15" \
    "results-unsteered/agent-eval-ul-if-tcp-beta5-fixed-native/triangle-ul-if-tcp-fixed-native" \
    "TCP, four aerial interferers, native (checkpoint PPO_2026-09-21_12-07-15)"

run_cell "$HOME/ray_results/PPO_2026-09-21_12-07-15" \
    "results-unsteered/agent-eval-ul-if-quic-beta5-fixed-native/triangle-ul-if-quic-fixed-native" \
    "QUIC, four aerial interferers, native (checkpoint PPO_2026-09-21_12-07-15)"

echo
echo "==================================================================="
echo "feature importance finished $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="
