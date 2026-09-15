#!/usr/bin/env bash
# Extend selected evaluation cells with seeds 21-30, without re-running 1-20.
# Unattended; logs to results/_logs/seed-extension-<timestamp>.log.
#
# Sets (SET=...):
#   quic  (default) the four QUIC cells: RL no-if, RL if, A3 quic-ul-no-if,
#         A3 quic-ul-if. A replication of the transport-robustness QUIC legs.
#   2x2   the full transport-robustness block: the four QUIC cells plus the
#         four TCP cells (RL beta5 no-if, RL beta5 if-transfer, A3 tcp-ul-no-if,
#         A3 tcp-ul-if), so the regime x transport design keeps one seed set.
#
# Tunables: JOBS, SEED_START=21, N_SEEDS=30, TIMEOUT, DRY_RUN=1, WAIT_FOR_IDLE=1.
# Usage: ./campaigns/run-seed-extension.sh
#        SET=2x2 JOBS=4 ./campaigns/run-seed-extension.sh

set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"
cd "$ROOT"

SET="${SET:-quic}"
SEED_START="${SEED_START:-21}"
N_SEEDS="${N_SEEDS:-30}"
JOBS="${JOBS:-5}"
TIMEOUT="${TIMEOUT:-3600}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_FOR_IDLE="${WAIT_FOR_IDLE:-0}"
DRY_FLAG=""; [[ "$DRY_RUN" == "1" ]] && DRY_FLAG="--dry-run"

# RL configs shell out to run-agent; pick up the poetry venv when not active.
if ! command -v run-agent >/dev/null 2>&1; then
    for v in "$HOME"/.cache/pypoetry/virtualenvs/ns-defiance-*/bin; do
        [[ -x "$v/run-agent" ]] && { export PATH="$v:$PATH"; break; }
    done
fi
export NS3_HOME="$NS3ROOT"
export PATH="$NS3ROOT:$PATH"

QUIC_CFGS=(
    evaluation-scenarios/rl/agent-eval-ul-no-if-quic.yaml
    evaluation-scenarios/rl/agent-eval-ul-if-quic.yaml
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul.yaml
)
TCP_CFGS=(
    evaluation-scenarios/rl/agent-eval-ul-no-if-tcp-beta5.yaml
    evaluation-scenarios/rl/agent-eval-ul-if-tcp-beta5-transfer.yaml
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-tcp-ul.yaml
)

case "$SET" in
    quic) CFGS=("${QUIC_CFGS[@]}") ;;
    2x2)  CFGS=("${QUIC_CFGS[@]}" "${TCP_CFGS[@]}") ;;
    *)    echo "unknown SET=$SET (use quic or 2x2)"; exit 1 ;;
esac

LOGDIR="results/_logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/seed-extension-$STAMP.log"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "seed extension ($SET) -- started $(date '+%Y-%m-%d %H:%M:%S')"
echo "seeds $SEED_START..$N_SEEDS | log: $LOG"
echo "==================================================================="

# ---------------------------------------------------------------- preflight
fail=0
for f in "${CFGS[@]}"; do
    [[ -e "$f" ]] || { echo "MISSING: $f"; fail=1; }
done
[[ -x "$NS3ROOT/ns3" ]] || { echo "MISSING: $NS3ROOT/ns3"; fail=1; }
targets="$("$NS3ROOT/ns3" show targets 2>/dev/null || true)"
grep -q "defiance-nr-rl-handover" <<<"$targets" \
    || { echo "MISSING: build target defiance-nr-rl-handover"; fail=1; }
[[ $fail -eq 0 ]] || { echo "preflight failed -- nothing was run"; exit 1; }

running_sims=$(pgrep -fc 'defiance-nr-rl-handover' || true)
echo "running defiance sims right now: ${running_sims:-0}"
if [[ "${running_sims:-0}" -gt 0 ]]; then
    if [[ "$WAIT_FOR_IDLE" == "1" ]]; then
        echo "WAIT_FOR_IDLE=1 -> waiting for them to finish ..."
        while [[ "$(pgrep -fc 'defiance-nr-rl-handover' || true)" -gt 0 ]]; do sleep 60; done
        echo "machine idle, starting"
    else
        echo "WARNING: other simulations are running; $JOBS more jobs may oversubscribe"
        echo "         the 16 cores (set JOBS=4 or WAIT_FOR_IDLE=1 to be polite)"
    fi
fi

# --------------------------------------------------------------- run cells
declare -a RCS=()
i=0
for cfg in "${CFGS[@]}"; do
    i=$((i + 1))
    name="$(basename "$cfg" .yaml)"
    echo
    echo "=== [$i/${#CFGS[@]}] $name   $(date '+%H:%M:%S') ==="
    python3 -u run-evaluations.py "$cfg" --jobs "$JOBS" --n-seeds "$N_SEEDS" \
        --seed-start "$SEED_START" --analyze --timeout "$TIMEOUT" $DRY_FLAG
    rc=$?
    RCS+=("$name=$rc")
    [[ $rc -eq 0 ]] || echo "WARNING: $name exited $rc"
done

echo
echo "==================================================================="
echo "seed extension finished $(date '+%Y-%m-%d %H:%M:%S')"
printf 'per-config rc: %s\n' "${RCS[*]}"
echo "log : $LOG"
echo "next: re-check the affected cells for seeds with empty goodput, then"
echo "      refresh the tables and KnowledgeBase numbers."
echo "==================================================================="
