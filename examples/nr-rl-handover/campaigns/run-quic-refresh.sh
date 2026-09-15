#!/usr/bin/env bash
# Re-run every QUIC evaluation cell after the handshake fixes and the 1.5 s
# QUIC flow start (BUGS-ISSUES #36). Unattended; logs to results/_logs/.
#
# 20 seeds (the canonical count); the existing QUIC cells are overwritten.
# TCP/UDP cells are untouched: the transport-comparison configs are filtered to
# their bw30-n1_quic tag with --tags.
#
# Tunables: JOBS, TIMEOUT, DRY_RUN=1, WAIT_FOR_IDLE=1.
# Usage: ./campaigns/run-quic-refresh.sh

set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"
cd "$ROOT"

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

# "<config>|<tags>" -- empty tags means all tags in the config.
JOBS_LIST=(
    "evaluation-scenarios/rl/agent-eval-ul-no-if-quic.yaml|"
    "evaluation-scenarios/rl/agent-eval-ul-if-quic.yaml|"
    "evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul.yaml|"
    "evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul-m5t256.yaml|"
    "evaluation-scenarios/baseline/transport-comparison.yaml|bw30-n1_quic"
    "evaluation-scenarios/baseline/transport-comparison-load-matched.yaml|bw30-n1_quic"
)

LOGDIR="results/_logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/quic-refresh-$STAMP.log"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "QUIC refresh -- started $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="

# ---------------------------------------------------------------- preflight
fail=0
for entry in "${JOBS_LIST[@]}"; do
    cfg="${entry%%|*}"
    [[ -e "$cfg" ]] || { echo "MISSING: $cfg"; fail=1; }
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

echo
echo "--- provenance ----------------------------------------------------"
echo "ns-3      : $(git -C "$NS3ROOT" describe --always --dirty 2>/dev/null || echo 'n/a')"
echo "defiance  : $(git -C "$NS3ROOT/contrib/defiance" describe --always --dirty 2>/dev/null || echo 'n/a')"
echo "quic      : $(git -C "$NS3ROOT/contrib/quic" describe --always --dirty 2>/dev/null || echo 'n/a')"
echo "QUIC start: source 1.5 s / sink 1.45 s / trace hookup 1.6 s"
echo "jobs      : $JOBS   per-seed timeout: ${TIMEOUT}s"

# --------------------------------------------------------------- run cells
declare -a RCS=()
i=0
for entry in "${JOBS_LIST[@]}"; do
    i=$((i + 1))
    cfg="${entry%%|*}"
    tags="${entry#*|}"
    name="$(basename "$cfg" .yaml)"
    tag_flag=(); [[ -n "$tags" ]] && tag_flag=(--tags "$tags")
    echo
    echo "=== [$i/${#JOBS_LIST[@]}] $name ${tags:+($tags)}   $(date '+%H:%M:%S') ==="
    python3 -u run-evaluations.py "$cfg" "${tag_flag[@]}" --jobs "$JOBS" \
        --analyze --timeout "$TIMEOUT" $DRY_FLAG
    rc=$?
    RCS+=("$name${tags:+:$tags}=$rc")
    [[ $rc -eq 0 ]] || echo "WARNING: $name exited $rc"
done

echo
echo "==================================================================="
echo "QUIC refresh finished $(date '+%Y-%m-%d %H:%M:%S')"
printf 'per-config rc: %s\n' "${RCS[*]}"
echo "log : $LOG"
echo "next: check that no QUIC seed has an empty goodputMbps, then run"
echo "      python3 sync-thesis-data.py and refresh the QUIC numbers in the"
echo "      KnowledgeBase (quic-handshake-defects.md)."
echo "==================================================================="
