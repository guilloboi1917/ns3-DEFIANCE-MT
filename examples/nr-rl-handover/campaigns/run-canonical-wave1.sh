#!/usr/bin/env bash
# Canonical-regime headline wave, unattended. Wave 2 is a separate script.
# Cells: A3 anchors (TCP std/tuned, QUIC), transport comparison, tcp-variants,
# the RL core cells and the RL+QUIC leg. Logs to
# results/_logs/canonical-wave1-<timestamp>.log.
#
# Tunables: JOBS, TIMEOUT, DRY_RUN=1, WAIT_FOR_IDLE=1.
# Usage: ./campaigns/run-canonical-wave1.sh

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

CFGS=(
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-tcp-ul.yaml
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-tcp-ul-m5t256.yaml
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul.yaml
    evaluation-scenarios/baseline/transport-comparison.yaml
    evaluation-scenarios/baseline/tcp-variants.yaml
    evaluation-scenarios/rl/agent-eval-ul-no-if-tcp.yaml
    evaluation-scenarios/rl/agent-eval-ul-no-if-tcp-beta5.yaml
    evaluation-scenarios/rl/agent-eval-ul-if-tcp.yaml
    evaluation-scenarios/rl/agent-eval-ul-if-tcp-transfer.yaml
    evaluation-scenarios/rl/agent-eval-ul-if-tcp-beta5-transfer.yaml
    evaluation-scenarios/rl/agent-eval-ul-no-if-quic.yaml
)

LOGDIR="results/_logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/canonical-wave1-$STAMP.log"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "canonical WAVE 1 (headline) -- started $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="

# ---------------------------------------------------------------- preflight
fail=0
for f in "${CFGS[@]}"; do
    [[ -e "$f" ]] || { echo "MISSING: $f"; fail=1; }
    [[ "$(head -1 "$f")" == "# REGIME: canonical"* ]] || echo "NOTE: $f has no REGIME marker"
done
[[ -x "$NS3ROOT/ns3" ]] || { echo "MISSING: $NS3ROOT/ns3"; fail=1; }
command -v run-agent >/dev/null 2>&1 || { echo "MISSING: run-agent (activate the ns-defiance poetry venv)"; fail=1; }
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
echo "nr module : $(git -C "$NS3ROOT/contrib/nr" describe --always --dirty 2>/dev/null || echo 'n/a')"
echo "defiance  : $(git -C "$NS3ROOT/contrib/defiance" describe --always --dirty 2>/dev/null || echo 'n/a')"
echo "canonical : rlcTxBufferBytes 2 MiB / tcpSndBufBytes 1 MiB / tcpMinRtoMs 200 /"
echo "            tcpDelAckTimeoutMs 40 / tcpDelAckCount 2   (also the CLI defaults)"
echo "jobs      : $JOBS   per-seed timeout: ${TIMEOUT}s"

# --------------------------------------------------------------- run cells
declare -a RCS=()
i=0
for cfg in "${CFGS[@]}"; do
    i=$((i + 1))
    name="$(basename "$cfg" .yaml)"
    echo
    echo "=== [$i/${#CFGS[@]}] $name   $(date '+%H:%M:%S') ==="
    python3 run-evaluations.py "$cfg" --jobs "$JOBS" --analyze --timeout "$TIMEOUT" $DRY_FLAG
    rc=$?
    RCS+=("$name=$rc")
    [[ $rc -eq 0 ]] || echo "WARNING: $name exited $rc (failed seeds are marked in run-info.yaml)"
done

# ---------------------------------------------------------------- summary
echo
echo "--- summary $(date '+%H:%M:%S')"
python3 - "${CFGS[@]}" <<'PY'
import csv, sys
from pathlib import Path

def agg(cell):
    p = cell / "aggregate.csv"
    if not p.exists():
        return None
    return {r["metric"]: (r.get("p50") or r.get("mean"))
            for r in csv.DictReader(open(p))}

def fmt(d, k, w=8, prec=2):
    try:
        v = float(d.get(k, "nan"))
    except (TypeError, ValueError):
        return f"{'-':>{w}}"
    return f"{v:{w}.{prec}f}" if v == v else f"{'-':>{w}}"

print(f"\n{'evaluation/cell':<46} {'goodput':>8} {'dly ms':>8} {'loss %':>7} "
      f"{'RTT p50':>8} {'HOs':>5} {'retx':>7}")
for cfg in sys.argv[1:]:
    name = Path(cfg).stem
    root = Path("results") / name
    if not root.is_dir():
        continue
    for cell in sorted(d for d in root.iterdir() if d.is_dir()):
        a = agg(cell)
        if not a:
            continue
        print(f"{name + '/' + cell.name:<46} {fmt(a,'goodputMbps')} "
              f"{fmt(a,'delayMs_avg',8,1)} {fmt(a,'e2eLossPct',7)} "
              f"{fmt(a,'rttMs_p50',8,1)} {fmt(a,'handovers',5,0)} "
              f"{fmt(a,'retransmissions',7,0)}")
print("\nNote: retransmissions is not comparable across transports (TCP counts")
print("segments, QUIC counts loss-triggered events).")
PY

echo
echo "==================================================================="
echo "WAVE 1 finished $(date '+%Y-%m-%d %H:%M:%S')"
printf 'per-config rc: %s\n' "${RCS[*]}"
echo "log    : $LOG"
echo "next   : campaigns/run-canonical-wave2.sh   (support blocks, ~1.6 h)"
echo "==================================================================="
