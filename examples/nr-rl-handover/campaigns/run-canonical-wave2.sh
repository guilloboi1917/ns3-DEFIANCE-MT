#!/usr/bin/env bash
# Canonical-regime support wave, unattended (B0/B2/B3b/B4 blocks). Logs to
# results/_logs/canonical-wave2-<timestamp>.log.
#
# Tunables: JOBS, TIMEOUT, DRY_RUN=1, WAIT_FOR_IDLE=1.
# Usage: ./campaigns/run-canonical-wave2.sh

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

if ! command -v run-agent >/dev/null 2>&1; then
    for v in "$HOME"/.cache/pypoetry/virtualenvs/ns-defiance-*/bin; do
        [[ -x "$v/run-agent" ]] && { export PATH="$v:$PATH"; break; }
    done
fi
export NS3_HOME="$NS3ROOT"
export PATH="$NS3ROOT:$PATH"

CFGS=(
    evaluation-scenarios/baseline/nr-rl-handover-a3-baseline-30mhz-quic-ul-m5t256.yaml
    evaluation-scenarios/baseline/a3-sweep.yaml
    evaluation-scenarios/baseline/topology-hexgrid.yaml
    evaluation-scenarios/baseline/a3-extremes-alt.yaml
    evaluation-scenarios/baseline/mtx-ul.yaml
)

LOGDIR="results/_logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/canonical-wave2-$STAMP.log"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "canonical WAVE 2 (support) -- started $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="

# ---------------------------------------------------------------- preflight
fail=0
for f in "${CFGS[@]}"; do
    [[ -e "$f" ]] || { echo "MISSING: $f"; fail=1; }
    [[ "$(head -1 "$f")" == "# REGIME: canonical"* ]] || echo "NOTE: $f has no REGIME marker"
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
    python3 -u run-evaluations.py "$cfg" --jobs "$JOBS" --analyze --timeout "$TIMEOUT" $DRY_FLAG
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

print(f"\n{'evaluation/cell':<40} {'goodput':>8} {'dly ms':>8} {'loss %':>7} "
      f"{'HOs':>5} {'seeds':>6}")
for cfg in sys.argv[1:]:
    name = Path(cfg).stem
    root = Path("results") / name
    if not root.is_dir():
        continue
    for cell in sorted(d for d in root.iterdir() if d.is_dir()):
        a = agg(cell)
        if not a:
            continue
        seeds = len([d for d in cell.glob("seed_*") if d.is_dir()])
        print(f"{name + '/' + cell.name:<40} {fmt(a,'goodputMbps')} "
              f"{fmt(a,'delayMs_avg',8,1)} {fmt(a,'e2eLossPct',7)} "
              f"{fmt(a,'handovers',5,0)} {seeds:6d}")
PY

echo
echo "==================================================================="
echo "WAVE 2 finished $(date '+%Y-%m-%d %H:%M:%S')"
printf 'per-config rc: %s\n' "${RCS[*]}"
echo "log : $LOG"
echo "next: review results, then python3 sync-thesis-data.py (EvaluationData)"
echo "      and the KnowledgeBase number refresh before the .tex pass."
echo "==================================================================="
