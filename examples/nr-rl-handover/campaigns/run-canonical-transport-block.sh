#!/usr/bin/env bash
# Canonical-regime transport block in one unattended pass:
#   1 capacity probe (B0), 2 derive the offered loads, 3 transport comparison
#   (B1), 4 summary (goodput/delay/loss/handovers + QUIC sanity checks).
# Logs to results/_logs/canonical-transport-<timestamp>.log.
#
# Tunables: JOBS, JOBS_PROBE, TIMEOUT, WAIT_FOR_IDLE=1, DRY_RUN=1, SKIP_PROBE=1.
# Usage: ./campaigns/run-canonical-transport-block.sh

set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"
cd "$ROOT"

RUN_FLAG=""; DRY_FLAG=""
if [[ "${DRY_RUN:-0}" == "1" ]]; then RUN_FLAG="--dry-run"; DRY_FLAG="--dry-run"; fi

JOBS="${JOBS:-5}"
JOBS_PROBE="${JOBS_PROBE:-5}"
TIMEOUT="${TIMEOUT:-3600}"
WAIT_FOR_IDLE="${WAIT_FOR_IDLE:-0}"
SKIP_PROBE="${SKIP_PROBE:-0}"
DRY_RUN="${DRY_RUN:-0}"

PROBE_CFG="evaluation-scenarios/baseline/capacity-probe-canonical.yaml"
TRANSPORT_CFG="evaluation-scenarios/baseline/transport-comparison-load-matched.yaml"
PROBE_RES="results/capacity-probe-canonical"
TRANSPORT_RES="results/transport-comparison-load-matched"
LOGDIR="results/_logs"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$LOGDIR/canonical-transport-$STAMP.log"
mkdir -p "$LOGDIR"
exec > >(tee -a "$LOG") 2>&1

echo "==================================================================="
echo "canonical transport block — started $(date '+%Y-%m-%d %H:%M:%S')"
echo "log: $LOG"
echo "==================================================================="

# ---------------------------------------------------------------- preflight
fail=0
for f in "$PROBE_CFG" "$TRANSPORT_CFG" "$HERE/update-offered-loads.py"; do
    [[ -e "$f" ]] || { echo "MISSING: $f"; fail=1; }
done
[[ -x "$NS3ROOT/ns3" ]] || { echo "MISSING: $NS3ROOT/ns3"; fail=1; }
# NOTE: capture first - `ns3 show targets | grep -q` breaks under `set -o pipefail`
# because grep -q exits early (SIGPIPE to ns3) and the pipeline then reports 141.
targets="$("$NS3ROOT/ns3" show targets 2>/dev/null || true)"
if ! grep -q "defiance-nr-rl-handover" <<<"$targets"; then
    echo "MISSING: build target defiance-nr-rl-handover (run: ./ns3 build defiance-nr-rl-handover)"
    fail=1
fi
[[ $fail -eq 0 ]] || { echo "preflight failed — nothing was run"; exit 1; }

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
echo "canonical : tcpSndBufBytes 1 MiB / rlcTxBufferBytes 2 MiB / tcpMinRtoMs 200 /"
echo "            tcpDelAckTimeoutMs 40 / tcpDelAckCount 2  (arm G, adopted 2026-09-08)"
echo "jobs      : probe=$JOBS_PROBE  transport=$JOBS  per-seed timeout=${TIMEOUT}s"

# ------------------------------------------------------------- step 1: probe
probe_rc="skipped"
transport_rc="not-run"
if [[ "$SKIP_PROBE" == "1" ]]; then
    echo
    echo "--- step 1/4 SKIPPED (SKIP_PROBE=1): using the offered loads in $TRANSPORT_CFG"
else
    echo
    echo "--- step 1/4 capacity probe (B0, canonical regime) $(date '+%H:%M:%S')"
    echo "    expecting about 10 min (3 bands x 3 seeds x 50 s)"
    python3 run-evaluations.py "$PROBE_CFG" --jobs "$JOBS_PROBE" --analyze --timeout "$TIMEOUT" $RUN_FLAG
    probe_rc=$?
    [[ $probe_rc -eq 0 ]] || echo "WARNING: probe exited $probe_rc (partial results kept)"

    # ------------------------------------------------- step 2: offered loads
    echo
    echo "--- step 2/4 derive offered loads from the probe $(date '+%H:%M:%S')"
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "    skipped in DRY_RUN (no probe results exist yet)"
    else
        python3 "$HERE/update-offered-loads.py" --probe-results "$PROBE_RES" --config "$TRANSPORT_CFG" \
            || echo "WARNING: offered-load derivation failed; keeping the loads already in the config"
    fi
fi

# --------------------------------------------------- step 3: transport block
echo
echo "--- step 3/4 transport comparison (B1, canonical regime) $(date '+%H:%M:%S')"
echo "    expecting 1.5-3 h (6 cells x 20 seeds x 50 s; 2 of the cells are QUIC)"
python3 run-evaluations.py "$TRANSPORT_CFG" --jobs "$JOBS" --analyze --timeout "$TIMEOUT" $RUN_FLAG
transport_rc=$?
[[ $transport_rc -eq 0 ]] || echo "WARNING: transport block exited $transport_rc (failed seeds are marked in run-info.yaml)"

# ------------------------------------------------------------- step 4: summary
echo
echo "--- step 4/4 summary $(date '+%H:%M:%S')"
python3 - "$TRANSPORT_RES" "$PROBE_RES" <<'PY'
import csv, os, statistics, sys
from pathlib import Path

transport, probe = Path(sys.argv[1]), Path(sys.argv[2])

def agg(tag_dir):
    p = tag_dir / "aggregate.csv"
    if not p.exists():
        return None
    rows = list(csv.DictReader(open(p)))
    return {r["metric"]: r.get("p50") or r.get("mean") for r in rows}

print("\nB0 ceilings (canonical probe, median delivered goodput):")
for d in sorted(probe.glob("bw*")):
    a = agg(d)
    if a:
        print(f"  {d.name:<10} {float(a.get('goodputMbps', 'nan')):6.2f} Mbps")

def col(d):
    p = d / "sink-packets.csv"
    if not p.exists():
        return None
    t = []
    with open(p) as f:
        r = csv.reader(f); next(r, None)
        for row in r:
            if len(row) >= 2:
                try: t.append(float(row[0]))
                except ValueError: pass
    return t

def drops(d):
    p = d / "nr-rl-rlc-tx-drop.csv"
    return sum(1 for _ in open(p)) - 1 if p.exists() else 0

print("\nB1 transport comparison (canonical regime, median over seeds):")
print(f"  {'cell':<14} {'goodput':>8} {'1-way dly':>10} {'e2e loss':>9} {'RTT p50':>8} {'HOs':>5} {'drops':>6} {'last pkt':>9}")
for d in sorted(transport.glob("bw*")):
    a = agg(d)
    if not a:
        continue
    seeds = sorted(d.glob("seed_*"))
    lat = [col(s) for s in seeds]
    last = [t[-1] for t in lat if t]
    dp = [drops(s) for s in seeds if s.is_dir()]
    def show(k, w, prec=2):
        try:
            v = float(a.get(k, "nan"))
        except (TypeError, ValueError):
            return f"{'-':>{w}}"
        return f"{v:{w}.{prec}f}" if v == v else f"{'-':>{w}}"
    print(f"  {d.name:<14} {show('goodputMbps', 8)} {show('delayMs_avg', 10, 1)} "
          f"{show('e2eLossPct', 9)} {show('rttMs_p50', 8, 1)} {show('handovers', 5, 0)} "
          f"{statistics.median(dp) if dp else 0:6.0f} {statistics.median(last) if last else 0:9.1f}")
print("\nQUIC check: RLC drops must be ~0 and 'last pkt' ~50 s (a wedge would show")
print("drops in the hundreds and a last packet near the freeze time).")
PY

echo
echo "==================================================================="
echo "finished $(date '+%Y-%m-%d %H:%M:%S')  (probe rc=${probe_rc:-skipped} / transport rc=${transport_rc:-not-run})"
echo "log:     $LOG"
echo "results: $TRANSPORT_RES/<cell>/seed_<N>/   ($TRANSPORT_RES/*/aggregate.csv)"
echo "figures: python3 plots/plot-nr-rl-stats.py $TRANSPORT_RES/<cell>/seed_<N> -o /tmp/seed.png"
echo "==================================================================="
