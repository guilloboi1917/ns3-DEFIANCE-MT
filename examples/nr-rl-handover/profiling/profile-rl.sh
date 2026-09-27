#!/usr/bin/env bash
# Perf profile of the RL-mode (rlMode=true) workload, logging off and on.
# The RL-vs-A3 walltime delta is the synchronous per-step python round trip.
# Same perf prerequisites as profiling/profile-perf.sh.
#
# Usage: ./profiling/profile-rl.sh [checkpoint-dir]

set -eu
if [ -n "${BASH_VERSION:-}" ]; then
    set -o pipefail
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../output/perf-rl"
VENV_BIN="/home/nisaak/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin"
RUN_AGENT="$VENV_BIN/run-agent"
CKPT="${1:-$HERE/../checkpoints/PPO_2026-09-07_13-52-01/}"
ENV_NAME="defiance-nr-rl-handover"

if ! command -v perf >/dev/null 2>&1; then
    echo "[ERROR] perf not found (see profiling/profile-perf.sh header for the WSL2 build)."
    exit 1
fi
PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 2)
if [ "$PARANOID" -gt 0 ]; then
    echo "[ERROR] perf_event_paranoid=$PARANOID blocks user-space sampling."
    echo "        Run: sudo sysctl -w kernel.perf_event_paranoid=-1"
    exit 1
fi
if [ ! -x "$RUN_AGENT" ]; then
    echo "[ERROR] run-agent not found: $RUN_AGENT (poetry venv moved?)"
    exit 1
fi
if [ ! -d "$CKPT" ]; then
    echo "[ERROR] checkpoint dir not found: $CKPT (pass one as \$1)"
    exit 1
fi

mkdir -p "$OUT"

BASE=(simDuration=20 flowDirection=dl transportProtocol=udp \
      addInterferingUes=4 aerialUeRatio=1.0 topology=triangle \
      bandwidthMhz=10 errorModel=eesm-ir-t1 channelUpdateMs=50 \
      trafficRateMbps=100 seed=10 runId=1 \
      rlMode=true handoverAlgorithm=agent parallel=0)

for mode in off on; do
    [ "$mode" = on ] && LOG=true || LOG=false
    RUN_DIR="$OUT/run-log-$mode"
    rm -rf "$RUN_DIR"; mkdir -p "$RUN_DIR"
    DATA="$OUT/perf-rl-log-$mode.data"
    TRIAL="perf-rl-log-$mode"
    # Each k=v must be its own argv element after -c (ParseKwargs nargs="*").
    set -- "${BASE[@]}" outputDir="$RUN_DIR" logging=$LOG trial_name="$TRIAL"

    echo "== profiling RL-mode (logging=$mode): $DATA =="
    T0=$(date +%s%N)
    perf record -o "$DATA" --call-graph dwarf -F 999 \
        -e cycles,cache-misses,branch-misses \
        "$RUN_AGENT" infer -n "$ENV_NAME" -a "$CKPT" -c "$@"
    T1=$(date +%s%N)
    WALL=$(awk "BEGIN { printf \"%.2f\", ($T1 - $T0) / 1e9 }")
    echo "== RL-mode (logging=$mode) walltime: ${WALL} s =="

    echo "== perf report (logging=$mode, sim dso) =="
    # Filter to the simulator code so the python/torch frames (a few %) do
    # not clutter the list. The sim's symbols live in the ns3 shared modules
    # (libns3-dev-*.so) plus the example binary (obs/act/rwd apps).
    perf report -i "$DATA" --stdio --no-children --sort=symbol \
        --dsos=libns3-dev,ns3-dev-defiance-nr-rl-handover-optimized 2>/dev/null \
        | head -45 > "$OUT/perf-rl-log-$mode.txt"
    cat "$OUT/perf-rl-log-$mode.txt"
done

cat <<EOF

Done. Data files in $OUT:
  hotspot $OUT/perf-rl-log-off.data
  hotspot $OUT/perf-rl-log-on.data
Compare against the A3-mode baselines (profiling/profile-perf.sh, same workload):
  logging off: 25.7 s | logging on: 27.5 s
The RL-mode - A3-mode delta is the synchronous python round trip per step
(quantify with: python3 profiling/rl-step-latency.py)
EOF
