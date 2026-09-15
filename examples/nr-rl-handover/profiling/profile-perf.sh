#!/usr/bin/env bash
# Perf profile of the canonical DL-if A3 workload, logging off and on.
# Writes the perf .data files (Hotspot) and filtered .txt reports to
# output/perf/. Requires perf from the WSL2 kernel and perf_event_paranoid=-1.
#
# Usage: ./profiling/profile-perf.sh

set -eu
if [ -n "${BASH_VERSION:-}" ]; then
    set -o pipefail
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../../../.." && pwd)"      # ns-3-dev root
OUT="$HERE/../output/perf"
BIN="$ROOT/build/optimized/contrib/defiance/examples/ns3-dev-defiance-nr-rl-handover-optimized"

if ! command -v perf >/dev/null 2>&1; then
    echo "[ERROR] perf not found. Build it from the WSL2 kernel source first"
    echo "        (see the header comment)."
    exit 1
fi
PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo 2)
if [ "$PARANOID" -gt 0 ]; then
    echo "[ERROR] perf_event_paranoid=$PARANOID blocks user-space sampling."
    echo "        Run: sudo sysctl -w kernel.perf_event_paranoid=-1"
    exit 1
fi
if [ ! -x "$BIN" ]; then
    echo "[ERROR] optimized binary not found: $BIN"
    exit 1
fi

mkdir -p "$OUT"

set -- --simDuration=20 --flowDirection=dl --transportProtocol=udp \
      --addInterferingUes=4 --aerialUeRatio=1.0 --topology=triangle \
      --bandwidthMhz=10 --errorModel=eesm-ir-t1 --channelUpdateMs=50 \
      --trafficRateMbps=100 --seed=10 --runId=1 \
      --rlMode=false --handoverAlgorithm=a3 --parallel=1

for mode in off on; do
    [ "$mode" = on ] && LOG=true || LOG=false
    RUN_DIR="$OUT/run-log-$mode"
    rm -rf "$RUN_DIR"; mkdir -p "$RUN_DIR"
    DATA="$OUT/perf-dlif-log-$mode.data"
    echo "== profiling (logging=$mode): $DATA =="
    perf record -o "$DATA" --call-graph dwarf -F 999 \
        -e cycles,cache-misses,branch-misses \
        "$BIN" "$@" --outputDir="$RUN_DIR" --logging=$LOG
    echo "== perf report (logging=$mode) =="
    perf report -i "$DATA" --stdio --no-children \
        --sort=symbol 2>/dev/null | head -40 \
        > "$OUT/perf-report-log-$mode.txt"
    cat "$OUT/perf-report-log-$mode.txt"
done

cat <<EOF

Done. Data files in $OUT:
  hotspot $OUT/perf-dlif-log-off.data
  hotspot $OUT/perf-dlif-log-on.data
(baseline walltime for this workload: 25.7 s logging off vs 35.1 s logging on)
EOF
