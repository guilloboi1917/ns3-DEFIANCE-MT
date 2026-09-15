#!/usr/bin/env bash
# Canonical-configuration search (probe).
#
# The 1-segment trap is a race between the sender's RTO floor (MinRto) and the
# receiver's delayed-ACK timer (ns-3 defaults 200 ms / 2 segments): at cwnd =
# 1 MSS every ACK waits 200 ms and the RTO fires at 200 ms, so the ACK always
# loses. This searches the three candidate fixes at fixed stuck seeds.
#
# Already measured (no need to re-run):
#   rlc-buffer-sweep/180k   cap 180k mr200 dack200   baseline (trapped)
#   buffer-2d-sweep/sb1024k-rlc180k-mr500 / mr1000   sender-side fix
#   rlc-buffer-sweep/500k, /2m                       cap-only (trigger fixed,
#                                                    handover trap remains)
# Cells below:
#   A dack40-rlc180k   receiver-side fix alone (drops unchanged)
#   B dack40-rlc1m     proposed canonical (aligned cap + Linux-like ACK)
#   C mr1000-rlc1m     the untested cross the user flagged
#
# Results: results/config-search/<label>/<variant>/seed_<N>/
set -u

# Archived under obsolete/: resolve the example dir (parent) so NS3ROOT and
# results/ still point at the real locations.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../.." && pwd)"
OUTROOT="$HERE/results/config-search"
JOBS="${JOBS:-6}"
mkdir -p "$OUTROOT"
export NS3ROOT OUTROOT

# run_one <label> <sndbuf> <rlc> <minrto> <delack_ms> <delack_count> <variant> <seed>
run_one() {
    local label="$1" sndbuf="$2" rlc="$3" minrto="$4" dack="$5" dcount="$6" variant="$7" seed="$8"
    local out="$OUTROOT/$label/$variant/seed_$seed"
    mkdir -p "$out"
    "$NS3ROOT/ns3" run "defiance-nr-rl-handover \
--simDuration=50 --seed=$seed --runId=1 --topology=triangle \
--flowDirection=ul --uavMobility=random-waypoint --tcpVariant=$variant \
--startHeight=50 --endHeight=200 --addInterferingUes=0 --aerialUeRatio=0 \
--interfererMobility=static --transportProtocol=tcp --bandwidthMhz=30 \
--rlRewardRefMbps=15.0 --ueSpeed=20 --numerology=1 --trafficRateMbps=35 \
--logging=true --parallel=0 --rlMode=false --handoverAlgorithm=a3 \
--rlcTxBufferBytes=$rlc --tcpSndBufBytes=$sndbuf --tcpMinRtoMs=$minrto \
--tcpDelAckTimeoutMs=$dack --tcpDelAckCount=$dcount \
--outputDir=$out" > "$out/stdout.log" 2>&1
    echo "done $label $variant seed_$seed (exit=$?)"
}
export -f run_one

SEEDS=("TcpNewReno:1" "TcpNewReno:2" "TcpNewReno:8" "TcpNewReno:13" \
       "TcpCubic:1" "TcpCubic:2" "TcpCubic:4" "TcpCubic:5" \
       "TcpVeno:1" "TcpVeno:2" "TcpVeno:3" "TcpVeno:8" \
       "TcpBbr:1" "TcpBbr:8")

JOBS_LIST=()
add() { JOBS_LIST+=("$*"); }
for vs in "${SEEDS[@]}"; do
    add "A-dack40-rlc180k"  1048576 180000 200  40 2 "${vs%%:*}" "${vs##*:}"
    add "B-dack40-rlc1m"    1048576 1048576 200 40 2 "${vs%%:*}" "${vs##*:}"
    add "C-mr1000-rlc1m"    1048576 1048576 1000 200 2 "${vs%%:*}" "${vs##*:}"
done

printf '%s\n' "${JOBS_LIST[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one {}'
echo "config search complete"
