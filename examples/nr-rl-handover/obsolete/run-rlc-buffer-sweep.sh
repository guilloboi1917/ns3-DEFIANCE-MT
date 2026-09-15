#!/usr/bin/env bash
# RLC TX-buffer sensitivity sweep (probe, not a thesis campaign).
#
# Motivated by BUGS-ISSUES "RLC TX buffer cap is the limiting factor for
# classic congestion controls": the tcp-variants campaign ran with the
# scenario default --rlcTxBufferBytes=180000, at which the UE RLC TX buffer
# overflows (NrRlc::TxDrop) and silently discards PDCP PDUs, collapsing the
# classic CCs into a 1-segment RTO limit cycle.
#
# This sweeps the cap at fixed everything else and logs the drop trace.
# Results land in results/rlc-buffer-sweep/<arm>/<variant>/seed_<N>/.
set -u

# Archived under obsolete/: resolve the example dir (parent) so NS3ROOT and
# results/ still point at the real locations.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../.." && pwd)"
cd "$NS3ROOT"
export HERE NS3ROOT

ARMS="${ARMS:-180000 500000 2000000}"
JOBS="${JOBS:-6}"

run_one() {
    local bytes="$1" variant="$2" seed="$3"
    local arm
    case "$bytes" in
        180000) arm="180k" ;;
        500000) arm="500k" ;;
        2000000) arm="2m" ;;
        *) arm="$bytes" ;;
    esac
    local out="$HERE/results/rlc-buffer-sweep/$arm/$variant/seed_$seed"
    mkdir -p "$out"
    "$NS3ROOT/ns3" run "defiance-nr-rl-handover \
--simDuration=50 --seed=$seed --runId=1 --topology=triangle \
--flowDirection=ul --uavMobility=random-waypoint --tcpVariant=$variant \
--startHeight=50 --endHeight=200 --addInterferingUes=0 --aerialUeRatio=0 \
--interfererMobility=static --transportProtocol=tcp --bandwidthMhz=30 \
--rlRewardRefMbps=15.0 --ueSpeed=20 --numerology=1 --trafficRateMbps=35 \
--logging=true --parallel=0 --rlMode=false --handoverAlgorithm=a3 \
--rlcTxBufferBytes=$bytes --outputDir=$out" \
        > "$out/stdout.log" 2>&1
    echo "done $arm $variant seed_$seed (exit=$?)"
}
export -f run_one

# variant:seed list (seeds chosen from the stuck set of the 180 kB baseline)
JOBS_LIST=()
for v in TcpNewReno:1 TcpNewReno:2 TcpNewReno:8 TcpNewReno:13 \
         TcpCubic:1 TcpCubic:2 TcpCubic:4 TcpCubic:5 \
         TcpVeno:1 TcpVeno:2 TcpVeno:3 TcpVeno:8 \
         TcpBbr:1 TcpBbr:8; do
    for a in $ARMS; do
        JOBS_LIST+=("$a ${v%%:*} ${v##*:}")
    done
done

printf '%s\n' "${JOBS_LIST[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one {}'
echo "sweep complete"
