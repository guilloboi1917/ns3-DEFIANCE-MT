#!/usr/bin/env bash
# Two-knob buffer/timer sweep (probe, not a thesis campaign).
#
# Cell A: TCP MinRto ablation at (SndBuf 1 MiB, RLC 180 kB) - does raising the
#         RTO floor above the uplink per-segment access latency close the
#         1-segment RTO trap? (MinRto 200 ms cell already exists in
#         results/rlc-buffer-sweep/180k.)
# Cell B: 2D SndBuf x RLC cap - is the drop burst set by the transport budget
#         (socket buffer) versus the UE link buffer (RLC TX cap)?
# Cell C: QUIC at 180 kB vs 512 kB - was the QUIC NR-scenario freeze the same
#         RLC-overflow trigger?
#
# Results: results/buffer-2d-sweep/<label>/<variant>/seed_<N>/
set -u

# Archived under obsolete/: resolve the example dir (parent) so NS3ROOT and
# results/ still point at the real locations.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../.." && pwd)"
OUTROOT="$HERE/results/buffer-2d-sweep"
JOBS="${JOBS:-6}"
mkdir -p "$OUTROOT"
export NS3ROOT OUTROOT

# run_one <label> <sndbuf> <rlc> <minrto> <variant> <seed> <dur> <rate> <proto>
run_one() {
    local label="$1" sndbuf="$2" rlc="$3" minrto="$4" variant="$5" seed="$6" dur="$7" rate="$8" proto="$9"
    local out="$OUTROOT/$label/$variant/seed_$seed"
    mkdir -p "$out"
    "$NS3ROOT/ns3" run "defiance-nr-rl-handover \
--simDuration=$dur --seed=$seed --runId=1 --topology=triangle \
--flowDirection=ul --uavMobility=random-waypoint --tcpVariant=$variant \
--startHeight=50 --endHeight=200 --addInterferingUes=0 --aerialUeRatio=0 \
--interfererMobility=static --transportProtocol=$proto --bandwidthMhz=30 \
--rlRewardRefMbps=15.0 --ueSpeed=20 --numerology=1 --trafficRateMbps=$rate \
--logging=true --parallel=0 --rlMode=false --handoverAlgorithm=a3 \
--rlcTxBufferBytes=$rlc --tcpSndBufBytes=$sndbuf --tcpMinRtoMs=$minrto \
--outputDir=$out" > "$out/stdout.log" 2>&1
    echo "done $label $variant seed_$seed (exit=$?)"
}
export -f run_one

JOBS_LIST=()
add() { JOBS_LIST+=("$*"); }

# ---- Cell A: MinRto ablation (SndBuf 1 MiB = 1048576, RLC 180 kB) ----
for mr in 500 1000; do
    for vs in "TcpNewReno:1" "TcpNewReno:2" "TcpNewReno:8" "TcpNewReno:13" \
              "TcpCubic:1" "TcpCubic:2" "TcpCubic:4" "TcpCubic:5" \
              "TcpVeno:1" "TcpVeno:2" "TcpVeno:3" "TcpVeno:8" \
              "TcpBbr:1" "TcpBbr:8"; do
        add "sb1024k-rlc180k-mr${mr}" 1048576 180000 "$mr" "${vs%%:*}" "${vs##*:}" 50 35 tcp
    done
done

# ---- Cell B: 2D SndBuf x RLC cap ----
for sb in 131072 524288; do
    for rlc in 180000 500000; do
        for vs in "TcpNewReno:1" "TcpNewReno:2" "TcpNewReno:8" \
                  "TcpCubic:1" "TcpCubic:2" "TcpCubic:4" \
                  "TcpBbr:1" "TcpBbr:8"; do
            add "sb$((sb/1024))k-rlc$((rlc/1000))k-mr200" "$sb" "$rlc" 200 "${vs%%:*}" "${vs##*:}" 50 35 tcp
        done
    done
done

# ---- Cell C: QUIC A/B (user probe: 512 kB @ 50 Mbps / 10 s) ----
add quic-rlc180k-50s 1048576 180000 200 TcpBbr 0 50 35 quic
add quic-rlc512k-50s 1048576 524288 200 TcpBbr 0 50 35 quic
add quic-rlc512k-10s 1048576 524288 200 TcpBbr 0 10 50 quic

printf '%s\n' "${JOBS_LIST[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one {}'
echo "2d sweep complete"
