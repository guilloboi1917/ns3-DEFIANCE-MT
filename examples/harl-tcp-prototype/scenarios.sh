#!/usr/bin/env sh
# =============================================================================
# HARL TCP — Standalone Scenario Runner (non-RL mode)
# =============================================================================
#
# Runs preconfigured scenarios for data collection and analysis.
# All runs produce CSV output in output/ for post-processing.
# Results independent of the RL pipeline.
#
# Usage:  cd contrib/defiance/examples/harl-tcp-prototype && bash scenarios.sh
#
# Environment: set NS3_HOME if not already set
# =============================================================================

NS3_HOME="${NS3_HOME:-$(cd ../../../.. && pwd)}"
cd "$NS3_HOME" || exit 1

NS3="$(pwd)/ns3"
SCENARIO="defiance-tcp-harl"
OUTPUT_BASE="contrib/defiance/examples/harl-tcp-prototype/output"

mkdir -p "$NS3_HOME/$OUTPUT_BASE"

run() {
    local name="$1"
    shift
    echo ""
    echo "========================================================"
    echo "  $name"
    echo "========================================================"
    echo "  $NS3 run \"$SCENARIO -- $*\""
    echo ""
    $NS3 run "$SCENARIO -- $*" 2>&1
    echo ""
    echo "  Done: $name"
}

# =========================================================================
# 1. Baseline — simple topology, default parameters
# =========================================================================

run "Simple: 2 eNBs, UAV helix, 50s, default" \
    --simDuration=50 --topology=simple --uavMobility=helix \
    --logging=true --handoverAlgorithm=a3

run "Simple: 2 eNBs, constant UAV, 50s" \
    --simDuration=50 --topology=simple --uavMobility=constant \
    --logging=true --handoverAlgorithm=a3

run "Simple: 2 eNBs, random waypoint, 50s" \
    --simDuration=50 --topology=simple --uavMobility=random-waypoint \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 2. Simple topology — interference and background traffic
# =========================================================================

run "Simple: 2 eNBs, 4 ground static UEs" \
    --simDuration=50 --topology=simple --addStaticUes=4 --aerialUeRatio=0 \
    --logging=true --handoverAlgorithm=a3

run "Simple: 2 eNBs, 4 aerial static UEs" \
    --simDuration=50 --topology=simple --addStaticUes=4 --aerialUeRatio=1 \
    --logging=true --handoverAlgorithm=a3

run "Simple: 2 eNBs, 8 mixed static UEs (50% aerial)" \
    --simDuration=50 --topology=simple --addStaticUes=8 --aerialUeRatio=0.5 \
    --logging=true --handoverAlgorithm=a3

run "Simple: 2 eNBs, 4 aerial UEs + full-buffer interference" \
    --simDuration=50 --topology=simple --addStaticUes=4 --aerialUeRatio=1 \
    --fullBufferInterference=true \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 3. Simple topology — TCP variants
# =========================================================================

run "Simple: TCP NewReno, 50s" \
    --simDuration=50 --topology=simple --tcpVariant=TcpNewReno \
    --logging=true --handoverAlgorithm=a3

run "Simple: TCP Cubic, 50s" \
    --simDuration=50 --topology=simple --tcpVariant=TcpCubic \
    --logging=true --handoverAlgorithm=a3

run "Simple: TCP BBR, 50s (for comparison)" \
    --simDuration=50 --topology=simple --tcpVariant=TcpBbr \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 4. Simple — handover algorithm comparison
# =========================================================================

run "Simple: A3 handover, 50s" \
    --simDuration=50 --topology=simple --handoverAlgorithm=a3 \
    --logging=true

run "Simple: NoOp handover (no handovers), 50s" \
    --simDuration=50 --topology=simple --handoverAlgorithm=noop \
    --logging=true

# =========================================================================
# 5. Hexgrid topology — eNB grid
# =========================================================================

run "Hexgrid: 21 cells, UAV helix, 50s" \
    --simDuration=50 --topology=hexgrid --uavMobility=helix \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 21 cells, random waypoint, 50s" \
    --simDuration=50 --topology=hexgrid --uavMobility=random-waypoint \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 6. Hexgrid — interference and background traffic
# =========================================================================

run "Hexgrid: 4 ground static UEs" \
    --simDuration=50 --topology=hexgrid --addStaticUes=4 --aerialUeRatio=0 \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 4 aerial static UEs" \
    --simDuration=50 --topology=hexgrid --addStaticUes=4 --aerialUeRatio=1 \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 8 mixed static UEs (50% aerial)" \
    --simDuration=50 --topology=hexgrid --addStaticUes=8 --aerialUeRatio=0.5 \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 8 ground static UEs" \
    --simDuration=50 --topology=hexgrid --addStaticUes=8 --aerialUeRatio=0 \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 8 aerial static UEs (dense interference)" \
    --simDuration=50 --topology=hexgrid --addStaticUes=8 --aerialUeRatio=1 \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 7. Hexgrid — high density (max cells, max interference)
# =========================================================================

run "Hexgrid: 8 mixed UEs, full-buffer interference" \
    --simDuration=50 --topology=hexgrid --addStaticUes=8 --aerialUeRatio=0.5 \
    --fullBufferInterference=true \
    --logging=true --handoverAlgorithm=a3

run "Hexgrid: 12 mixed UEs, 50s" \
    --simDuration=50 --topology=hexgrid --addStaticUes=12 --aerialUeRatio=0.5 \
    --logging=true --handoverAlgorithm=a3

# =========================================================================
# 8. Parametric sweeps — UAV speed
# =========================================================================

for speed in 5 10 20 30 50; do
    run "Speed sweep: ${speed}m/s, hexgrid, 30s" \
        --simDuration=30 --topology=hexgrid --ueSpeed=${speed} \
        --logging=true --handoverAlgorithm=a3
done

# =========================================================================
# 9. Parametric sweeps — UAV altitude
# =========================================================================

for start_h in 50 100 150 200 300; do
    end_h=$((start_h + 150))
    run "Altitude sweep: ${start_h}-${end_h}m, hexgrid, 30s" \
        --simDuration=30 --topology=hexgrid --uavMobility=helix \
        --startHeight=${start_h} --endHeight=${end_h} \
        --logging=true --handoverAlgorithm=a3
done

# =========================================================================
# 10. Episodic sweeps — simulation duration
# =========================================================================

for dur in 10 30 60 120; do
    run "Duration sweep: ${dur}s, hexgrid, 4 aerial UEs" \
        --simDuration=${dur} --topology=hexgrid --addStaticUes=4 --aerialUeRatio=1 \
        --logging=true --handoverAlgorithm=a3
done

# =========================================================================
# Summary
# =========================================================================
echo ""
echo "========================================================"
echo "  All scenarios complete."
echo "  Output files in: $NS3_HOME/$OUTPUT_BASE"
echo "========================================================"
echo ""
echo "CSV files generated:"
ls -1 "$NS3_HOME/$OUTPUT_BASE"/*.csv 2>/dev/null | wc -l
echo ""
echo "To check a specific run:"
echo "  head -5 $NS3_HOME/$OUTPUT_BASE/harl-tcp-rtt.csv"
