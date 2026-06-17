#!/usr/bin/env sh

# =============================================================================
# HARL TCP — RL Training Script
# =============================================================================
#
# Usage:  cd contrib/defiance/examples/harl-tcp-prototype && ./train.sh
#
# Runs multiple training configurations for comparison.
# Results are saved to ~/ray_results/defiance-tcp-harl/ by default.
#
# Scenario: UAV with TCP BBR upload, LTE cellular network.
# The RL agent learns handover decisions (which eNB to connect to).
#
# Key CLI arguments (passed via -c key=value):
#   parallel=N          Number of parallel envs (default 4)
#   simDuration=S       Episode duration in seconds (default 40)
#   topology=STRING     simple (2 eNBs) or hexgrid (21 cells)
#   addStaticUes=N      Background UEs for interference (default 0)
#   aerialUeRatio=F     Fraction of UEs at aerial height (0 or 1)
#   ueSpeed=F           UAV speed in m/s
#   handoverPenalty=F   Cost per handover in normalized [0,1] units
#   handoverMargin=F    RSRP margin (3GPP range, ~1dB/step). Target must
#                       have RSRP > serving + margin. -999 disables.
#   rlRttPenaltyWeight=F  Weight of RTT inflation penalty
#   rlMinRttMs=F        Baseline RTT for inflation calc
#   rlReferenceRate=F   Reference UL rate for throughput normalization
#   delay=MS            Transmission delay for agent observations
# =============================================================================

# ---- Simple topology: baseline A3 ----------------------------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     handoverAlgorithm=a3 -i 300

# ---- Simple topology: RL agent (default params) ---------------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent handoverMargin=3 -i 300

# ---- Simple topology: RL, no margin (full exploration) --------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent handoverMargin=-999 -i 300

# ---- Simple topology: RL, tighter margin (conservative) -------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent handoverMargin=6 -i 300

# ---- Simple topology: RL, higher handover cost ----------------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent handoverPenalty=0.5 handoverMargin=3 -i 300

# ---- Simple topology: RL, lower handover cost ------------------------------ #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent handoverPenalty=0.05 handoverMargin=3 -i 300

# ---- Simple topology: RL, heavier RTT penalty ------------------------------ #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=simple \
     rlMode=true handoverAlgorithm=agent rlRttPenaltyWeight=0.5 handoverMargin=3 -i 300

# ================================================================ #
# Hexgrid topology (21 cells, harder problem)                     #
# ================================================================ #

# ---- Hexgrid: baseline A3 ------------------------------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=hexgrid \
     handoverAlgorithm=a3 -i 300

# ---- Hexgrid: RL agent ---------------------------------------------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=hexgrid \
     rlMode=true handoverAlgorithm=agent handoverMargin=3 -i 300

# ---- Hexgrid: RL + 4 static aerial UEs (dense interference) ---------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=50 topology=hexgrid \
     addStaticUes=4 aerialUeRatio=1 \
     rlMode=true handoverAlgorithm=agent handoverMargin=3 -i 300

# ---- Hexgrid: RL + 4 static ground UEs (less interference) ---------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=50 topology=hexgrid \
     addStaticUes=4 aerialUeRatio=0 \
     rlMode=true handoverAlgorithm=agent handoverMargin=3 -i 300

# ---- Hexgrid: RL + transmission delay (realistic feedback lag) ------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=40 topology=hexgrid \
     rlMode=true handoverAlgorithm=agent delay=500 handoverMargin=3 -i 300

# ---- Hexgrid: RL + longer episodes (more data per episode) ---------------- #
run-agent train -n defiance-tcp-harl \
  -c parallel=8 simDuration=80 topology=hexgrid \
     addStaticUes=2 aerialUeRatio=0.5 \
     rlMode=true handoverAlgorithm=agent handoverMargin=3 -i 300

# ================================================================ #
# Inference examples (after training)                               #
# ================================================================ #

# If you have a trained checkpoint, run inference:
#   run-agent infer -n defiance-tcp-harl -a /path/to/PPO/checkpoint \
#     -c simDuration=40 topology=simple
#
# In a devcontainer the path is usually /root/ray_results/PPO_run/...
