#!/usr/bin/env sh

# Usual handover algorithms for comparison
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=a2a4 -i 300
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=a3 -i 300

# RL handover algorithm (TCP BBR, discrete action, SINR + cwnd observations)
## No transmission delay
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=agent handoverPenalty=1000 -i 300

## Transmission delay 500 ms
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=agent delay=500 handoverPenalty=1000 -i 300

## Transmission delay 5000 ms
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=agent delay=5000 handoverPenalty=1000 -i 300

## Varying handover penalty
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=agent handoverPenalty=500 -i 300
run-agent train -n defiance-uav-handover -c parallel=124 stepTime=1000 simTime=150 handoverAlgorithm=agent handoverPenalty=2000 -i 300
