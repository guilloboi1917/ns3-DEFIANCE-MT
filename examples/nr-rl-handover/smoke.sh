#!/usr/bin/env sh
# Clean stale shared memory from previous crashes (prevents deadlocks)
rm -f /dev/shm/ns3-ai_*

# cd /home/nisaak/masterthesis-ns3/ns-3-dev/contrib/defiance && eval $(poetry env activate)
run-agent train -n defiance-nr-rl-handover -c parallel=1 simDuration=5 topology=triangle rlMode=true handoverAlgorithm=agent transportProtocol=udp flowDirection=dl addInterferingUes=0 stepTime=200 -t PPO -st 600 -i 1
