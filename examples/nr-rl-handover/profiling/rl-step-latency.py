#!/usr/bin/env python3
"""Per-step ns3-ai round-trip latency of the RL-mode sim.

Usage: python3 profiling/rl-step-latency.py [--sim-duration 20] [--seed 10] ...
"""

import argparse
import time

import numpy as np
from ns3ai_gym_env.envs.ns3_multi_agent_environment import Ns3MultiAgentEnv
from defiance import NS3_HOME


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim-duration", type=float, default=20.0)
    ap.add_argument("--output-dir", default="output/perf-rl/latency")
    ap.add_argument("--seed", type=int, default=10)
    ap.add_argument("--trial-name", default="perf-rl-latency")
    ap.add_argument("--n-steps", type=int, default=0,
                    help="0 = run until the episode ends (simDuration)")
    args = ap.parse_args()

    settings = {
        "simDuration": str(args.sim_duration),
        "flowDirection": "dl",
        "transportProtocol": "udp",
        "addInterferingUes": "4",
        "aerialUeRatio": "1.0",
        "topology": "triangle",
        "bandwidthMhz": "10",
        "errorModel": "eesm-ir-t1",
        "channelUpdateMs": "50",
        "trafficRateMbps": "100",
        "seed": str(args.seed),
        "runId": "1",
        "rlMode": "true",
        "handoverAlgorithm": "agent",
        "parallel": "0",
        "logging": "false",
        "outputDir": args.output_dir,
        "trial_name": args.trial_name,
    }

    env = Ns3MultiAgentEnv(
        targetName="defiance-nr-rl-handover",
        ns3Path=NS3_HOME,
        ns3Settings=settings,
        trial_name=args.trial_name,
    )
    t0 = time.perf_counter()
    obs, info = env.reset()
    reset_s = time.perf_counter() - t0
    agent_id = next(iter(obs))

    lats: list[float] = []
    step = 0
    done = False
    while not done and (args.n_steps == 0 or step < args.n_steps):
        t = time.perf_counter()
        obs, rew, term, trunc, info = env.step({agent_id: 0})
        lats.append(time.perf_counter() - t)
        done = bool(term[agent_id] or trunc[agent_id])
        step += 1
    env.close()

    l = np.array(lats) * 1e3  # ms
    print(f"reset: {reset_s:.3f} s")
    print(f"steps: {len(l)}")
    print("step latency (ms): "
          f"min {l.min():.2f} | mean {l.mean():.2f} | p50 {np.percentile(l, 50):.2f} "
          f"| p95 {np.percentile(l, 95):.2f} | max {l.max():.2f}")
    sync_s = l.sum() / 1e3
    print(f"total sync overhead (sim blocked on python): {sync_s:.2f} s "
          f"({sync_s / (args.sim_duration or 1) * 100:.1f}% of sim time)")
    print(f"total env wall: {reset_s + sync_s:.2f} s")


if __name__ == "__main__":
    main()
