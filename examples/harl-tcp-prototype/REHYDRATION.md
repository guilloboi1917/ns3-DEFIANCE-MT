# HARL TCP Prototype — Rehydration Protocol

> Generated 2026-06-17. For a future agent to resume work without conversation history.
> **Update this file on every notable code change** to keep rehydration viable.

---

## 1. Project Structure

```
contrib/defiance/examples/harl-tcp-prototype/
├── .gitignore
├── RL-doc/README.md                          # Full analysis & design doc
├── scenarios.sh                              # 26 preconfigured standalone (non-RL) commands
├── train.sh                                  # RL training configurations
├── tpc-bug.md                                # TPC-0 power control bug doc
├── flowmon-parse-results.py                  # FlowMonitor analysis
├── plot-antenna-pattern.py                   # Antenna gain visualization
├── plot-harl-tcp-stats.py                    # Simulation stats plotting
├── plot-three-gpp-antenna.py                 # 3GPP pattern reference
├── uav_path.py                               # UAV mobility path plotter
├── harl-tcp-scenario.cc                      # Main entry point
├── harl-tcp-scenario-setup.cc                # Full scenario setup (#included)
├── handover/                                 # RL handover agent for TCP
│   ├── harl-tcp-handover-agent-app.cc/.h     # RL agent (obs space, action dispatch)
│   ├── harl-tcp-handover-obs-app.cc/.h       # Observation collection (LTE + TCP metrics)
│   ├── harl-tcp-handover-act-app.cc/.h       # Action execution (handover via g_lteHelper)
│   └── harl-tcp-handover-rwd-app.cc/.h       # Reward computation (UL goodput, RTT penalty)
└── tcp-cca/                                  # TCP CCA agent (scaffolding stubs only)
    ├── harl-tcp-cca-agent-app.cc/.h          # Stub - not implemented
    ├── harl-tcp-cca-obs-app.cc/.h            # Stub - not implemented
    ├── harl-tcp-cca-act-app.cc/.h            # Stub - not implemented
    └── harl-tcp-cca-rwd-app.cc/.h            # Stub - not implemented
```

Build target: `defiance-tcp-harl` (defined in `contrib/defiance/examples/CMakeLists.txt`).

---

## 2. Architecture Overview

### Dual-mode design
| Mode | `rlMode` | Handover algorithm | Logging | TCP deadline |
|------|:--------:|:------------------:|:-------:|:------------:|
| Standalone | false | A3 (configurable) | CSV + FlowMonitor | None |
| RL training | true | NoOp + RL agent | Disabled | `g_tcpConnected` guard on actions |

### RL apps (all on UAV node, StartTime=1.0s)

```
ObsApp ──observations──▶ AgentApp ──actions──▶ ActApp
                              ▲                     
RewardApp ────rewards─────────┤
```

- **ObsApp**: Collects LTE PHY measurements (periodic, 200ms trace), TCP metrics (RTT), UAV position/velocity, PHY stats (MCS, Tx power). Observation is sent periodically at 480ms cadence (MS480). The PHY callback only stores values — no inference overhead. RNTI is lazily cached on first callback.
- **AgentApp**: Receives observations, calls `InferAction()` → Python/RLlib → receives action, dispatches to ActApp.
- **ActApp**: Validates preconditions, applies handover-in-progress guard + margin check, executes `g_lteHelper->HandoverRequest()`. (TCP guard was removed — it created a circular dependency: blocked handovers before TCP connected, but handovers were needed to reach a good cell for TCP to connect.)
- **RewardApp**: Tracks PacketSink Rx bytes (UL goodput), computes `reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm`

### Observation Space (Dict, 12 keys: 10 base + 3 delta groups, removed rrcState/cwnd/deliveryRate)
| Key | Type | Shape | Range | Source |
|-----|------|:-----:|:-----:|:-------|
| `rsrps` | float64 | (numBs,) | [-160, -40] dBm | UE PHY (all detectable cells) |
| `rsrqs` | float64 | (numBs,) | [-100, -3] dB | UE PHY (all detectable cells, -100 = sentinel) |
| `sinr` | float64 | (1,) | [-40, 50] dB | UL SRS SINR (current serving cell only) |
| `cellId` | Discrete | 1 | [0, numBs] | UE RRC (one-hot encoded by FlattenObservations) |
| `position` | float64 | (3,) | [-5000, 5000] | MobilityModel |
| `velocity` | float64 | (3,) | [-200, 200] | MobilityModel |
| `mcs` | int32 | (1,) | [0, 31] | UL PHY transmission |
| `txPower` | float64 | (1,) | [-50, 50] dBm | UE power control |
| `rtt` | int32 | (1,) | [0, 10000] ms | TCP socket |

**Changes**: `sinrs` → `sinr` (scalar). RSRP/RSRQ now from UE PHY `ReportUeMeasurements` trace (dBm/dB, all detectable cells), not eNB RRC measurement report (3GPP indices, A3-only cells). Observation bounds widened to accommodate real LTE stack values: [-160, -40] dBm for RSRP (was [-140, -40]), [-100, -3] dB for RSRQ (was [-20, -3]). Deltas widened accordingly: rsrpDelta [-60, 60] (was [-40, 40]), rsrqDelta [-60, 60] (was [-20, 20]). All observation values are clamped to their space bounds before sending to Python to prevent `ValueError` in RLlib's space validation (see §16).

**Flattened size = 117** (74 without delta fields if using LSTM): 21 rsrps + 21 rsrqs + 1 sinr + 21 rsrpDelta + 21 rsrqDelta + 1 sinrDelta + 22 one-hot cellId + 3 pos + 3 vel + 1 mcs + 1 txPower + 1 rtt.

**Delta fields**: `rsrpDelta`, `rsrqDelta`, `sinrDelta` capture per-cell RSRP/RSRQ and serving-cell SINR change since the last observation step. This gives the FC policy temporal trend information (rising/falling signal) without an LSTM. Delta is computed as `current - previous` and clamped to 0 when either value is a sentinel (unknown).

**LSTM mode**: When `use_lstm=True` is enabled via `config.rl_module(model_config={...})`, the LSTM hidden state captures temporal trends automatically, making the delta fields redundant. If removed, flattened size drops to **74** (117 − 43 delta dims).

**Sentinel values**: `rsrp` uses -140.0 (below typical LTE RSRP), `rsrq` uses -20.0 (below typical LTE RSRQ); `sinr` initializes to -40 dB. Sentinels are placed at the bottom of realistic ranges so an untrained NN with random weights produces valid softmax outputs (no NaN).

### Action Space
`Discrete(numBs + 1)`: 0 = no-op, 1..numBs = target cell ID.

### Reward
```
reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm

Throughput (normGoodput):
    goodput >= referenceRate               →  1.0
    minAcceptable <= goodput < reference    →  (goodput - R_min) / (R_ref - R_min)  [0..1]
    goodput < minAcceptable                 →  (goodput - R_min) / R_min             [negative]

Delay (rttPenalty):
    rtt <= delayMinRtt                      →  0.0
    delayMinRtt < rtt < maxAcceptableRtt    →  (rtt - D_min) / (D_max - D_min)       [0..1]
    rtt >= maxAcceptableRtt                 →  1.0

tcpPenalty  = tcpFailurePenalty                  (per step if !g_tcpAlive)
rlfTerm     = rlfPenalty                         (one-shot on RLF)
```
| Parameter | Default | Notes |
|-----------|---------|-------|
| `referenceRate` | 5 Mbps | Reference UL goodput; goodput >= this → reward=1 |
| `minimumAcceptableGoodput` | 2.5 Mbps | Below this → negative reward; ramp midpoint |
| `delayMinRttMs` | 55 ms | 10ms PGW + ~45ms LTE processing; zero penalty below this |
| `maxAcceptableRttMs` | 100 ms | Full delay penalty at this RTT |
| `tcpFailurePenalty` | 0.5 | Per-step when TCP dead |
| `rlfPenalty` | 1.0 | One-shot on RLF detection |
| `handoverPenalty` | 0.01 | Unused (tracked for logging only) |
| `pingPongInterval` | 1.0s | Window for ping-pong detection |

Typical reward per step: **[-2, 1]** range.

---

## 3. Key Design Decisions

### 3a. Periodic observation + trace source switch
Observation source switched from eNB RRC `RecvMeasurementReport` (3GPP index, A3-only cells, int32) to UE PHY `ReportUeMeasurements` (dBm/dB, all detectable cells, float64). PHY trace fires every 200ms; observation and inference are decoupled — the PHY callback only stores values, and a separate 480ms periodic timer triggers `Send(BuildObservation())` → agent `OnRecvObs` → `InferAction()`. This avoids 25× more sync Python round-trips (key was the old event-driven design firing `InferAction()` only ~6×/sim on A3 events, vs 150×/sim at 200ms periodic).

**RSRP/RSRQ sentinels**: -140.0 (RSRP, below typical -44 dBm LTE maximum) and -20.0 (RSRQ). Placed at realistic range floors so untrained NN with random weights produces valid softmax (prevents NaN). Box bounds: [-160, -40] RSRP, [-100, -3] RSRQ. Observation values are clamped to bounds in `BuildObservation()` before transmission.

### 3b. Observation cadence (MS480)
All timing aligned to **MS480 (480ms)**:
- Observation send: periodic timer at 480ms (decoupled from PHY callback)
- Reward calculation: 480ms (`m_calculationInterval`)
- RRC measurement report: MS480
- `stepTime` default: 480ms

The PHY `ReportUeMeasurements` trace fires every 200ms (default `UeMeasurementsFilterPeriod`) and only stores values (RSRP/RSRQ arrays). No `InferAction()` is triggered by PHY callbacks. `SendObservation()` is a separate periodic timer.

**RNTI caching**: The UAV's RNTI is lazily cached on the first PHY callback where the UE has connected (non-zero RNTI). Subsequent callbacks use a fast integer comparison — no `GetObject<>()` overhead at 200ms trace rate.

### 3b. TCP connection guard (REMOVED)
The TCP guard was removed because it created a **circular dependency**:
- The guard blocked handovers until TCP connected.
- But with NoOp handover (RL mode), the UE stayed on its initial cell, which often had poor signal.
- Poor signal caused TCP SYN loss → RTO backoff (3s → 6s → 12s) → TCP took 7-19s to connect.
- The agent couldn't handover to a better cell because TCP wasn't connected yet.

The handover-in-progress guard + margin check + UeManager state check already prevent handover storms during the brief handshake window, making the TCP guard redundant.

### 3c. Handover margin gating
**Location**: `harl-tcp-handover-act-app.cc`, margin check before `HandoverRequest`:

```cpp
if (m_handoverMargin > -999.0 && ...)
{
    double servingRsrp = g_lastRsrpValues[currentCellId];
    double targetRsrp = g_lastRsrpValues[newCellId];
    if (servingRsrp >= 0 && targetRsrp >= 0 &&
        targetRsrp < servingRsrp + m_handoverMargin)
        return; // blocked
}
```

- RSRP in 3GPP range (0-97, ~1 dB/step). Margin=3 means target must be 3 steps (~3 dB) better than serving.
- Set `--handoverMargin=-999` to disable gating.
- Per-cell RSRP stored in `g_lastRsrpValues[cellId]` (1-indexed), updated by ObsApp from measurement reports.
- Default margin = 3.0 (from CLI parameter).

### 3d. NoOp handover algorithm in RL mode
`lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm")` when `rlMode=true` or `handoverAlgorithm=agent`. RLF detection disabled:
```cpp
Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));
```

### 3e. Power control
Absolute mode, open-loop fractional compensation:
```cpp
Config::SetDefault("ns3::LteUePowerControl::AccumulationEnabled", BooleanValue(false));
Config::SetDefault("ns3::LteUePowerControl::ClosedLoop", BooleanValue(true));
Config::SetDefault("ns3::LteUePowerControl::Alpha", DoubleValue(1.0));
Config::SetDefault("ns3::LteUePowerControl::PoNominalPusch", IntegerValue(-80));
Config::SetDefault("ns3::LteUePowerControl::PoUePusch", IntegerValue(0));
```
See `tpc-bug.md` for details on why accumulation mode breaks.

### 3f. Seed advancement across restarts
**Location**: `ns3_environment.py`, `Ns3Env._get_init_run_id()` class method.

Class-level counter per config key ensures each process restart gets a unique seed block (+10000). Combined with `reset()` incrementing by 1, seeds never collide.

**Seed placement fix (2026-06-22)**: `RngSeedManager::SetSeed()` was moved from line 785 to line 2 of `scenarioSetup()` function body, **before** any random variables are created or sampled. Previously, the UAV mobility random waypoints were generated with the default RNG seed (identical across runs) because `SetSeed()` was called 200 lines too late. Now all RNG operations are properly seeded from `--seed`. Also, both `ascend-random` and `random-waypoint` mobility models now sample the UAV start position from the bounding box via `rbx->GetValue()`/`rby->GetValue()` instead of using the fixed center point `(bbox.min+max)/2`.

### 3g. Shared memory buffer
`MSG_BUFFER_SIZE=2048`, `shmSize/m_size=8192` (changed from 1024/4096). Init message (~1050 bytes for 21 cells × 11 obs keys + action) fits in 2048. Env state messages (~500-600 bytes).

### 3h. Handover-in-progress guard (event-driven)
**Location**: `harl-tcp-handover-act-app.cc`, precondition 1 in `ExecuteAction()`:

```cpp
if (g_handoverInProgress)
{
    NS_LOG_DEBUG("Handover already in progress, deferring.");
    return;
}
```

Set right before `g_lteHelper->HandoverRequest()`, cleared by `HandoverEndOk` and `HandoverEndError` trace callbacks. Prevents the crash where two RL actions arrive at the same sim tick for the same UE — the second dispatch finds the UE already in `HANDOVER_PREPARATION` and triggers `NS_FATAL`.

### 3i. TCP failure penalty (event-driven)
When the TCP connection fails or dies mid-simulation, the reward app applies a `tcpFailurePenalty` (default 5.0) each step.

Detection is entirely event-driven:
- `g_tcpAlive = true` in `NotifyConnectionSucceeded`
- `g_tcpAlive = false` in `NotifyConnectionFailed` (initial failure)
- `g_tcpAlive = false` in `NotifyTcpStateChange` when TCP socket transitions to `CLOSED` or `LAST_ACK` (mid-flight disconnection)

The TCP socket `State` trace is connected at sim time 1.5s (same schedule slot as the existing RTT trace).

### 3j. External patches
**File: `src/antenna/model/three-gpp-antenna-model-oriented.cc`**
Clamped `thetaEff` via `std::fabs(thetaEff)` to prevent invalid negative inclination with aerial UEs above downtilt angle. The 3GPP vertical pattern `(thetaDeg-90)^2` is symmetric, so reflection is valid.

### 3k. Shared memory semaphore reset on episode transitions

**Verdict: The semaphore cycle is self-balancing. No reset bug.**

Extensive investigation (py-spy profiling, strace, per-step timing logs, standalone ns-3 tests) confirmed:
- Each `InferAction()` → `NotifyCurrentState()` round-trip leaves all four semaphore counters at their initial state `{1,0,1,0}`.
- `NotifySimulationEnd()` follows the same protocol and correctly sets `isGameOver=true`.
- The original `reset()` code (reusing the same msgInterface) is correct.

**What caused the apparent deadlocks**:
1. **Stale `/dev/shm/ns3-ai_*` files** from crashed training runs (SIGABRT). These left semaphore counters in arbitrary states. The next training run opened them and deadlocked.
2. **`exit(1)` in `Experiment.run()`** — when ns-3 died within 0.5s (now 2.0s), `exit(1)` killed the entire Ray worker, triggering restart loops.
3. **`SIMULATION_EARLY_ENDING=0.5s`** was too aggressive for hexgrid topology initialization.

**Fixes applied**:
- `exit(1)` → `raise RuntimeError` in `contrib/ai/python_utils/ns3ai_utils.py:Experiment.run()`. RLlib's `_try_env_reset` catches this and retries via `make_env()`.
- `SIMULATION_EARLY_ENDING`: 0.5s → **2.0s** — gives ns-3 enough time to parse args, initialize LTE stack, and open shared memory.
- Always `rm -f /dev/shm/ns3-ai_*` before starting training.

**Abandoned approaches**:
- Experiment recreation in `reset()` — caused `__del__` race conditions and "No such file" crashes on ns-3 subprocess startup.
- Direct `/dev/shm` semaphore reset via ctypes — semaphore struct is not at a known offset (Boost segment manager header).
- Making ns-3 the shared memory creator — would break the init probe flow.

### 3l. Trace source switch: eNB RRC → UE PHY

Observation source switched from eNB RRC `RecvMeasurementReport` (3GPP 0-97 indices, A3-event-triggered cells only, `int32_t`) to UE PHY `ReportUeMeasurements` (dBm/dB, all detectable cells, `float64`).

**Key changes**:
- Sentinel values: -1 (int32) → **-140 RSRP / -20 RSRQ** (bottom of realistic LTE range). Prevents NaN from untrained NN random weights.
- Box bounds: `[-140, -40]` RSRP dBm, `[-20, -3]` RSRQ dB.
- Delta threshold: `> -135` RSRP / `> -15` RSRQ (vs old `>= 0`).
- Handover margin check: `servingRsrp < -135.0` (vs old `servingRsrp < 0` — was always true with dBm).
- PHY trace fires every **200ms** (default `UeMeasurementsFilterPeriod`); stores values only (no inference overhead).
- Observation send decoupled: periodic timer at **480ms** (MS480), matching reward interval.
- **RNTI caching**: lazy-initialized on first PHY callback — avoids expensive `GetObject<LteUeNetDevice>()` lookup on every 200ms callback.

**Trace connection path**:
```
/NodeList/<uavNodeId>/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/ReportUeMeasurements
```
Note: must be `ComponentCarrierMapUe` (not `ComponentCarrierMap`) and use `Config::ConnectWithoutContext` (the trace has a context-string first param).

### 3m. Timing and performance

Per-step wall time (from `[TIMING]` instrumentation):
| Metric | Value |
|--------|-------|
| `send_actions` | <1ms |
| `rx_env_state` (spin-wait) | **0.49–0.81s** |
| `get_state` (protobuf) | <1ms |
| Total per step | **0.5–0.8s** |
| Steps per 10s episode (480ms) | ~21 |
| Episode wall time | ~15–20s |
| Reset (Experiment.run + init) | ~2.05s |

**Bottleneck**: `rx_env_state()` blocks Python while ns-3 simulates the next 480ms step. The 0.5–0.8s is **ns-3 simulation wall time**, not semaphore overhead — the atomic spin-lock itself is nanoseconds. Python is simply parked waiting for ns-3 to finish simulating LTE (PHY, TCP, mobility). The sync round-trip (C++ sends obs → Python NN forward → Python sends action → C++ continues) dominates.

**CPU pattern** (parallel=2):
- Rollout: 2 ns-3 processes + 2 Ray workers at low-moderate CPU
- Training: ns-3 processes exit, learner runs 5 epochs on CPU (3 cores at 100% due to PyTorch threading), workers idle
- Between iterations: workers idle, no ns-3 processes — **normal**

### 3n. Current working state (2026-06-19)

✅ Agent training runs end-to-end with hexgrid topology
✅ PHY trace provides actual dBm RSRP (-68 to -44) and dB RSRQ (-5 to -20)
✅ Agent makes handover decisions based on real cell measurements
✅ Throughput jumps from ~4 Mbit (no handovers) to ~50-80 Mbit (with handovers)
✅ Episodes complete normally (`truncated=True` via `NotifySimulationEnd`)
✅ Resets work: `[RESET] run() returned in 2.051s`
✅ Per-step timing: 0.49-0.81s rx wait
✅ `exit(1)` → `RuntimeError` prevents Ray worker kills on ns-3 early death

⚠️ Training iterations complete but collection is slow (~600s per iteration with parallel=2). This is expected for ns-3 RL.
⚠️ With parallel≥4, intermittent worker stalls occur (ns-3 startup race).

### 3o. ASYNC vectorization (throughput optimization)

RLlib v2's `MultiAgentEnvRunner` supports `gymnasium.vector.AsyncVectorEnv` mode
via the config:

```python
.env_runners(
    num_envs_per_env_runner=N,          # default: 1
    gym_env_vectorize_mode=ASYNC,       # default: SYNC
    ...
)
```

In ASYNC mode, each sub-env runs in its own `multiprocessing` subprocess.
This is critical because the singleton guards (`Ns3Env._created`,
`Experiment._created`) are class-level — each subprocess has its own copy,
so **no collision** occurs. The env factory receives an `EnvContext` with
`vector_index=0,1,...`, producing unique trial names like `training3_0` and
`training3_1`, which means separate shared memory segments per sub-env.

**Why it could help**: The per-step bottleneck (0.5–0.8s) is actually
**ns-3 simulation time**, not semaphore overhead. Python is blocked waiting
for ns-3 to simulate 480ms of LTE events. With ASYNC mode, these waits
overlap:

```
Wall time:  0s          0.2s        0.4s        0.6s        0.8s
           ──┬───────────┬───────────┬───────────┬───────────┬──
Env A:     [step]·········ns-3 sim·············[done]
Env B:                  [step]·········ns-3 sim·············[done]
```

Two sub-envs per runner → ~2× raw sample throughput in the same wall time.

**Trade-offs** (assuming `num_envs_per_env_runner=2`):

| Aspect | SYNC (current) | ASYNC |
|--------|:--------------:|:-----:|
| Total ns-3 processes | `parallel` | `parallel × 2` |
| Memory (1–2GB each) | 4–8 GB (parallel=4) | 8–16 GB |
| Ray actors | `parallel` | `parallel` (same) |
| Throughput per runner | baseline | ~2× (overlapped waits) |
| Startup race | N processes | **2N processes** (worse) |

**Comparison: ASYNC vectorization vs More Workers**

Both approaches give the same total ns-3 subprocess count. The difference
is Ray actor overhead vs per-actor memory pressure:

| Approach | ns-3 procs | Memory | Actors | Throughput |
|----------|:----------:|:------:|:-----:|:----------:|
| `parallel=8, num_envs=1` (SYNC) | 8 | 8–16 GB | 8 | 8× ns-3 sims overlapped |
| `parallel=4, num_envs=2` (ASYNC) | 8 | 8–16 GB | **4** | 8× ns-3 sims overlapped |

**Verdict**: ASYNC vectorization makes sense _after_ the agent starts
learning (see §12 for hyperparameter fixes). Until then, doubling bad
samples doubles wasted wall time. When ready, start with
`num_envs_per_env_runner=2` and monitor for startup race increases.

---

## 4. CLI Parameters

| Flag | Default | Description |
|------|---------|-------------|
| `simDuration` | 50 | Simulation duration (s) |
| `topology` | hexgrid | simple (2 eNBs) or hexgrid (21 cells) |
| `rlMode` | false | Enable RL training mode |
| `handoverAlgorithm` | a3 | a3, noop, or agent |
| `handoverMargin` | 3.0 | RSRP margin, -999 = disable |
| `tcpFailurePenalty` | 1.0 | Penalty per step when TCP connection is dead |
| `rlfPenalty` | 2.0 | One-shot penalty when RLF is detected |
| `pingPongPenalty` | 0.05 | Extra penalty per handover within PingPongInterval |
| `pingPongInterval` | 1.0 | Seconds; handovers closer incur ping-pong penalty |
| `stepTime` | 480 | Step interval (ms) — aligned with MS480 measurement cadence |
| `seed` | 0 | RNG seed |
| `runId` | 0 | Episode counter (advanced by Python) |
| `parallel` | 0 | Worker offset for multi-worker |
| `addStaticUes` | 0 | Background traffic UEs |
| `aerialUeRatio` | 0.0 | Fraction of static UEs at aerial height |
| `fullBufferInterference` | false | Full-buffer DL interference |
| `tcpVariant` | TcpBbr | TCP congestion control algorithm |
| `handoverPenalty` | 0.01 | Reward penalty per handover |
| `rlReferenceRate` | 5000000 | UL throughput normalization (bps, 5 Mbps) |
| `rlRttPenaltyWeight` | 0.01 | RTT inflation penalty weight |
| `rlMinRttMs` | 55.0 | Baseline RTT (ms) |
| `logging` | true | CSV output (auto-disabled in rlMode) |
| `uavMobility` | ascend-random | constant, ascend-random, or random-waypoint |
| `ueSpeed` | 20.0 | UAV speed (m/s) |
| `startHeight` | 80.0 | UAV start altitude (m) |
| `endHeight` | 250.0 | UAV end altitude (m) |
| `delay` | 0 | Inter-app communication delay (ms) |

---

## 5. Build & Run

### Build
```bash
cd ~/masterthesis-ns3/ns-3-dev
source venv/bin/activate
./ns3 configure --enable-examples --enable-tests --build-profile=optimized
./ns3 build defiance-tcp-harl
```

### Standalone
```bash
./ns3 run "defiance-tcp-harl --simDuration=30 --topology=hexgrid --addStaticUes=4 \
  --aerialUeRatio=0.5 --logging=true --handoverAlgorithm=a3 --seed=1"
```

### RL training
```bash
# Always clean stale shared memory first (prevents deadlocks from previous crashes)
rm -f /dev/shm/ns3-ai_*

# Single worker
run-agent train -n defiance-tcp-harl -c rlMode=true handoverAlgorithm=agent \
  handoverMargin=3 parallel=1 stepTime=480 simDuration=10 -i 1

# Multi-worker
run-agent train -n defiance-tcp-harl -c parallel=8 stepTime=480 simDuration=10 \
  topology=hexgrid addStaticUes=8 aerialUeRatio=0.5 rlMode=true \
  handoverAlgorithm=agent handoverMargin=3 -i 1
```

### Common Pitfalls

#### Stale shared memory after crashes

The ns3-ai interface uses POSIX shared memory (`/dev/shm/ns3-ai_*`) for C++↔Python synchronization. Each file contains spinlock semaphores (atomic counters) governing the send/receive protocol.

When training is killed or crashes (SIGABRT, Ctrl+C), the ns-3 subprocesses die without resetting their semaphores. The counters get stuck mid-protocol — the next training run opens the same segments, finds semaphores in "waiting" state, and **deadlocks forever** (zero CPU, zero progress, no errors logged).

**Fix**: always clean shared memory after a crash or before starting a new training run:
```bash
rm -f /dev/shm/ns3-ai_*
```
Also check for leftover ns-3 processes:
```bash
pgrep -af "ns3-dev-defiance" && pkill -9 -f "ns3-dev-defiance"
```

#### Iteration time vs step cadence
At 480ms (MS480), each 30s sim produces ~63 steps. Sync round-trips (C++ `InferAction()` → Python NN forward → action back) dominate wall time.

- **63 steps × ~0.7s round-trip = ~44s blocking** + 30s sim = ~74s wall per worker
- With 7 parallel workers + learner training, iterations take ~150s

If iteration time is too high: reduce `parallel` (less semaphore contention), increase `simDuration` (more steps per worker before training), or switch to MS240.

#### `sample_timeout_s` — rollout workers timing out
```
WARNING rollout_ops.py:122 -- No samples returned from remote workers.
```
The rollout worker took longer than the configured timeout. With 480ms cadence and `batch_mode="complete_episodes"`, each episode runs until sim end (30s).

**Fix**: omit `-st` entirely (defaults to `None` = no timeout), or pass `-st auto`.

#### Parallel workers trade-off

| Scenario | Best config | Reason |
|----------|-------------|--------|
| No timeouts | 7 workers | More episodes/iteration |
| High iteration time | 4 workers | Less semaphore contention per round-trip |
| Memory pressure (< 16GB) | 4 workers | Each ns-3 process uses ~1-2GB |
| First training run | 4 workers | Conservative start |

#### Observation space errors (worker crashes)
```
ValueError: Observation (...) outside given space (...)
```
The obs app sent a value outside the declared space bounds. Most common: `cwnd` exceeds `MaxCwnd` (now 1000000 for BBR byte-level cwnd). See §3j for cwnd fix.

#### Stale ns-3 processes from previous runs
If you forget to `ray stop` between training runs, old ns-3 processes accumulate unseen in the background. Each consumes 40-65% of a core. With ~20 stale processes, they fight the current training run for CPU and inflate wall-clock time by **10-14×**.

**Symptom**: `Simulation time` reports 400-500s wall time for a 10s sim, even though standalone mode finishes in 34s. `ps aux` shows many `ns3-dev-defiance-tcp-harl` processes with different `--runId` values.

**Fix**: Always clean up between runs:
```bash
ray stop
pkill -f "defiance-tcp-harl"   # kill any leftover ns-3 processes
```

**Check for stale processes**:
```bash
ps aux | grep "defiance-tcp-harl" | grep -v grep | wc -l
# Should be 0 before starting a new training run
```

#### Continuing training from a checkpoint
Ray 2.x `Tuner.restore()` preserves the original experiment's stop condition (`training_iteration`). If the original run had 10 iterations, restoring and calling `.fit()` will immediately terminate.

**Workaround**: Manually edit the experiment state file to raise the iteration cap before restoring:
```bash
# 1. Find the experiment state file
ls ~/ray_results/PPO_*/experiment_state-*.json

# 2. Edit the "stop" condition inside (change 10 to your desired total):
#    "stop": {"training_iteration": 30}

# 3. Run restore — training continues until the new limit
run-agent train -n defiance-tcp-harl -st 600 \
  -a ~/ray_results/PPO_2026-06-17_14-20-18/ \
  -c parallel=8 simDuration=40 rlMode=true handoverAlgorithm=agent \
  topology=hexgrid addStaticUes=6 aerialUeRatio=0.7 -i 30
```
The `-i` value should match the new `training_iteration` in the state file.

---

## 6. Current State & Known Issues

### Working
- [x] Standalone mode: TCP connects at ~1.035s, handovers via A3, CSV logging
- [x] RL mode: apps install, shared memory communication works
- ~~TCP guard: no handovers before TCP connection~~ (removed — circular dependency with NoOp + RL agent)
- [x] Handover-in-progress guard: prevents double-dispatch at same sim tick (fixes NS_FATAL crash)
- [x] Margin gating: RSRP threshold filters marginal handovers
- [x] TCP failure penalty: reward penalized when connection dies (event-driven via state trace)
- [x] RLF penalty: one-shot penalty on RLF detected via RRC state transition
- [x] Ping-pong penalty: extra cost for handovers within 1s window
- [x] RSRQ in observation: per-cell RSRQ (dB) from UE PHY trace
- [x] Early app start: RL apps start at 0.5s (500ms for agent to act before TCP)
- [x] Periodic observations: 480ms timer sends observation (decoupled from PHY trace)
- [x] PHY trace source: UE `ReportUeMeasurements` (dBm/dB, all detectable cells) replaces eNB RRC (3GPP indices, A3-only)
- [x] RNTI caching: lazy-init avoids GetObject overhead on every 200ms PHY callback
- [x] Seed advancement: unique seeds across worker restarts
- [x] Antenna clamping fix: prevents negative thetaEff crash
- [x] Inference mode: RLModule.from_checkpoint() restores trained weights for standalone evaluation
- [x] Episode lifecycle: `NotifySimulationEnd` correctly signals truncated/terminated
- [x] Reset cycle: experiment.run() starts new ns-3 subprocess on each reset
- [x] Handover decisions: agent outputs valid handover targets from dBm RSRP/RSRQ observations

### Open Issues
1. **Agent not learning effectively**: 100-iteration (old config) PPO run shows episode returns [-101, 262] with mean 75. Returns are noisy and the value function struggles (vf_loss clipped at 10.0). See §12 for hyperparameter analysis.
2. **TCP CCA agent**: `tcp-cca/` stubs need implementation (separate future work).
3. **RL-doc/README.md outdated**: still documents removed fields (bwEstimate, inflightBdp, bbrState, rrcState, cwnd, deliveryRate) and old 3GPP-index RSRP/RSRQ ranges.
4. **Iteration time**: ~600s per iteration with parallel=2, hexgrid topology, 10s simDuration. Sync round-trips dominate (0.5-0.8s per step × 21 steps per episode). See §3m.
5. **Worker stalls (parallel≥4)**: ns-3 startup race conditions when ≥4 workers reset simultaneously. Mitigated by `SIMULATION_EARLY_ENDING=2.0s` and `RuntimeError` retry.

### Key Global Variables
```cpp
// Defined in harl-tcp-scenario.cc
bool g_tcpConnected;                  // Set by NotifyConnectionSucceeded
bool g_tcpAlive;                      // True while TCP connection is alive
bool g_handoverInProgress;            // True while a handover is being prepared
bool g_rlfTriggered;                  // True when RLF detected (cleared after one reward step)
uint32_t g_totalHandovers;            // Incremented on each HandoverRequest
uint64_t g_totalRxBytes;              // Accumulated by SinkRxPacket callback
uint32_t g_totalRetransmissions;      // Accumulated by SourceRetransmissionPacket callback
double g_rttSumMs;                    // Sum of RTT samples for average
uint32_t g_rttSamples;                // Count of RTT samples
std::vector<double> g_lastRsrpValues; // [cellId] = RSRP in dBm (-140 = unknown)
std::vector<double> g_lastRsrqValues; // [cellId] = RSRQ in dB  (-20 = unknown)
std::vector<double> g_lastSinrValues; // [cellId] = SINR in dB (-40 = unknown)
NetDeviceContainer g_uavLteDevs;      // UAV's LTE device
NetDeviceContainer g_enbLteDevs;      // All eNB LTE devices
Ptr<LteHelper> g_lteHelper;           // Global LTE helper for HandoverRequest
```

---

## 7. External Dependencies Changed

### ns-3-ai
| File | Change |
|------|--------|
| `contrib/ai/model/gym-interface/ns3-ai-gym-msg.h` | `MSG_BUFFER_SIZE 1024 → 2048` |
| `contrib/ai/model/msg-interface/ns3-ai-msg-interface.h` | `m_size 4096 → 8192` |
| `contrib/ai/model/gym-interface/py/ns3ai_gym_env/envs/ns3_environment.py` | `shmSize 4096 → 8192`, added `_get_init_run_id()` |

### ns-3 core
| File | Change |
|------|--------|
| `src/antenna/model/three-gpp-antenna-model-oriented.cc` | Add `std::fabs(thetaEff)` + `>180` reflection |

---

## 8. TensorBoard Metrics Reference

Track these in TensorBoard (`tensorboard --logdir ~/ray_results/`) to diagnose training health.

### Primary Metrics

| Metric | Healthy Range | What It Means | If Broken |
|--------|---------------|---------------|-----------|
| `episode_return_mean` | Increasing over time | Average total reward per episode. Should trend upward as policy improves. | Flat or decreasing → reward scaling off, value function broken, or exploration not working. |
| `episode_len_mean` | ~277-302 (full episode) | Steps per episode before truncation. Shorter episodes = early termination (RLF, TCP failure). | Consistently short → agent causes RLF frequently; check handover policy and RLF penalty weight. |
| `vf_loss` | Decreasing, < 10.0 | Value function MSE loss. Should fall as value predictions improve. | **Stuck at 10.0** → TD errors exceed `vf_clip_param`, value function learns nothing. Fix reward scaling. |
| `vf_explained_var` | > 0.2 (positive) | Fraction of return variance explained by value function. 1.0 = perfect. | **Near 0 or negative** → value function predictions are worse than a constant. Same root cause as vf_loss. |
| `entropy` | Slowly decreasing | Policy randomness. Starts high (~ln(22)=3.09 for Discrete(22)), should gradually fall as policy converges. | **Drops to 0 too fast** → policy collapsed to deterministic too early, insufficient exploration. Lower lr or increase entropy_coeff. **Stays high** → policy never commits to any action. |

### Training Hyperparameters (Default Code Values)

| Parameter | PPO | DQN/D3QN | Effect |
|:---|---:|:---:|:---|
| `lr` | 0.0005 | 0.0003 | Standard PPO learning rate |
| `entropy_coeff` | 0.01 | — | Increased from 0.001; encourages exploration in Discrete(22) action space |
| `kl_target` | 0.01 | — | Standard KL divergence target (adaptive when use_kl_loss=True) |
| `use_kl_loss` | True | — | Adaptive KL penalty instead of fixed clipping; handles noisy rewards better |
| `grad_clip` | 10.0 | 100.0 | Global gradient norm clipping |
| `clip_rewards` | 1.0 | — | Clips rewards to [-1, 1]; prevents reward scale from destabilizing value function |
| `gamma` | 0.99 | 0.99 | Standard discount factor |
| `lambda` (GAE) | 0.95 | — | Standard GAE trace length |
| `vf_clip_param` | 5.0 | — | Value function clipping threshold; must match return magnitude |
| `clip_param` | 0.2 | — | Policy clipping (standard PPO) |
| `train_batch_size` | auto | 2048 | Samples per SGD update |
| `target_network_update_freq` | — | 1000 | Steps between target Q network sync (DQN/D3QN) |
| `replay_buffer_capacity` | — | 50000 | Experience replay size (DQN/D3QN) |
| `num_steps_sampled_before_learning_starts` | — | 1000 | Fill replay buffer before training starts (DQN/D3QN) |

### Secondary Metrics

| Metric | Healthy Range | What It Means | If Broken |
|--------|---------------|---------------|-----------|
| `policy_loss` | Varies, should be negative early then near 0 | Policy gradient loss. Slightly negative = policy is improving on average. Large positive = unstable updates. | **Large positive spikes** → advantage estimates are noisy; check batch size, GAE lambda, or reward variance. |
| `grad_norm` | < `grad_clip` (10.0) | L2 norm of gradient before clipping. | **Consistently at clip limit** → gradients exploding, reduce lr or increase grad_clip. **Near 0** → gradients vanished (unlikely with tanh). |
| `kl_divergence` | ~0.01 (target) | KL div between old and new policy per update. | **Too high (> 0.1)** → policy changing too fast per update, reduce `lr` or increase `kl_target`. |
| `num_episodes` | Steadily increasing | Total episodes completed. Compare with `episode_return_mean` — if episodes are short but return is flat, the agent is crashing early. | **Not increasing** → workers stuck or dead. Check rollout worker logs. |
| `learner_throughput` | Stable | Samples/second processed by learner. | Dropping → replay buffer bottleneck (DQN) or GPU not utilized (check `num_gpus` vs `num_gpus_per_learner`). |

### Domain-Specific (ns-3 End-of-Sim Output)

| Metric | What It Tracks | Notes |
|--------|----------------|-------|
| `Total received (Mbit)` | Cumulative UL goodput | Correlates with `episode_return_mean` via `normGoodput` term. Low but handovers working → agent moving too much, wasting throughput with handover overhead. |
| `Total handovers` | Handover count per episode | Should be moderate (5-15 for 30s episode). Too high → handover storms. Too low (0) → agent learned no-op, or TCP never connected. |
| `Average RTT (ms)` | Mean TCP RTT | Low RTT = good signal, low bufferbloat. Consistently > 200ms → congestion or poor cell choice. |
| `Total retransmissions` | TCP retransmission count | Spikes → poor radio conditions or handovers causing packet drops. Should decrease as policy improves. |

### Debugging Workflow

1. **If `vf_loss` stuck at clip limit** → reward scale is wrong. Check `episode_return_mean`: if returns are -1500 and `vf_clip_param=10`, TD errors will always exceed the clip.
2. **If `episode_return_mean` is flat** → agent isn't learning. Check `entropy`: if near 0, policy collapsed; if still ~3.0, exploration is fine but policy gradients are too weak (increase lr).
3. **If `episode_len_mean` is dropping** → agent causes RLF more often as training progresses. Either the RLF penalty is too weak, or the handover policy is getting worse.
4. **If workers keep dying** → check the `ValueError` in worker logs. Usually means observation value outside declared space (e.g., cwnd > MaxCwnd).

### TCP Connection Timing Pattern

TCP connection succeeds at deterministic `1.035s`, `4.035s`, or `10.035s` across different seeds. The `.035s` offset is the TCP handshake (35ms after BulkSendApp start at 1.0s). The three anchors are **SYN retransmission timeouts**:

| Timestamp | Mechanism |
|-----------|-----------|
| `1.035s` | First SYN succeeds immediately — clean cell, uninterrupted handshake |
| `4.035s` | 1.0s + **3.0s** (first SYN RTO) + 35ms — SYN retransmitted once |
| `10.035s` | 1.0s + **3.0s** + **6.0s** (doubled RTO) + 35ms — SYN retransmitted twice |

The 3s/6s doubling is standard TCP exponential backoff for SYN timeouts. Different seeds hit different timestamps because the agent's early handover (at `0.52s`, before TCP starts) can land on a cell with **no working S1/X2 path to the PGW**. The RRC handover succeeds (`HandoverEndOk` fires) but the data plane is broken — SYN packets never reach the remote host.

**If you see predominantly 4.035s or 10.035s seeds**, the early handover policy is consistently moving the UE to cells with broken backhaul. This is a strong negative signal for the reward: those episodes waste the first 3-9 seconds of TCP without any data transfer. The agent should learn to avoid such cells or delay handover until after TCP connects.

**To debug**: enable `--logging=true` and check which cell the agent hands over to at ~0.52s vs where the TCP packet sink is reachable. Not all eNBs in the topology may have S1 connectivity to the PGW.

---

## 9. Next Steps

1. **Test RL training** — run-agent with TCP guard removed, handover-in-progress guard + margin gating
2. **Tune hyperparameters** — handoverMargin, tcpFailurePenalty, reward weights, stepTime
3. **Implement TCP CCA agent** — see `tcp-cca/` stubs; separate action space (cwnd/bw scaling), separate observation space (TCP internal state), separate dual-mode design
4. **Update RL-doc** — sync with current observation space (cwnd bound now 1000000 instead of 65535)

## 10. Checkpoint Continuation

Ray 2.55.1's `Tuner.restore()` ignores `run_config`, so to continue training past the original `--iterations` count you must manually edit the experiment state file.

### Manual Edit Steps

1. Open `<ray_results>/<run>/experiment_state-*.json` in a text editor
2. Change `"status": "TERMINATED"` to `"status": "PAUSED"`
3. Change `"done": true` (near `"training_iteration": N`) to `"done": false`
4. Ensure `"stopping_criterion"` has the desired iteration limit (e.g., `"training_iteration": 200`)
5. Save and run the restore command

**Important**: The checkpoint must be from the **exact same code version**. If you changed the observation space (added/removed fields, changed dimensions), the old checkpoint's network weights won't match the new input layer and restore will fail.

---

## 11. Inference (Standalone Model Evaluation)

Inference runs a trained checkpoint against a fresh ns-3 simulation without any training. The RL agent makes decisions but does not update its weights.

### How it works

**Command**:
```bash
cd contrib/defiance && eval $(poetry env activate)
run-agent infer -n defiance-tcp-harl \
  -a /home/nisaak/ray_results/PPO_2026-06-17_19-42-14/ \
  -c simDuration=20 addStaticUes=4 aerialUeRatio=0.5 seed=10 \
     handoverAlgorithm=agent rlMode=true logging=true parallel=1
```

**Checkpoint path resolution**: The `-a` argument accepts either:
- An experiment directory (e.g., `PPO_2026-06-17_19-42-14/`) — automatically appends `best_checkpoint/`
- A direct checkpoint directory (e.g., `checkpoint_000099/`)

**Architecture**: The `start_inference()` function in `model/agents/ray.py` uses two different restore paths based on the checkpoint format:

| Checkpoint format | Detection | Restore API | Used by |
|---|---|---|---|
| New RLModule stack | `learner_group/` exists | `RLModule.from_checkpoint()` | PPO (all recent runs) |
| Old Policy stack | `policies/` exists | `Algorithm.from_checkpoint()` | DQN/D3QN |

**New stack flow (PPO)**:
1. Create `Ns3MultiAgentEnv` manually (not via env runners)
2. Load `RLModule` directly from `learner_group/learner/rl_module/agent_0/` — no Ray workers needed
3. Flatten dict observations with `gym.spaces.flatten()` to match the 133-dim input
4. Run `module.forward_inference({"obs": tensor})` → get action logits
5. Sample action via `torch.argmax(logits)` (deterministic)
6. Step the environment with the action

**Key design decisions**:
- **No env runners**: Loading `RLModule.from_checkpoint()` avoids creating Ray `EnvRunner` actors entirely. This is much faster and avoids the `NameNotFound: Environment 'defiance'` error that happens when the env isn't registered with `tune.register_env()`.
- **Manual observation flattening**: Since no `env_to_module_connector` is available (that requires env runners), `gym.spaces.flatten()` handles one-hot encoding discrete fields and concatenating into the flat vector.
- **Deterministic actions**: `torch.argmax()` over logits gives the greedy action. For stochastic behavior, use `torch.distributions.Categorical(logits=logits).sample()`.

**Known limitations**
- The inference loop currently assumes single-agent (agent_0). For multi-agent setups, the observation unwrapping would need per-agent flattening spaces.
- **LSTM models (`use_lstm=True`) are not supported in standalone inference.** The current `forward_inference({"obs": tensor})` call does not pass `state_in` and `state_out` tracking, causing the LSTM encoder to crash during `tokenize()` (the observation is interpreted as `[B, T]` instead of `[T, features]`). To support LSTM inference, the loop would need to:
    1. Call `module.get_initial_state()` to get zero initial states
    2. Pass `batch = {"obs": tensor.unsqueeze(0).unsqueeze(0), "state_in": init_state}` (add both batch and time dims)
    3. Extract next state from `out.get("state_out")` for the next step
    4. Squeeze the time dimension from logits before argmax

### Debugging inference

**Check what the model outputs**:
```python
import torch
from pathlib import Path
from ray.rllib.core.rl_module.rl_module import RLModule

ckpt = Path("~/ray_results/PPO_2026-06-17_19-42-14/best_checkpoint").expanduser()
module = RLModule.from_checkpoint(str(ckpt / "learner_group/learner/rl_module/agent_0"))
module.eval()

# Dummy observation (133 zeros)
dummy = torch.zeros((1, 133), dtype=torch.float32)
out = module.forward_inference({"obs": dummy})
print("Logits:", out["action_dist_inputs"])  # shape [1, 22]
action = torch.argmax(out["action_dist_inputs"], dim=-1).item()
print("Action:", action)  # 0 = no-op, 1-21 = handover to cell
```

**Action interpretation**:
```
0       = No-op (stay on current cell)
1..21   = Handover to target cell ID
```

---

## 12. Hyperparameter Analysis (PPO)

### Current config (as of 2026-06-22)
```python
lr=0.0003, entropy_coeff=0.01, grad_clip=10.0,
vf_clip_param=100.0, num_epochs=10, use_kl_loss=True,
gamma=0.99, clip_rewards=1.0, kl_coeff=0.2, kl_target=0.01
```

### Reward function (as of 2026-06-22)
```
reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm

normGoodput:
  1.0                           if goodput >= referenceRate (5 Mbps)
  (g - R_min) / (R_ref - R_min) if R_min <= goodput < R_ref  [0..1]
  (g - R_min) / R_min           if goodput < R_min (2.5 Mbps)  [-1..0]

rttPenalty:
  0.0                                    if rtt <= delayMinRtt (55 ms)
  (rtt - D_min) / (D_max - D_min)        if D_min < rtt < D_max  [0..1]
  1.0                                    if rtt >= maxAcceptableRtt (100 ms)
```
No explicit handover penalty or ping-pong penalty. Penalties are purely self-normalized ramps.

### RSRP noise-floor threshold
All cells with RSRP <= -110 dBm are blocked from handover in the action mask (both with and without margin gating).

### Key parameters
| Parameter | Value | Rationale |
|---|---|---|
| `lr` | 0.0003 | Reduced from 0.0005 for more stable updates |
| `vf_clip_param` | 100.0 | Increased from 5.0 to accommodate clipped reward range [-1,1] after discounting |
| `num_epochs` | 10 | More passes per batch for better value function convergence |
| `entropy_coeff` | 0.01 | Encourages exploration without overwhelming policy gradient |
| `clip_rewards` | 1.0 | Clamps per-step rewards to [-1, 1] |
| `batch_mode` | `complete_episodes` | ns-3 requirement — cannot truncate mid-episode |

### SAC hyperparams (as of 2026-06-22)
```python
actor_lr=3e-5, critic_lr=3e-4, tau=0.005, gamma=0.99,
train_batch_size=256, n_step=1, initial_alpha=1.0,
clip_rewards=1.0, batch_mode=complete_episodes,
replay_buffer: MultiAgentPrioritizedReplayBuffer (50K capacity),
simple_optimizer=True,
api_stack: old (enable_rl_module_and_learner=False)
```
SAC is off-policy: no `num_epochs`, no `vf_clip`, no `entropy_coeff`. Entropy is controlled by the adaptive `alpha` parameter. Learning starts after 1500 warmup steps (`num_steps_sampled_before_learning_starts`).

---

## 13. Future Exploration Ideas

Ideas to revisit when the agent shows signs of life after the current hyperparameter fixes.

### Reward shaping
| Idea | Rationale | Priority |
|------|-----------|----------|
| **RSRP improvement bonus**: `+0.01 × (rsrp_serving_current − rsrp_serving_prev)` | Gives the agent a small positive gradient toward better cells before goodput materializes. Counters the "no-op is safer" problem. | High |
| **Ping-pong escalation**: Increase ping-pong penalty exponentially on repeated rapid handovers | Linear penalty (0.05) may not deter 3-4 rapid handovers. Exponential punishes oscillation patterns. | Low |
| **Goodput in observation**: Add a short-term goodput window (e.g., 3-step moving average) as an observation field | The agent sees deliveryRate (TCP rate sample) but is rewarded on goodput (PacketSink bytes). Having both in the observation could help the agent learn the relationship. | Medium |

### Observation experiments
| Idea | Rationale | Priority |
|------|-----------|----------|
| **Frame stacking (2-3 past obs)**: Append past 2-3 flattened observations to current → 266-399 dim input | Provides temporal context without LSTM. Delta features (43/133 dims) could then be removed, recovering dimensionality. The agent could learn handover oscillation patterns from the stacked history. | Medium |
| **Drop velocity**: UAV speed is constant (~20 m/s) and may carry no predictive signal for handover decisions. Savings: 3 dims. | Low |
| **LSTM policy**: Replace the default FC network with an LSTM-based policy. Handles temporal dependencies naturally and could subsume delta features. RLlib supports this via `model_config={'use_lstm': True}`. | Medium |

### What NOT to remove
- **UE txPower**: Good proxy for UE-to-BS distance. Power control raises Tx power when far from serving cell.
- **MCS**: Correlates with theoretical max link bandwidth. Low MCS → poor radio → candidate for handover.
- **cwnd**: BBR cwnd = pacing_rate × RTT. While not fully independent, it carries TCP state information that rtt alone doesn't capture.

### Reward normalization experiments
| Idea | Rationale | Priority |
|------|-----------|----------|
| **Per-batch advantage normalization**: Normalize advantages to zero mean, unit variance within each training batch | Standard PPO practice. Would enable tight clipping values (vf_clip=0.2, grad_clip=1.0). Requires RLlib support check. | Medium |
| **Running return statistics**: Track episode return μ and σ across iterations via a custom RLlib callback | More accurate than clip_rewards=1.0. Preserves relative differences between returns. | Low |
| **Remove clip_rewards=1.0 after advantage norm**: Once advantages are normalized, raw reward clipping becomes unnecessary | Cleanup step after other normalization is in place. | Low |

## 14. Action Masking (Implemented)

### Problem

The action app silently blocks handovers that fail the margin precondition. The RL agent cannot distinguish between a no-op and a blocked handover, wasting exploration on impossible actions.

### Solution

RLlib's `ActionMaskingTorchRLModule` forces the policy to only sample from actions marked valid by an environment-generated mask (applied as `-inf` in the logits before softmax).

### Implementation (6 files changed)

#### 1. `harl-tcp-handover-obs-app.h` / `.cc`
- Added `HandoverMargin` attribute (default: `3.0`)
- Added `StepTimeMs` attribute (default: `480`, set from `stepTime` cmd arg)
- `BuildObservation()` computes `action_mask`:
  - `action_mask[0] = 1` (no-op always valid)
  - `action_mask[i] = 1` if:
    - `i != currentCellId`
    - `rsrp[target] > -110 dBm` (above noise floor)
    - `rsrp[serving] > -110 dBm`
    - `rsrp[target] > rsrp[serving] + margin` (when margin enabled, `handoverMargin > -999`)
    - If margin is disabled (`<= -999`): only blocks same-cell and noise-floor cells
  - Added to observation dict as `Box(0, 1, (numBs+1,))` float64
- `SendObservation()` now schedules at `m_stepTimeMs` instead of hardcoded 480ms

#### 2. `ns3_multi_agent_environment.py`
- `initialize_env()` conditionally restructures observation spaces (only when `useActionMasking=true` in `ns3Settings`):
  - Extracts `action_mask` space as `Box(22,)`
  - Flattens remaining 12-key dict to `Box(117,)` via `spaces.flatten_space()`
  - Stores inner dict space for runtime flattening
  - Replaces `observation_spaces[agent]` with `Dict(\"observations\": Box(117,), \"action_mask\": Box(22,))`
- `_restructure_obs(raw_obs, agent_id)`: extracts mask, flattens inner obs, returns nested dict
- `step()` and `reset()` call `_restructure_obs()` before returning observations
- Controlled by `ns3Settings[\"useActionMasking\"]` flag set in `ray.py`

#### 3. `ray.py` — RLlib config
- Imports `ActionMaskingTorchRLModule` and `RLModuleSpec`
- PPO config uses `RLModuleSpec(module_class=ActionMaskingTorchRLModule, model_config={...})`
- `.environment()` sets `action_mask_key=\"action_mask\"` for PPO
- `FlattenObservations` connector is only attached for non-PPO algorithms (SAC, DQN, D3QN)
- `ns3_settings[\"useActionMasking\"]` flag propagated to env

#### 4. `harl-tcp-handover-act-app.cc`
- Margin precondition kept as safety guard for when `handoverMargin=-999` (disabled). The action mask handles the standard case.

#### 5. `harl-tcp-scenario-setup.cc`
- Passes `HandoverMargin` and `StepTimeMs` to the observation app via `SetAttribute`

### Constraints
- `ActionMaskingTorchRLModule` inherits from `PPOTorchRLModule` — only works with PPO.
- For SAC, a custom `ActionMaskingSACTorchRLModule` would need to be written.
- The inner `\"observations\"` must be a flat `Box` (the PPO encoder cannot handle Dict inputs).
- The observation space dimension changes from `Box(117,)` (flat dict) to `Dict(\"observations\": Box(117,), \"action_mask\": Box(22,))`.
- When action masking is disabled (SAC/DQN/D3QN), the env returns the flat dict as before and `FlattenObservations` handles flattening.

### NaN guards and observation clamping (2026-06-22)

Added `std::isnan()` guards and a static `Clamp()` helper to `BuildObservation()` in `obs-app.cc`:

- **RSRP/RSRQ/SINR**: NaN values replaced with sentinels (`-140.0` / `-20.0` / `-40.0`) before entering the observation dict. The sentinel was already being replaced in `AddValue()`, but `NaN` from the LTE stack could still propagate through the delta computation and snapshot storage.
- **Delta computation**: Uses nan-guarded values (`curRsrp`, `curRsrq`) from the same loop rather than raw `m_rsrpValues[i]`. Previous snapshots (`m_lastRsrpSnapshot`) may still contain NaN values from earlier callbacks — the `> -110.0` threshold naturally filters these out (NaN comparison returns false).
- **SINR delta**: Added explicit `std::isnan(m_lastSinrSnapshot)` check to prevent `NaN - val` cascade.
- **Observation clamping**: Every field in the observation dict is clamped to its declared space bound before adding to the container:

  ```cpp
  static double Clamp(double val, double lo, double hi) {
      return val < lo ? lo : (val > hi ? hi : val);
  }
  ```

  | Field | Clamp range |
  |-------|-------------|
  | `rsrps` | [-160, -40] |
  | `rsrqs` | [-100, -3] |
  | `rsrpDelta` | [-60, 60] |
  | `rsrqDelta` | [-60, 60] |
  | `sinr` | [-40, 50] |
  | `sinrDelta` | [-20, 20] |
  | `velocity` | [-200, 200] |
  | `txPower` | [-50, 50] |
  | `mcs` | [0, 31] |
  | `rtt` | [0, 10000] |

**Why clamping instead of widening bounds**: The LTE stack can produce RSRQ values as low as `-101` dB (RSSI drops much faster than RSRP for very weak cells). Widening bounds would chase these outliers to infinity. Clamping at the source guarantees the observation space is always satisfied, which is critical for RLlib's old API stack (old API stack validates `Preprocessor.transform()` observations against the space).

## 15. SAC Algorithm Support

### Overview

SAC (Soft Actor-Critic) is an off-policy RL algorithm with a stochastic actor and twin Q critics. Key differences from PPO:

| Aspect | PPO | SAC |
|--------|:---:|:---:|
| Policy type | On-policy | Off-policy |
| Replay buffer | No (uses on-policy batches) | Yes (reuses past experience) |
| Sample efficiency | Low | High (much better) |
| Q critics | Single value function | Twin Q networks (min ensemble) |
| Entropy | Fixed `entropy_coeff` | Adaptive temperature (`alpha`) |
| Batch size | ~2000 steps | ~256 transitions (from buffer) |
| Steps before learning | Immediate | ~1500 warmup steps |
| Training intensity | 1:1 (one batch per sample) | Configurable (N batches per sample) |
| num_epochs | 5-10 | N/A (single pass) |
| GAE/lambda | Yes | No |
| vf_clip / grad_clip | Used | N/A |
| batch_mode | `complete_episodes` | `truncate_episodes` (but ns-3 needs complete) |

### RLlib defaults
```python
SACConfig().training(
    twin_q=True,
    initial_alpha=1.0,
    alpha_lr=0.0003,
    target_entropy="auto",
    actor_lr=3e-5,
    critic_lr=3e-4,
    tau=0.005,
    n_step=1,
    num_steps_sampled_before_learning_starts=1500,
    replay_buffer_config={
        "type": "PrioritizedEpisodeReplayBuffer",
        "capacity": 1000000,
        "alpha": 0.6,
        "beta": 0.4,
    },
)
```

### Implementation (as of 2026-06-22)

#### 1. `_build_sac_config` in `ray.py`

SAC uses the **old API stack** (`enable_rl_module_and_learner=False`, `enable_env_runner_and_connector_v2=False`) and `MultiAgentPrioritizedReplayBuffer`:

```python
def _build_sac_config(
    base_config: AlgorithmConfig,
    ns3_settings: dict[str, Any],
    env: Ns3MultiAgentEnv,
) -> AlgorithmConfig:
    base_config.clip_rewards = 1.0
    base_config.batch_mode = "complete_episodes"
    base_config.simple_optimizer = True
    return (
        base_config
        .api_stack(enable_rl_module_and_learner=False, enable_env_runner_and_connector_v2=False)
        .resources(num_gpus=1 if HAS_GPU else 0)
        .training(
            twin_q=True,
            initial_alpha=1.0,
            alpha_lr=0.0003,
            target_entropy="auto",
            actor_lr=3e-5,
            critic_lr=3e-4,
            tau=0.005,
            n_step=1,
            gamma=0.99,
            train_batch_size=256,
            num_steps_sampled_before_learning_starts=1500,
            store_buffer_in_checkpoints=False,
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 50000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
            },
        )
    )
```

#### 2. `_BUILDERS` dict

```python
_BUILDERS = {
    "PPO": _build_ppo_config,
    "SAC": _build_sac_config,
    "DQN": _build_dqn_config,
    "D3QN": _build_d3qn_config,
}
```

#### 3. Old API stack vs new API stack

| Aspect | New API stack (PPO) | Old API stack (SAC) |
|--------|:-------------------:|:-------------------:|
| Learner | `RLModule` + `Learner` | `Policy` (e.g. `SACTorchPolicy`) |
| Env runner | `MultiAgentEnvRunner` | `RolloutWorker` |
| Connectors | `ConnectorV2` pipeline | Preprocessors + agent connectors |
| Replay buffer | `PrioritizedEpisodeReplayBuffer` | `MultiAgentPrioritizedReplayBuffer` |
| Training step | `_training_step_new_api_stack()` | `_training_step_old_api_stack()` (inherited from DQN) |
| Checkpoint format | `learner_group/learner/rl_module/` | `policies/default_policy/` |

#### 4. Key learnings

- **`simple_optimizer=True`**: Prevents `multi_gpu_train_one_step` from partitioning the batch, avoiding `AssertionError` in `update_priorities_in_replay_buffer` where `batch_indices` and `td_error` lengths differ.
- **`MultiAgentPrioritizedReplayBuffer`** (not `PrioritizedEpisodeReplayBuffer`): The old API stack training loop expects `ReplayBuffer`-style sampling (`sample_min_n_steps_from_buffer`), not episode-based storage. `PrioritizedEpisodeReplayBuffer` is only compatible with the new API stack.
- **No LSTM**: Commented out `use_lstm=True` — the old API stack with `FlattenObservations` produces a flat `Box(117,)` obs that would be misinterpreted as `[B=1, T=117]` by the LSTM encoder's `tokenize()` function, causing `AssertionError: The first dimension of the tensor must be equal to the product of the desired batch and time dimensions. Got 256 and 117.`
- **No `rl_module()` config**: The old API stack uses `ModelCatalog` / `Policy` abstraction, not `RLModuleSpec`.

#### 5. Observation space validation

The old API stack validates observations against the space using `Preprocessor.check_shape()` inside `obs_preproc.py`. The `PrioritizedEpisodeReplayBuffer` stores raw dict observations and the old API stack connector pipeline passes them through `FlattenObservations` + `Preprocessor`. If any obs value exceeds its Box bound, `check_shape` raises `ValueError`. This was the motivation for the clamping fix in §14.

### Usage

```bash
run-agent train -n defiance-tcp-harl -c parallel=4 simDuration=64 ... -t SAC -tbs 256 -st 600 -i 100
```

### Considerations

- **`batch_mode="truncate_episodes"` is incompatible**: Overridden to `complete_episodes`.
- **Warmup overhead**: ~1500 steps before learning starts (= 2 episodes at 272 steps/episode x 7 workers = 1 round). Negligible.
- **Replay buffer memory**: 50K capacity x 117 floats x 4 bytes = ~23 MB.
- **No GAE/vf_clip/clip_param**: Main hyperparams are `actor_lr`, `critic_lr`, `tau`, `initial_alpha`.
- **No action masking**: SAC uses `FlattenObservations` (not `ActionMaskingTorchRLModule`). The env receives `useActionMasking=false` and strips `action_mask` from the obs dict before returning. A custom `ActionMaskingSACTorchRLModule` would be needed.

## 16. Latest Fixes (2026-06-22)

### Seed placement fix
`RngSeedManager::SetSeed()` moved to the top of `scenarioSetup()` before any random variable creation. Previously, UAV mobility waypoints were generated using the default (identical) seed because `SetSeed()` was called ~200 lines too late.

### Random UAV start position
Both `ascend-random` and `random-waypoint` mobility models now sample the initial position uniformly from the eNB bounding box:
```cpp
// Before: fixed center
Vector startPos((bbox.minX + bbox.maxX) / 2.0, (bbox.minY + bbox.maxY) / 2.0, 1.5);
// After: random uniform
Vector startPos(rbx->GetValue(), rby->GetValue(), 1.5);
```
Each `--seed` value produces a different starting (x, y), increasing scenario diversity for RL training.

### NaN guards
All measurement values (RSRP, RSRQ, SINR) are checked for `std::isnan()` before entering the observation dict. NaN values are replaced with sentinel defaults. SINR delta computation guards against NaN in the previous snapshot. See §14 for details.

### Observation clamping
Every field in `BuildObservation()` is clamped to its declared space bound via a static `Clamp()` helper. This prevents `ValueError: Observation outside given space` in RLlib's old API stack `Preprocessor.check_shape()`. Without clamping, the LTE stack can produce extreme RSRQ values (e.g., `-101 dB`) that exceed the space bounds.

### SAC old API stack
SAC now uses the old RLlib API stack with:
- `enable_rl_module_and_learner=False`
- `enable_env_runner_and_connector_v2=False`
- `MultiAgentPrioritizedReplayBuffer` (handles multi-agent dicts correctly)
- `simple_optimizer=True` (avoids batch partitioning assert error)
- LSTM disabled (incompatible with `FlattenObservations` connector)

### SAC inference note
The `start_inference()` function detects `ActionMaskingTorchRLModule` from checkpoints and enables action masking in the env by recreating it with `useActionMasking=true`. This check is done by examining the loaded module class name — SAC checkpoints use the old API stack format (`policies/`), which is handled by the `Algorithm.from_checkpoint()` path, not the `RLModule.from_checkpoint()` path.

### Seed offset from `parallel`

In `harl-tcp-scenario.cc`, the effective seed is computed as `seed += parallel` (line 168) **before** `RngSeedManager::SetSeed()` is called. This means a standalone run and an inference run with the same `--seed` CLI value will produce different UAV paths because:

- **Standalone**: `--seed=4 --parallel=0` → effective seed = 4
- **Inference**: `--seed=4 --parallel=7` (from ns3_settings) → effective seed = 11

The `parallel` value cannot be set to 0 during inference because it controls the number of Ray workers (`num_env_runners`). To reproduce a specific UAV trajectory from a standalone run during inference, adjust the CLI seed: `--seed=(standalone_seed - parallel)`.

Also, when the old API stack (SAC/DQN/D3QN) creates `RolloutWorker` actors via `Algorithm.from_checkpoint()`, each worker's env receives its own `parallel=worker_index` from the `create_env()` factory function, further diverging seeds across workers: worker 1 gets effective seed `X+1`, worker 2 gets `X+2`, etc. The manually created inference env (at the top of `start_inference()`) keeps the original `parallel` from ns3_settings (e.g., 7), producing yet another effective seed.

### Working SAC training command (backup)

Verified working configuration (2026-06-22, old API stack):

**Command**:
```bash
rm -f /dev/shm/ns3-ai_*
run-agent train -n defiance-tcp-harl \
  -c parallel=7 simDuration=64 topology=hexgrid addStaticUes=0 \
     rlMode=true handoverAlgorithm=agent handoverMargin=-999 \
     tcpFailurePenalty=0.5 rlfPenalty=1.0 --useActionMasking=false \
  -t SAC -tbs 256 -st 600 -i 100
```

**Config breakdown**:

| Parameter | Value | Note |
|-----------|-------|------|
| Algorithm | SAC | Off-policy, twin Q, adaptive entropy |
| API stack | old | `enable_rl_module_and_learner=False` |
| `num_env_runners` | 7 | From `--parallel=7` |
| `batch_mode` | `complete_episodes` | Required by ns-3 |
| `clip_rewards` | 1.0 | Clamp per-step reward to [-1, 1] |
| `simple_optimizer` | True | Avoids multi-GPU batch partitioning assert |
| `twin_q` | True | Twin Q critics (standard SAC) |
| `actor_lr` | 3e-5 | Actor learning rate |
| `critic_lr` | 3e-4 | Critic learning rate (10x actor) |
| `tau` | 0.005 | Target network update rate |
| `gamma` | 0.99 | Discount factor |
| `n_step` | 1 | Single-step TD (not n-step) |
| `initial_alpha` | 1.0 | Initial entropy temperature |
| `target_entropy` | `"auto"` | Auto-computed from action space |
| `train_batch_size` | 256 | Samples per gradient update (from replay buffer) |
| `num_steps_sampled_before_learning_starts` | 1500 | Warmup steps before first update |
| Replay buffer | `MultiAgentPrioritizedReplayBuffer` | 50K capacity, α=0.6, β=0.4 |
| `store_buffer_in_checkpoints` | False | Reduces checkpoint size |
| `useActionMasking` | false | SAC uses FlattenObservations (no action mask) |
| `num_gpus` | 1 | Single GPU |
| LSTM | disabled | Incompatible with FlattenObservations connector |
