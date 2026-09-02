#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h" // Waypoint for the g_uavWaypoints plan (needed before the globals)
#include "ns3/nr-module.h"

using namespace ns3;

double ueSpeed = 20.0;            // m/s
double simDuration = 50.0;        // seconds
uint32_t bandwidthMhz = 30;        // MHz
uint32_t numerology = 1;           // 0 = 15 kHz SCS, 1 = 30 kHz SCS
double trafficRateMbps = 50.0;   // UAV OnOff data rate (Mbps)
double intersiteDistance = 500.0; // m
uint32_t numMacroCells = 7;       // number of macro cells
double gnbDowntilt = 10.0;        // degrees
uint32_t seed = 0;                // Seed for RNG
uint32_t runId = 0;
std::string trialName = "1";
std::string tcpVariant = "TcpBbr";
std::string uavMobility = "random-waypoint";
std::string topology = "triangle";
double startHeight = 50.0; // m
double endHeight = 200.0;  // m (max Z for random-waypoint)
uint32_t bbrWindowLength = 10;
uint32_t g_addInterferingUes = 0;
double g_aerialUeRatio = 0.0;
std::string g_interfererMobility = "static";
bool g_logging = false;
bool rlMode = false;
std::string handoverAlgorithm = "a3";
uint32_t stepTime = 400; // ms (observation cadence)
uint32_t delay = 0;      // ms
double rlAlphaGoodput = 0.8;              ///< Deng-style: weight for goodput term [0,1]
double rlBetaGoodput = 5.0;              ///< Deng-style: goodput sensitivity
double rlBetaHandover = 5.0;             ///< Deng-style: handover sensitivity
std::string rlRewardComposition = "multiplicative"; ///< Reward combination: additive | multiplicative
double rlPingPongMultiplier = 5.0;            ///< betaHandover multiplier on A->B->A ping-pong
uint32_t rlHandoverHangoverLength = 1;        ///< Reward tax duration (steps) per handover event
uint32_t rlHandoverRateWindowMs = 10000;      ///< Sliding window for the ho_count_10s obs count (ms)
double rlRewardRefMbps = 15.0;                ///< Reward/obs goodput reference (Mbps): normG = goodput/ref
std::string rlRewardGoodputShape = "compl_pwr";   ///< R_G shape: deng | linear | exp_decay | compl_pwr
double rlRewardGoodputAlpha = 3.0;            ///< exp_decay rate
double rlRewardGoodputP = 0.4;                ///< compl_pwr exponent
uint32_t rlcTxBufferBytes = 180000;          ///< RLC TX buffer cap (0 = unlimited)
std::string errorModel = "eesm-ir-t1";       ///< PHY error model: eesm-ir-t1 (default) | eesm-ir-t2 | eesm-cc-t2 | eesm-cc-t1 | lte-mi
uint32_t channelUpdateMs = 20;               ///< channel UpdatePeriod ms (0 = disabled/module default; larger periods decorrelate the fading and square-wave the obs SINR)
std::string channelModel = "umav";          ///< channel: umav (3GPP TR 38.901 UMa-AV, default) | tworay (TwoRaySpectrumPropagationLossModel)
std::string g_flowDirection = "ul";
std::string g_transportProtocol = "udp";
uint32_t g_senderNodeId = 0;     //!< Node ID running OnOff (varies by flowDirection)
uint32_t g_receiverNodeId = 0;   //!< Node ID running PacketSink (varies by flowDirection)
int parallel = 0;
std::string outputDirCli;   // overrides default output directory

// global variables (must be defined before the #include below since
// nr-rl-handover-scenario-setup.cc references them)
NodeContainer g_uavContainer;
NodeContainer g_remoteHostContainer;
NodeContainer g_gnbContainer;
NetDeviceContainer g_uavNrDevs;
NetDeviceContainer g_gnbNrDevs;
NodeContainer g_interferingUeContainer;
NetDeviceContainer g_interferingUeNrDevs;
NodeContainer g_interferingRemoteHostContainer;
uint32_t g_totalHandovers = 0;
uint64_t g_totalRxBytes = 0;         // Total bytes received by PacketSink
uint32_t g_totalRetransmissions = 0; // Total TCP retransmissions
double g_rttSumMs = 0.0;             // Sum of RTT samples (ms) for average
uint32_t g_rttSamples = 0;           // Number of RTT samples
uint32_t g_rlfCount = 0;             // Number of RLF events
bool g_tcpConnected = false;
bool g_handoverInProgress = false;    // True while a handover is being prepared
bool g_tcpAlive = false;              // True while TCP connection is alive
bool g_rlfTriggered = false;          // True when RLF detected mid-episode
std::vector<double> g_lastRsrpValues; // per-cell RSRP in dBm (-200 = unknown)
std::vector<double> g_lastSinrValues; // per-cell SINR, direction-dependent: DL data SINR at the UE (dl mode) / UL SRS SINR at the gNB (ul & tcp modes); dB, -40 = unknown
std::vector<double> g_ulSinrSum;    // per-cell UL-SINR step accumulators (gNB RB-averaged, dB)
std::vector<uint32_t> g_ulSinrCount; // per-cell UL-SINR step sample counts
std::vector<double> g_lastRsrqValues; // per-cell RSRQ in dB  (-200 = unknown)
uint32_t g_topNCells[4] = {0, 0, 0, 0};       // Top-3 ranked cell IDs [0]=no-op, [1..3]=ranked (Top-N fixed at 3)

// Trajectory observation support: the pre-generated UAV waypoint plan and the
// observation tier. The plan is copied here by scenarioSetup when the mobility
// is installed (both topology branches); the obs-app lerps positions from it.
std::vector<Waypoint> g_uavWaypoints; ///< (time, pos) plan; piecewise-linear motion between consecutive entries
double g_simDuration = 0.0;          ///< Episode duration (s); obs-app lookahead clamp

// UL load observations (serving cell, last UL slot): RB utilization and the
// scheduled-UE count (contention). 0 when the UL scheduler is idle.
double g_ulServingRbUtil = 0.0;    ///< Serving-cell UL RB utilization fraction [0,1]
uint32_t g_ulServingSchedUe = 0;  ///< Serving-cell scheduled-UE count (last slot)

Ptr<NrHelper> g_nrHelper;
Ptr<NrPointToPointEpcHelper> g_nrEpcHelper;
Ptr<NrChannelHelper> g_nrChannelHelper;

#include "nr-rl-handover-scenario-setup.cc"

#include "ns3/applications-module.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/rl-application-helper.h"

using namespace ns3;

#include <cstdint>
#include <string>
#include <vector>

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("ueSpeed", "Speed of the UAV in m/s", ueSpeed);
    cmd.AddValue("simDuration", "Duration of the simulation in seconds", simDuration);
    cmd.AddValue("bandwidthMhz",
                 "Carrier bandwidth in MHz (10 default; 20 doubles RBs)",
                 bandwidthMhz);
    cmd.AddValue("numerology",
                 "Numerology (0 = 15 kHz SCS, 1 = 30 kHz SCS; n78 typical)",
                 numerology);
    cmd.AddValue("trafficRateMbps",
                 "UAV OnOff data rate in Mbps (default 100; below link capacity the rate "
                 "becomes the goodput ceiling and idle slots appear)",
                 trafficRateMbps);
    cmd.AddValue("intersiteDistance",
                 "Distance between the two gNodeBs in meters",
                 intersiteDistance);
    cmd.AddValue("numMacroCells",
                 "Number of Macro Cells in the Simulation, (4, 7, 19)",
                 numMacroCells);
    cmd.AddValue("gnbDowntilt", "Downtilt of the gNodeB antennas in degrees", gnbDowntilt);
    cmd.AddValue("seed", "Seed for random number generator", seed);
    cmd.AddValue("runId",
                 "Counts how often the environment has been reset (used for seeding)",
                 runId);
    cmd.AddValue("trial_name", "Trial name", trialName);
    cmd.AddValue(
        "tcpVariant",
        "TCP variant to use (TcpHarl, TcpNewReno, TcpCubic, TcpWestwoodplus, TcpVeno, TcpBbr)",
        tcpVariant);
    cmd.AddValue("uavMobility",
                 "UAV mobility: \"constant\", \"ascend-random\", or \"random-waypoint\"",
                 uavMobility);
    cmd.AddValue(
        "topology",
        "Topology: \"simple\" (2 gNBs on a line), \"triangle\" (3 sites, 3 sectors each), "
        "or \"hexgrid\" (hexagonal grid with 7 sites)",
        topology);
    cmd.AddValue("startHeight", "UAV starting altitude (m), used by all topologies", startHeight);
    cmd.AddValue("endHeight", "Maximum UAV altitude (m) for random-waypoint Z bounds", endHeight);
    cmd.AddValue("bbrWindowLength",
                 "TcpBbr BwWindowLength (RttWindowLength = bbrWindowLength * 1s)",
                 bbrWindowLength);
    cmd.AddValue("addInterferingUes",
                 "Number of background interfering UEs (aerial and/or ground) that create UL interference",
                 g_addInterferingUes);
    cmd.AddValue("aerialUeRatio",
                 "Fraction of ground UEs placed at aerial height (0=all ground, 1=all aerial)",
                 g_aerialUeRatio);
    cmd.AddValue("interfererMobility",
                 "Interferer mobility: static (hover in place, default) | waypoint (random flight)",
                 g_interfererMobility);
    cmd.AddValue("logging", "Enable CSV file logging (disable for RL training)", g_logging);
    cmd.AddValue("rlMode",
                 "Enable RL training mode (installs RL apps, disables FlowMonitor/CSV)",
                 rlMode);
    cmd.AddValue("handoverAlgorithm", "Handover algorithm (a3, noop, agent)", handoverAlgorithm);
    cmd.AddValue("stepTime",
                 "Step time in ms between RL agent decisions (only used with rlMode)",
                 stepTime);
    cmd.AddValue("delay", "Transmission delay (ms) for Simple Channel between apps", delay);
    cmd.AddValue("rlAlphaGoodput",
                 "Deng-style reward: weight for goodput term [0,1]. "
                 "1-alpha is the handover penalty weight.",
                 rlAlphaGoodput);
    cmd.AddValue("rlBetaGoodput",
                 "Deng-style reward: goodput sensitivity. "
                 "Higher = goodput drops penalised more. R_G = 1/(1+beta*(1-normG)).",
                 rlBetaGoodput);
    cmd.AddValue("rlBetaHandover",
                 "Deng-style reward: handover sensitivity. "
                 "Higher = more HO-averse. R_H = 1/(1+beta*I_ho).",
                 rlBetaHandover);
    cmd.AddValue("rlRewardComposition",
                 "Reward combination: additive (alpha*R_G + (1-alpha)*R_H) or "
                 "multiplicative (R_G * R_H)",
                 rlRewardComposition);
    cmd.AddValue("rlPingPongMultiplier",
                 "Multiplier applied to betaHandover on ping-pong (A->B->A) "
                 "patterns. Default 5.0.",
                 rlPingPongMultiplier);
    cmd.AddValue("rlHandoverHangoverLength",
                 "Number of steps the handover reward penalty persists after "
                 "a handover (I_ho stays true). Default 4. Halving (2) halves "
                 "the per-handover reward tax.",
                 rlHandoverHangoverLength);
    cmd.AddValue("rlHandoverRateWindowMs",
                 "Sliding window (ms) for the ho_count_10s observation dimension.",
                 rlHandoverRateWindowMs);
    cmd.AddValue("rlRewardRefMbps",
                 "Goodput reference for normalized goodput (normG = goodput/ref) "
                 "in Mbps. Affects both the reward (R_G) and the obs norm_goodput "
                 "feature: too low saturates normG at 1 (flat R_G), too high pins "
                 "it in the flat low region. Near the achievable throughput "
                 "(~40 Mbps) restores the gradient.",
                 rlRewardRefMbps);
    cmd.AddValue("rlRewardGoodputShape",
                 "R_G functional shape: deng (inverse, beta_G steepness), "
                 "linear (x), exp_decay (1-exp(-alpha*x), concave, maximal "
                 "gradient at low x), compl_pwr (1-(1-x)^p, convex, gradient "
                 "grows near x=1).",
                 rlRewardGoodputShape);
    cmd.AddValue("rlRewardGoodputAlpha",
                 "exp_decay rate for the reward goodput shape.",
                 rlRewardGoodputAlpha);
    cmd.AddValue("rlRewardGoodputP",
                 "compl_pwr exponent (0<p<1) for the reward goodput shape.",
                 rlRewardGoodputP);
    cmd.AddValue("rlcTxBufferBytes",
                 "RLC TX buffer cap in bytes for UM and AM (0 = unlimited)",
                 rlcTxBufferBytes);
    cmd.AddValue("errorModel",
                 "PHY error model: eesm-ir-t1 (default) | eesm-ir-t2 | eesm-cc-t2 | "
                 "eesm-cc-t1 | lte-mi (NrLteMiErrorModel)",
                 errorModel);
    cmd.AddValue("channelUpdateMs",
                 "Channel UpdatePeriod in ms (0 = disabled, module default; 20 default: "
                 "50 ms gave square-wave SINR)",
                 channelUpdateMs);
    cmd.AddValue("channelModel",
                 "Channel model: umav (3GPP TR 38.901 UMa-AV, default) | tworay "
                 "(TwoRaySpectrumPropagationLossModel — drops the 3GPP channel "
                 "matrix/spatial-consistency machinery; UMa-AV adopted (condition "
                 "+ propagation stay UMa-AV, FTR fading aliases UMa): median SINR "
                 "27 vs 25 dB but deeper fades — benchmark-only)",
                 channelModel);
    cmd.AddValue("flowDirection",
                 "TCP flow direction: ul (UAV->remoteHost) or dl (remoteHost->UAV)",
                 g_flowDirection);
    cmd.AddValue("transportProtocol",
                 "Transport protocol: tcp (OnOff+PacketSink) or udp (OnOff+PacketSink)",
                 g_transportProtocol);
    cmd.AddValue("parallel", "Number of parallel simulation runs", parallel);
    cmd.AddValue("outputDir", "Output directory for CSV files (overrides default)", outputDirCli);
    cmd.Parse(argc, argv);

    seed += parallel;

    if (rlMode)
    {
        // Both transports are valid in RL mode: the reward/obs goodput is
        // fed by the PacketSink Rx trace (transport-agnostic), and RLC AM
        // (TCP) vs UM (UDP) is chosen by the scenario setup.
        OpenGymMultiAgentInterface::Get();
        Ns3AiMsgInterface::Get()->SetTrialName(trialName);
        std::cout << "RL mode: trial_name=" << trialName << " seed=" << seed << " runId=" << runId
                  << " stepTime=" << stepTime << "ms"
                  << " handoverAlgorithm=" << handoverAlgorithm
                  << " alphaGoodput=" << rlAlphaGoodput
                  << " betaGoodput=" << rlBetaGoodput
                  << " betaHandover=" << rlBetaHandover
                  << " hangoverLength=" << rlHandoverHangoverLength
                  << " rewardComposition=" << rlRewardComposition << std::endl;
    }

    scenarioSetup(g_flowDirection,
                  g_transportProtocol,
                  ueSpeed,
                  simDuration,
                  bandwidthMhz,
                  trafficRateMbps,
                  intersiteDistance,
                  numMacroCells,
                  gnbDowntilt,
                  seed,
                  runId,
                  trialName,
                  tcpVariant,
                  uavMobility,
                  topology,
                  startHeight,
                  endHeight,
                  bbrWindowLength,
                  g_addInterferingUes,
                  g_aerialUeRatio,
                  g_interfererMobility,
                  g_logging,
                  rlMode,
                  handoverAlgorithm,
                  stepTime,
                  delay,
                  rlAlphaGoodput,
                  rlBetaGoodput,
                  rlBetaHandover,
                  rlRewardComposition,
                  rlPingPongMultiplier,
                  rlHandoverHangoverLength,
                  rlHandoverRateWindowMs,
                  rlRewardRefMbps,
                  rlRewardGoodputShape,
                  rlRewardGoodputAlpha,
                  rlRewardGoodputP,
                  rlcTxBufferBytes,
                  errorModel,
                  channelUpdateMs,
                  numerology,
                  channelModel,
                  outputDirCli);

    // Install before Simulator::Run() so the flows are observed; serialize
    // after Run to capture the final per-flow stats.
    FlowMonitorHelper flowmonHelper;
    if (g_logging)
    {
        flowmonHelper.Install(g_uavContainer);
        flowmonHelper.Install(g_remoteHostContainer);
    }

    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Stop(Seconds(simDuration));

    Simulator::Run();

    // Flush the CSV loggers so the output files are complete before the
    // summary prints (the loggers buffer writes; see LogStream in
    // nr-rl-handover-scenario-setup.cc).
    FlushLogStreams();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulation time: " << elapsed.count() << " seconds" << std::endl;
    std::cout << "Total handovers: " << g_totalHandovers << std::endl;
    std::cout << "Total received: " << (g_totalRxBytes * 8 / 1000000.0) << " Mbit" << std::endl;
    std::cout << "Total retransmissions: " << g_totalRetransmissions << std::endl;
    std::cout << "Total RLF: " << g_rlfCount << std::endl;
    double avgRttMs = (g_rttSamples > 0) ? (g_rttSumMs / g_rttSamples) : 0.0;
    std::cout << "Average RTT: " << avgRttMs << " ms" << std::endl;
    if (g_logging)
    {
        flowmonHelper.SerializeToXmlFile(
            g_outputDir + "nr-rl.flowmonitor",
            true,
            true);
    }
    if (rlMode)
    {
        OpenGymMultiAgentInterface::Get()->NotifySimulationEnd(0, {});
    }
    Simulator::Destroy();
    return 0;
}
