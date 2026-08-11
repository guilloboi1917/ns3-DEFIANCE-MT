#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h" // Remove later for RL learning
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

using namespace ns3;

double ueSpeed = 20.0;            // m/s
double simDuration = 30.0;        // seconds
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
bool g_logging = false;
bool rlMode = false;
std::string handoverAlgorithm = "a3";
uint32_t stepTime = 200; // ms (aligned with ReportUeMeasurements filter period)
uint32_t delay = 0;      // ms
double handoverPenalty = 0.01;
double rlAlphaGoodput = 0.8;              ///< Deng-style: weight for goodput term [0,1]
double rlBetaGoodput = 5.0;              ///< Deng-style: goodput sensitivity
double rlBetaHandover = 60.0;             ///< Deng-style: handover sensitivity
std::string rlRewardComposition = "additive"; ///< Reward combination: additive | multiplicative
bool useTbsObservation = false;                ///< Emit real avg TBS at obs index 26, or a constant
double rlPingPongMultiplier = 5.0;            ///< betaHandover multiplier on A->B->A ping-pong
bool rlHandoverRatePenalty = false;           ///< Windowed handover-rate (signaling budget) penalty
uint32_t rlHandoverRateWindowMs = 10000;      ///< Sliding window for the rate count (ms)
uint32_t rlHandoverRateBudget = 2;            ///< Free handovers per window before penalizing
double rlHandoverRateLambda = 0.2;            ///< Marginal penalty per excess handover in the window
uint32_t rlTopN = 3;                           ///< Action-space Top-N (non-serving cells); obs ranks 5 slots
uint32_t rlHandoverDebounceMs = 0;             ///< Min interval between executed handovers (ms); 0 = off
double rlRewardRefMbps = 40.0;                ///< Reward/obs goodput reference (Mbps): normG = goodput/ref
std::string rlRewardGoodputShape = "deng";   ///< R_G shape: deng | linear | exp_decay | compl_pwr
double rlRewardGoodputAlpha = 3.0;            ///< exp_decay rate
double rlRewardGoodputP = 0.4;                ///< compl_pwr exponent
std::string g_flowDirection = "dl";
std::string g_transportProtocol = "udp";
uint32_t g_senderNodeId = 0;     //!< Node ID running OnOff (varies by flowDirection)
uint32_t g_receiverNodeId = 0;   //!< Node ID running PacketSink (varies by flowDirection)
double handoverMargin = -5.0;
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
std::vector<double> g_lastSinrValues; // per-cell UL SRS SINR (dB, -40 = unknown)
std::vector<double> g_lastRsrqValues; // per-cell RSRQ in dB  (-200 = unknown)
uint32_t g_topNCells[6] = {0, 0, 0, 0, 0, 0}; // Top-N ranked cell IDs [0]=no-op, [1..5]=ranked

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

/**
 * Some descriptions here
 */

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("ueSpeed", "Speed of the UAV in m/s", ueSpeed);
    cmd.AddValue("simDuration", "Duration of the simulation in seconds", simDuration);
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
    cmd.AddValue("logging", "Enable CSV file logging (disable for RL training)", g_logging);
    cmd.AddValue("rlMode",
                 "Enable RL training mode (installs RL apps, disables FlowMonitor/CSV)",
                 rlMode);
    cmd.AddValue("handoverAlgorithm", "Handover algorithm (a3, noop, agent)", handoverAlgorithm);
    cmd.AddValue("stepTime",
                 "Step time in ms between RL agent decisions (only used with rlMode)",
                 stepTime);
    cmd.AddValue("delay", "Transmission delay (ms) for Simple Channel between apps", delay);
    cmd.AddValue("handoverPenalty",
                 "Reward penalty per handover in normalized [0,1] units",
                 handoverPenalty);
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
    cmd.AddValue("useTbsObservation",
                 "Emit the real avg TBS at obs index 26. false -> constant 0.0 "
                 "(removes the goodput-reward-proxy feature, layout unchanged)",
                 useTbsObservation);
    cmd.AddValue("rlPingPongMultiplier",
                 "Multiplier applied to betaHandover on ping-pong (A->B->A) "
                 "patterns. Default 5.0.",
                 rlPingPongMultiplier);
    cmd.AddValue("rlHandoverRatePenalty",
                 "Enable the windowed handover-rate (signaling budget) penalty: "
                 "on each handover event, handovers within the last "
                 "rlHandoverRateWindowMs are counted; above rlHandoverRateBudget "
                 "the step reward is multiplied by max(1 - lambda*excess, 0.05).",
                 rlHandoverRatePenalty);
    cmd.AddValue("rlHandoverRateWindowMs",
                 "Sliding window (ms) for the handover-rate penalty and the "
                 "ho_count_10s observation dimension.",
                 rlHandoverRateWindowMs);
    cmd.AddValue("rlHandoverRateBudget",
                 "Free handovers per window before the rate penalty applies.",
                 rlHandoverRateBudget);
    cmd.AddValue("rlHandoverRateLambda",
                 "Marginal reward penalty per excess handover in the window.",
                 rlHandoverRateLambda);
    cmd.AddValue("rlTopN",
                 "Action-space Top-N: 0 = stay, 1..rlTopN = handover to the "
                 "k-th best NON-serving cell. The serving cell is excluded "
                 "from the ranking. Obs keeps 5 ranked slots for context.",
                 rlTopN);
    cmd.AddValue("rlHandoverDebounceMs",
                 "Min interval between EXECUTED handovers in ms (3GPP TTT "
                 "analog). 0 = disabled. Hard rate cap: at D ms, at most "
                 "floor(W/D)+1 handovers fit in any W-window (e.g. D=6000 -> "
                 "at most 2 per 10s, exactly the rate-penalty budget).",
                 rlHandoverDebounceMs);
    cmd.AddValue("rlRewardRefMbps",
                 "Goodput reference for normalized goodput (normG = goodput/ref) "
                 "in Mbps. Affects both the reward (R_G) and the obs norm_goodput "
                 "feature: too low saturates normG at 1 (flat R_G), too high pins "
                 "it in the flat low region. Near the achievable throughput "
                 "(~40 Mbps) restores the gradient (RL-AUDIT.md §6).",
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
    cmd.AddValue("flowDirection",
                 "TCP flow direction: ul (UAV->remoteHost) or dl (remoteHost->UAV)",
                 g_flowDirection);
    cmd.AddValue("transportProtocol",
                 "Transport protocol: tcp (OnOff+PacketSink) or udp (OnOff+PacketSink)",
                 g_transportProtocol);
    cmd.AddValue("handoverMargin",
                 "RSRP margin for handover (3GPP range, ~1dB/step). "
                 "Target RSRP must > serving + margin. -999 disables.",
                 handoverMargin);
    cmd.AddValue("parallel", "Number of parallel simulation runs", parallel);
    cmd.AddValue("outputDir", "Output directory for CSV files (overrides default)", outputDirCli);
    cmd.Parse(argc, argv);

    seed += parallel;

    if (rlMode)
    {
        // RL mode requires UDP + DL
        if (g_transportProtocol != "udp")
        {
            NS_FATAL_ERROR("RL mode requires --transportProtocol=udp (got "
                           << g_transportProtocol << ")");
        }
        if (g_flowDirection != "dl")
        {
            NS_FATAL_ERROR("RL mode requires --flowDirection=dl (got "
                           << g_flowDirection << ")");
        }

        OpenGymMultiAgentInterface::Get();
        Ns3AiMsgInterface::Get()->SetTrialName(trialName);
        std::cout << "RL mode: trial_name=" << trialName << " seed=" << seed << " runId=" << runId
                  << " stepTime=" << stepTime << "ms"
                  << " handoverAlgorithm=" << handoverAlgorithm
                  << " alphaGoodput=" << rlAlphaGoodput
                  << " betaGoodput=" << rlBetaGoodput
                  << " betaHandover=" << rlBetaHandover
                  << " rewardComposition=" << rlRewardComposition << std::endl;
    }

    scenarioSetup(g_flowDirection,
                  g_transportProtocol,
                  ueSpeed,
                  simDuration,
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
                  g_logging,
                  rlMode,
                  handoverAlgorithm,
                  stepTime,
                  delay,
                  handoverPenalty,
                  handoverMargin,
                  rlAlphaGoodput,
                  rlBetaGoodput,
                  rlBetaHandover,
                  rlRewardComposition,
                  useTbsObservation,
                  rlPingPongMultiplier,
                  rlHandoverRatePenalty,
                  rlHandoverRateWindowMs,
                  rlHandoverRateBudget,
                  rlHandoverRateLambda,
                  rlTopN,
                  rlHandoverDebounceMs,
                  rlRewardRefMbps,
                  rlRewardGoodputShape,
                  rlRewardGoodputAlpha,
                  rlRewardGoodputP,
                  outputDirCli);

    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Stop(Seconds(simDuration));

    Simulator::Run();
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
        FlowMonitorHelper flowmonHelper;
        flowmonHelper.Install(g_uavContainer);
        flowmonHelper.Install(g_remoteHostContainer);
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
