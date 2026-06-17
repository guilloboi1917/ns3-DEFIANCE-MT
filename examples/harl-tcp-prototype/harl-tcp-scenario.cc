#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h" // Remove later for RL learning
#include "ns3/lte-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

double ueSpeed = 20.0;            // m/s
double simDuration = 50.0;        // seconds
double intersiteDistance = 500.0; // m
double enbDowntilt = 10.0;        // degrees
uint32_t seed = 0;                // Seed for RNG
uint32_t runId = 0;
std::string trialName = "1";
std::string tcpVariant = "TcpBbr";
std::string uavMobility = "random-waypoint";
std::string topology = "hexgrid";
double startHeight = 80.0; // m
double endHeight = 250.0;  // m
double helixRadius = 50.0; // m
uint32_t bbrWindowLength = 10;
uint32_t g_addStaticUes = 0;
bool g_fullBufferInterference = false;
double g_aerialUeRatio = 0.0;
bool g_logging = false;
bool rlMode = false;
std::string handoverAlgorithm = "a3";
uint32_t stepTime = 100; // ms
uint32_t delay = 0;      // ms
double handoverPenalty = 0.01;
double rlReferenceRate = 30000000.0; // bps
double rlRttPenaltyWeight = 0.05;
double rlMinRttMs = 30.0;
double tcpFailurePenalty = 0.5;
double rlfPenalty = 1.0;
double handoverMargin = 3.0;
int parallel = 0;

// global variables (must be defined before the #include below since
// harl-tcp-scenario-setup.cc references them)
NodeContainer g_uavContainer;
NodeContainer g_remoteHostContainer;
NodeContainer g_enbContainer;
NetDeviceContainer g_uavLteDevs;
NetDeviceContainer g_enbLteDevs;
NodeContainer g_staticUeContainer;
NetDeviceContainer g_staticUeLteDevs;
NodeContainer g_staticRemoteHostContainer;
NodeContainer g_interferingUeContainer;
NetDeviceContainer g_interferingUeLteDevs;
NodeContainer g_interferingRemoteHostContainer;

uint32_t g_totalHandovers = 0;
uint64_t g_totalRxBytes = 0;              // Total bytes received by PacketSink
uint32_t g_totalRetransmissions = 0;      // Total TCP retransmissions
double g_rttSumMs = 0.0;                  // Sum of RTT samples (ms) for average
uint32_t g_rttSamples = 0;                // Number of RTT samples
bool g_tcpConnected = false;
bool g_handoverInProgress = false;       // True while a handover is being prepared
bool g_tcpAlive = false;                 // True while TCP connection is alive
bool g_rlfTriggered = false;             // True when RLF detected mid-episode
std::vector<int32_t> g_lastRsrpValues; // per-cell RSRP (3GPP range 0-97, -1 = unknown)
std::vector<double> g_lastSinrValues;  // per-cell UL SRS SINR (dB, -40 = unknown)

Ptr<LteHelper> g_lteHelper;
Ptr<PointToPointEpcHelper> g_epcHelper;

#include "harl-tcp-scenario-setup.cc"

#include "ns3/applications-module.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
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
                 "Distance between the two eNodeBs in meters",
                 intersiteDistance);
    cmd.AddValue("enbDowntilt", "Downtilt of the eNodeB antennas in degrees", enbDowntilt);
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
                 "UAV mobility: \"constant\", \"helix\", or \"random-waypoint\"",
                 uavMobility);
    cmd.AddValue(
        "topology",
        "Topology: \"simple\" (2 eNBs on a line) or \"hexgrid\" (hexagonal grid with 7 sites)",
        topology);
    cmd.AddValue("startHeight", "UAV starting altitude (m), used by all topologies", startHeight);
    cmd.AddValue("endHeight", "UAV ending altitude (m), only used in hexgrid helix", endHeight);

    cmd.AddValue("helixRadius", "Radius of the helix in the Y direction (m)", helixRadius);
    cmd.AddValue("bbrWindowLength",
                 "TcpBbr BwWindowLength (RttWindowLength = bbrWindowLength * 1s)",
                 bbrWindowLength);
    cmd.AddValue("addStaticUes",
                 "Number of static UDP UEs placed on a circle at the topology center",
                 g_addStaticUes);
    cmd.AddValue(
        "fullBufferInterference",
        "Enable full-buffer DL traffic on one UE per eNB to create inter-cell interference",
        g_fullBufferInterference);
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
                 "Reward penalty per handover in normalized [0,1] units (default 0.1)",
                 handoverPenalty);
    cmd.AddValue("rlReferenceRate",
                 "Reference UL rate (bps) for throughput normalization (default 50 Mbps)",
                 rlReferenceRate);
    cmd.AddValue("rlRttPenaltyWeight",
                 "Weight of RTT inflation penalty term (default 0.2)",
                 rlRttPenaltyWeight);
    cmd.AddValue("rlMinRttMs",
                 "Baseline RTT (ms) for inflation calc (default 30 = 10ms PGW + 20ms LTE)",
                 rlMinRttMs);
    cmd.AddValue("tcpFailurePenalty",
                 "Reward penalty per step when TCP connection is dead (default 5.0)",
                 tcpFailurePenalty);
    cmd.AddValue("rlfPenalty",
                 "Reward penalty when RLF is detected (default 10.0)",
                 rlfPenalty);
    cmd.AddValue("handoverMargin",
                 "RSRP margin for handover (3GPP range, ~1dB/step). "
                 "Target RSRP must > serving + margin. -999 disables.",
                 handoverMargin);
    cmd.AddValue("parallel", "Number of parallel simulation runs", parallel);
    cmd.Parse(argc, argv);

    seed += parallel;

    if (rlMode)
    {
        OpenGymMultiAgentInterface::Get();
        Ns3AiMsgInterface::Get()->SetTrialName(trialName);
        std::cout << "RL mode: trial_name=" << trialName << " seed=" << seed << " runId=" << runId
                  << " stepTime=" << stepTime << "ms"
                  << " handoverAlgorithm=" << handoverAlgorithm << std::endl;
    }

    scenarioSetup(ueSpeed,
                  simDuration,
                  intersiteDistance,
                  enbDowntilt,
                  seed,
                  runId,
                  trialName,
                  tcpVariant,
                  uavMobility,
                  topology,
                  startHeight,
                  endHeight,
                  helixRadius,
                  bbrWindowLength,
                  g_addStaticUes,
                  g_fullBufferInterference,
                  g_aerialUeRatio,
                  g_logging,
                  rlMode,
                  handoverAlgorithm,
                  stepTime,
                  delay,
                  handoverPenalty,
                  rlReferenceRate,
                  rlRttPenaltyWeight,
                  rlMinRttMs,
                  tcpFailurePenalty,
                  rlfPenalty,
                  handoverMargin);

    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Stop(Seconds(simDuration));

    Simulator::Run();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulation time: " << elapsed.count() << " seconds" << std::endl;
    std::cout << "Total handovers: " << g_totalHandovers << std::endl;
    std::cout << "Total received: " << (g_totalRxBytes * 8 / 1000000.0) << " Mbit" << std::endl;
    std::cout << "Total retransmissions: " << g_totalRetransmissions << std::endl;
    double avgRttMs = (g_rttSamples > 0) ? (g_rttSumMs / g_rttSamples) : 0.0;
    std::cout << "Average RTT: " << avgRttMs << " ms" << std::endl;
    if (rlMode)
    {
        OpenGymMultiAgentInterface::Get()->NotifySimulationEnd(0, {});
    }
    if (g_logging)
    {
        FlowMonitorHelper flowmonHelper;
        flowmonHelper.Install(g_uavContainer);
        flowmonHelper.Install(g_remoteHostContainer);
        flowmonHelper.SerializeToXmlFile(
            "contrib/defiance/examples/harl-tcp-prototype/output/harl-tcp.flowmonitor",
            true,
            true);
    }

    Simulator::Destroy();
    return 0;
}
