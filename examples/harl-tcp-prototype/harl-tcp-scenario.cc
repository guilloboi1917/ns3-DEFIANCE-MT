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
std::string uavMobility = "helix";
std::string topology = "simple";
double startHeight = 150.0; // m
double endHeight = 300.0;   // m
double helixRadius = 50.0;  // m
uint32_t bbrWindowLength = 10;
uint32_t g_addStaticUes = 0;
bool g_fullBufferInterference = false;
double g_aerialUeRatio = 0.0;
bool g_logging = true;

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
    cmd.AddValue(
        "aerialUeRatio",
        "Fraction of ground UEs placed at aerial height (0=all ground, 1=all aerial)",
        g_aerialUeRatio);
    cmd.AddValue(
        "logging",
        "Enable CSV file logging (disable for RL training)",
        g_logging);
    cmd.Parse(argc, argv);

    // OpenGymMultiAgentInterface::Get(); // This is needed to initialize the interface and set the
    // trial name, but it needs to be called after scenarioSetup since that function also sets the
    // trial name and we don't want to overwrite it
    // Ns3AiMsgInterface::Get()->SetTrialName(trialName);

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
                  g_logging);

    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Stop(Seconds(simDuration));
    Simulator::Run();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulation time: " << elapsed.count() << " seconds" << std::endl;
    std::cout << "Total handovers: " << g_totalHandovers << std::endl;
    // OpenGymMultiAgentInterface::Get()->NotifySimulationEnd(0, {}); // Notify the interface that
    // the simulation has ended so it can perform any necessary cleanup
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
