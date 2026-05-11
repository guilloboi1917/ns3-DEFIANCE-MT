#include "uav-handover-scenario-setup.cc"

#include "ns3/applications-module.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rl-application-helper.h"

#include <cstdint>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavHandoverScenario");

std::string handoverAlgorithm = "agent";
uint16_t numberOfUes = 3;
uint16_t numberOfEnbs = 4;
double distance = 1000.0; // m
double speed = 20.0;      // m/s
double simTime = 50.0;    // s
uint32_t stepTime = 420;  // ms
uint32_t delay = 0;       // ms
double enbTxPowerDbm = 10.0;
double handoverPenalty = 1000.0;
double ueHeight = 10.0;
double enbHeight = 25.0;
std::string propagationModel = "ThreeGppUma";
std::string trialName = "1";
bool visualize = false;
int parallel = 0;
uint32_t seed = 1;
uint32_t runId = 1;
std::string statsDir = "";
NetDeviceContainer g_ueLteDevs;
NetDeviceContainer g_enbLteDevs;
uint32_t g_totalHandovers = 0;
uint32_t g_noopHandovers = 0;
uint32_t g_sameCellHandovers = 0;

// Variables for saving stats
std::string pathToNs3 = std::getenv("NS3_HOME");

/**
 * UAV handover playground to test an agent's ability to perform handovers
 * using TCP BBR as transport protocol.
 *
 * Some eNBs are created in a grid, while UEs move in the area between them.
 * UE 0 is controlled by the agent while the rest of the UEs will just
 * reconnect after a radio link failure.
 *
 * Key differences from the base handover scenario:
 * - Uses TCP with BBR congestion control instead of UDP
 * - CellId action space is discrete (0 = no-op, 1..numBs = target cell)
 * - SINR per cell is added as an observation
 * - Current TCP congestion window (cwnd) is included as an observation
 * - Reward includes a configurable penalty for handovers
 * - Logs cwnd and throughput to CSV for visualization
 *
 * Start training on this example with the following command:
 * run-agent train -n defiance-uav-handover
 * Start inference of the trained agent with the following command:
 * run-agent infer -n defiance-uav-handover -a /path/to/PPO-run
 * When in a devcontainer, the path would usually be /root/ray_results/PPO-run
 * Specify the beneath cmd line arguments like this:
 * --ns3-settings numberOfUes=4 seed=2
 */
int
main(int argc, char* argv[])
{
    // Set default TCP congestion control to BBR before any Internet stack installation
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpBbr::GetTypeId()));

    // some default attributes reasonable for this scenario, but before processing command line
    // arguments, so that user can override
    Config::SetDefault("ns3::BulkSendApplication::SendSize", UintegerValue(655));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(655));
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));

    CommandLine cmd(__FILE__);
    cmd.AddValue("handoverAlgorithm",
                 "Handover algorithm to use (agent, a2a4, a3)",
                 handoverAlgorithm);
    cmd.AddValue("numberOfUes", "Number of UEs in the simulation", numberOfUes);
    cmd.AddValue("numberOfEnbs", "Number of eNBs in the simulation", numberOfEnbs);
    cmd.AddValue("distance", "Meters between eNBs (default = 1000)", distance);
    cmd.AddValue("speed", "Meters/second of the UEs (default = 20)", speed);
    cmd.AddValue("simTime", "Total duration of the simulation in seconds (default = 100)", simTime);
    cmd.AddValue("stepTime",
                 "Milliseconds between each step in the simulation (default = 420)",
                 stepTime);
    cmd.AddValue("delay", "Transmission delay of Simple Channel Interfaces in ms", delay);
    cmd.AddValue("enbTxPowerDbm", "TX power [dBm] used by HeNBs (default = 10.0)", enbTxPowerDbm);
    cmd.AddValue("handoverPenalty",
                 "Reward penalty applied when a handover occurs (default = 1000)",
                 handoverPenalty);
    cmd.AddValue("ueHeight",
                 "Height of the UEs in meters (default = 10.0)",
                 ueHeight);
    cmd.AddValue("enbHeight",
                 "Height of the eNBs in meters (default = 25.0)",
                 enbHeight);
    cmd.AddValue("propagationModel",
                 "Propagation model: LogDistance (fast) or ThreeGppUma (accurate)",
                 propagationModel);
    cmd.AddValue("seed", "Seed for random number generator", seed);
    cmd.AddValue("runId",
                 "Counts how often the environment has been reset (used for seeding)",
                 runId);
    cmd.AddValue("parallel", "Number of parallel simulation runs", parallel);
    cmd.AddValue("trial_name", "Trial name", trialName);
    cmd.AddValue("visualize", "Log visualization data", visualize);
    cmd.AddValue("statsDir", "Directory for CSV stats output", statsDir);

    cmd.Parse(argc, argv);

    if (handoverAlgorithm == "agent")
    {
        Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));
    }
    seed += parallel;

    OpenGymMultiAgentInterface::Get();
    Ns3AiMsgInterface::Get()->SetTrialName(trialName);
    std::cout << "Trial name: " << trialName << "\t";
    std::cout << "Seed: " << seed << "\tRun ID: " << runId << std::endl;

    scenarioSetup(numberOfUes,
                  numberOfEnbs,
                  1,
                  distance,
                  speed,
                  simTime,
                  stepTime,
                  delay,
                  enbTxPowerDbm,
                  seed,
                  runId,
                  visualize,
                  trialName,
                  handoverAlgorithm,
                  handoverPenalty,
                  ueHeight,
                  enbHeight,
                  propagationModel,
                  statsDir);

    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Simulating " << simTime << "s took " << duration.count()
              << "s. Handovers: " << g_totalHandovers << "\tNoops: " << g_noopHandovers
              << "\tSame cell: " << g_sameCellHandovers << std::endl;
    OpenGymMultiAgentInterface::Get()->NotifySimulationEnd(0, {});
    Simulator::Destroy();
    return 0;
}
