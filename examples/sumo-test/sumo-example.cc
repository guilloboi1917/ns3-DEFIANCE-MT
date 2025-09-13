#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ns2-mobility-helper.h"

#include <cstdlib>
#include <filesystem>

using namespace ns3;

namespace fs = std::filesystem;

// Run this in console: export NS_LOG=SumoTest=info
NS_LOG_COMPONENT_DEFINE("SumoTest");

int
main(int argc, char* argv[])
{
    std::string stationTrace;
    std::string mobilityTrace;
    CommandLine cmd(__FILE__);

    cmd.Parse(argc, argv);

    std::string pathToNs3 = std::getenv("NS3_HOME");
    std::string stDir = "/contrib/defiance/examples/sumo-test";
    std::string buildDir = pathToNs3 + "/contrib/defiance/scenarios/examples/sumo-test";

    // create network from data
    std::string createNetworkCommand = "python " + pathToNs3 + stDir + "/create_network.py";
    if (system(createNetworkCommand.c_str()))
    {
        NS_LOG_ERROR("Error creating network");
        printf("Error creating network\n");
        return 1;
    }
    // test if directory was created
    if (!fs::exists(buildDir + "/config_files"))
    {
        printf("Could not find network files\n");
        return 1;
    }

    // generate traces from network
    std::string generateTraceCommand = "python " + pathToNs3 + stDir + "/generate_trace.py";
    if (system(generateTraceCommand.c_str()))
    {
        NS_LOG_ERROR("Error generating traces");
        printf("Error generating traces\n");
        return 1;
    }
    // test if traces were created
    if (!fs::exists(buildDir + "/traces"))
    {
        printf("Could not find trace files");
        return 1;
    }

    stationTrace = buildDir + "/traces/base_stations.tcl";
    mobilityTrace = buildDir + "/traces/traffic.tcl";

    Time::SetResolution(Time::NS);

    /*Ns2MobilityHelper ns2Stations = Ns2MobilityHelper(stationTrace);
    ns2Stations.Install();*/

    NodeContainer ueNodes;
    ueNodes.Create(200);

    Ns2MobilityHelper ns2Mobility = Ns2MobilityHelper(mobilityTrace);
    ns2Mobility.Install();

    Simulator::Stop(Seconds(1000));
    AnimationInterface anim(buildDir + "sumo-test-animation.xml");

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
