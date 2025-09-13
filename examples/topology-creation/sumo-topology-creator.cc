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

// Run this in console: export NS_LOG=SumoImport=info
NS_LOG_COMPONENT_DEFINE("SumoImport");

std::string pathToNs3 = std::getenv("NS3_HOME");
std::string tcDir = "/contrib/defiance/examples/topology-creation";
std::string buildDir = pathToNs3 + "/contrib/defiance/scenarios/examples/topology-creation";

/**
 * Creates human readable files for base stations and user equipment
 */
void
createOutputFiles(NodeContainer& nodes, std::string type)
{
    // Create output json file for type
    std::ofstream outputFile((buildDir + "/" + type + ".json").c_str());
    if (!outputFile.is_open())
    {
        std::cerr << "Error opening " << type << " output file for writing" << std::endl;
        return;
    }

    outputFile << "{" << std::endl;
    outputFile << "  \"nodes\": [" << std::endl;

    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        auto node = nodes.Get(i);
        auto mobility = node->GetObject<MobilityModel>();

        outputFile << "    {" << std::endl;
        outputFile << "      \"id\": " << i << "," << std::endl;

        if (mobility != nullptr)
        {
            Vector pos = mobility->GetPosition();
            outputFile << "      \"position\": {\"x\": " << pos.x << ", \"y\": " << pos.y
                       << ", \"z\": " << pos.z << "}" << std::endl;
        }
        else
        {
            std::cerr << "Error: Mobility model not found for node " << i << std::endl;
            std::cout << "Continuing to next node. This might lead to unexpected behaviour later."
                      << std::endl;
            outputFile << "      \"position\": {\"x\": 0, \"y\": 0, \"z\": 0}" << std::endl;
        }
        outputFile << "    }";

        if (i != nodes.GetN() - 1)
        {
            outputFile << "," << std::endl;
        }
        else
        {
            outputFile << std::endl;
        }
    }

    outputFile << "  ]" << std::endl;
    outputFile << "}" << std::endl;

    outputFile.close();
}

int
main(int argc, char* argv[])
{
    std::string stationTrace;
    std::string mobilityTrace;
    uint32_t numUes = 50;
    uint32_t numBs = 14;
    bool output = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("num-ues", "Number of User Equipment to be generated", numUes);
    cmd.AddValue("output", "Generate output files (default = true)", output);

    cmd.Parse(argc, argv);

    std::string createNetworkCommand = "python " + pathToNs3 + tcDir + "/create_tracefiles.py";
    createNetworkCommand += " -p " + std::to_string(numUes);
    if (system(createNetworkCommand.c_str()))
    {
        NS_LOG_ERROR("Error creating network files");
        std::cerr << "Error creating network files" << std::endl;
        return 1;
    }

    // test whether config files directory was created
    if (!fs::exists(buildDir + "/config_files"))
    {
        std::cerr << "Could not find network files" << std::endl;
        return 1;
    }

    stationTrace = buildDir + "/traces/base_stations.tcl";
    mobilityTrace = buildDir + "/traces/modified_traffic.tcl";

    NodeContainer nodes;
    nodes.Create(numUes + numBs);

    auto ns2Stations = Ns2MobilityHelper(stationTrace);
    auto ns2Mobility = Ns2MobilityHelper(mobilityTrace);
    ns2Stations.Install();
    ns2Mobility.Install();

    if (output)
    {
        createOutputFiles(nodes, "nodes");
    }

    return 0;
}
