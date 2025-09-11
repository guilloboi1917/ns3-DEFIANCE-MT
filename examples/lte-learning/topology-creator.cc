#include "topology-creator.h"

#include <ns3/mobility-module.h>

namespace py = pybind11;

namespace ns3
{
// Run this in console: export NS_LOG=TopologyCreator=info
NS_LOG_COMPONENT_DEFINE("TopologyCreator");
py::scoped_interpreter guard{};
py::object scenarioHelper;
py::object locationFinder;
std::string pathToNs3;

TopologyCreator::TopologyCreator()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("TopologyCreator created");
    defaultScenario = {
        {"Babelsberg", std::make_tuple(52.393610, 13.128111, 2.0, 2.0)},                 // 29 BS
        {"Babelsmall", std::make_tuple(52.393610, 13.128111, 0.5, 0.5)},                 // 21 BS
        {"Berlin", std::make_tuple(52.54603100769271, 13.49809958325796, 2.0, 2.0)},     // 14 BS
        {"Berlinbig", std::make_tuple(52.54603100769271, 13.49809958325796, 3.0, 3.0)}}; // 32 BS
    try
    {
        scenarioHelper = py::module_::import("defiance.utils.scenario_helper");
        NS_LOG_INFO("Imported module scenario_helper.");
    }
    catch (const py::error_already_set& e)
    {
        NS_LOG_ERROR(e.what());
        NS_ABORT_MSG("Failed to import module scenario_helper.");
    }
    try
    {
        locationFinder = py::module_::import("defiance.utils.find_location");
        NS_LOG_INFO("Imported module find_location.");
    }
    catch (const py::error_already_set& e)
    {
        NS_LOG_ERROR(e.what());
        NS_ABORT_MSG("Failed to import module find_location.");
    }
    pathToNs3 = std::getenv("NS3_HOME");
}

TopologyCreator::~TopologyCreator()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("TopologyCreator destroyed");
}

std::string
TopologyCreator::GetDirectoryName(float centerX, float centerY, float sizeX, float sizeY)
{
    py::object getDirectoryName = scenarioHelper.attr("get_directory_name");
    py::object scenarioNameObject =
        getDirectoryName(py::make_tuple(centerX, centerY), py::make_tuple(sizeX, sizeY));
    std::string scenarioName = py::str(scenarioNameObject);
    return scenarioName;
}

std::string
TopologyCreator::SetScenario(float centerX, float centerY, float sizeX, float sizeY)
{
    std::string scenarioName = GetDirectoryName(centerX, centerY, sizeX, sizeY);
    NS_LOG_INFO("Attempting to set scenario: " << scenarioName);

    py::object create_config_files = scenarioHelper.attr("create_config_files");
    defaultScenario[scenarioName] = std::make_tuple(centerX, centerY, sizeX, sizeY);
    py::object result =
        create_config_files(py::make_tuple(centerX, centerY), py::make_tuple(sizeX, sizeY));
    std::string str = py::str(result);

    if (scenarioName != str)
    {
        NS_ABORT_MSG("Scenario name mismatch. Expected: " + scenarioName + ", got: " + str);
    }

    NodeContainer bs = GetBaseStations(centerX, centerY, sizeX, sizeY);
    return scenarioName;
}

std::string
TopologyCreator::SetScenario(std::string scenario)
{
    NS_LOG_INFO("Attempting to set scenario: " << scenario);
    if (defaultScenario.find(scenario) != defaultScenario.end())
    {
        auto [centerX, centerY, sizeX, sizeY] = defaultScenario[scenario];
        NS_LOG_INFO("Using default scenario " << scenario << " with position (" << centerX << ", "
                                              << centerY << ") and size " << sizeX << "x" << sizeY
                                              << " km.");
        return SetScenario(centerX, centerY, sizeX, sizeY);
    }
    NS_ABORT_MSG("Scenario " + scenario +
                 "neither found in default scenarios nor already generated scenarios.");
}

std::string
TopologyCreator::SetScenario(uint32_t numBs,
                             uint32_t seed,
                             uint32_t minDensity,
                             uint32_t maxDensity)
{
    py::object find_location = locationFinder.attr("find_loc");
    py::int_ pynumBs(numBs);
    py::int_ pyseed(seed);
    py::int_ pyminDensity(minDensity);
    py::int_ pymaxDensity(maxDensity);

    py::tuple result = find_location(pynumBs, pyseed, pyminDensity, pymaxDensity);
    auto centerX = result[0].cast<float>();
    auto centerY = result[1].cast<float>();
    auto sizeX = result[3].cast<float>();
    auto sizeY = result[2].cast<float>();
    auto density = result[4].cast<float>();
    NS_LOG_INFO("Density: " << density);
    return SetScenario(centerX, centerY, sizeX, sizeY);
}

NodeContainer
TopologyCreator::GetBaseStations(float centerX, float centerY, float sizeX, float sizeY)
{
    std::string scenarioName = GetDirectoryName(centerX, centerY, sizeX, sizeY);

    py::object getBaseStations = scenarioHelper.attr("add_base_stations");
    py::object numBaseStations = getBaseStations(scenarioName);
    int bs = numBaseStations.cast<int>();
    auto numBs = (uint32_t)bs;
    NS_LOG_INFO("Adding " << numBs << " base stations to scenario " << scenarioName);
    std::flush(std::cout);

    NodeContainer baseStations;
    baseStations.Create(numBs);
    std::string traceDir =
        pathToNs3 + "/contrib/defiance/scenarios/" + scenarioName + "/traces/base_stations.tcl";
    Ns2MobilityHelper ns2Stations = Ns2MobilityHelper(traceDir);
    ns2Stations.Install(baseStations.Begin(), baseStations.End());
    return baseStations;
}

NodeContainer
TopologyCreator::GetBaseStations(std::string location)
{
    if (location == "")
    {
        NS_ABORT_MSG("No scenario set. Can't find base stations. Please set a scenario first.");
    }
    if (defaultScenario.find(location) != defaultScenario.end())
    {
        auto [centerX, centerY, sizeX, sizeY] = defaultScenario[location];
        return GetBaseStations(centerX, centerY, sizeX, sizeY);
    }
    else
    {
        NS_ABORT_MSG(
            "Location neither found in default scenarios nor already generated scenarios.");
    }
}

NodeContainer
TopologyCreator::CreateUserEquipment(uint32_t numUe, uint32_t seed, std::string location)
{
    if (location == "")
    {
        NS_ABORT_MSG("No scenario set. Can't generate UE traces. Please set a scenario first.");
    }
    if (defaultScenario.find(location) != defaultScenario.end())
    {
        auto [centerX, centerY, sizeX, sizeY] = defaultScenario[location];
        std::string scenarioName = GetDirectoryName(centerX, centerY, sizeX, sizeY);

        py::object randomRoutes = scenarioHelper.attr("generate_random_routes");
        randomRoutes(py::int_(numUe),
                     py::make_tuple(centerX, centerY),
                     py::make_tuple(sizeX, sizeY),
                     py::str(scenarioName),
                     py::int_(seed),
                     py::bool_(false));

        /*py::object generateTraces = scenarioHelper.attr("generate_traces");
        generateTraces(scenarioName, py::int_(seed));*/

        std::string command = "scenario-helper";
        command += " -o " + scenarioName;
        command += " -s " + std::to_string(seed);
        if (system(command.c_str()) != 0)
        {
            NS_ABORT_MSG("Error generating UE traces.");
        }

        NodeContainer userEquipment;
        userEquipment.Create(numUe);
        std::string traceDir =
            pathToNs3 + "/contrib/defiance/scenarios/" + scenarioName + "/traces/traffic.tcl";
        Ns2MobilityHelper ns2Mobility = Ns2MobilityHelper(traceDir);
        ns2Mobility.Install(userEquipment.Begin(), userEquipment.End());
        return userEquipment;
    }
    else
    {
        NS_ABORT_MSG(
            "Location neither found in default scenarios nor already generated scenarios.");
    }
}
} // namespace ns3
