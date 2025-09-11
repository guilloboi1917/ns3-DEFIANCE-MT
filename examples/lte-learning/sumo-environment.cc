#include "sumo-environment.h"

#include "topology-creator.h"

#include <ns3/constant-position-mobility-model.h>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("SumoMobComEnv");

void
SumoMobComEnv::CreateTopology()
{
    NS_LOG_FUNCTION(this);

    NodeContainer bsNodes;
    bsNodes.Create(1);
    auto bsPosition = CreateObject<ConstantPositionMobilityModel>();

    bsPosition->SetPosition(Vector(100, 320, 31));
    bsNodes.Get(0)->AggregateObject(bsPosition);
    m_deviceManager->SetBsNodes(bsNodes);
    TopologyCreator scenario;
    std::string scenarioName = scenario.SetScenario("Babelsberg");
    // m_deviceManager->SetBsNodes(scenario.GetBaseStations(scenarioName));
    m_deviceManager->SetUtNodes(scenario.CreateUserEquipment(2, 42, scenarioName));
}
} // namespace ns3
