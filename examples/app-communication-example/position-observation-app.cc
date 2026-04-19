#include "position-observation-app.h"

#include "ns3/base-test.h"
#include "ns3/mobility-model.h"

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("PositionObservationApp");


TypeId
PositionObservationApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PositionObservationApp")
                            .SetParent<ObservationApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PositionObservationApp>();
    return tid;
}

Ptr<OpenGymDictContainer>
PositionObservationApp::CreateDictContainer(Vector position, double velocity)
{
    auto dictContainer = CreateObject<OpenGymDictContainer>();
    dictContainer->Add("position", MakeBoxContainer<float>(2, position.x, position.y));
    dictContainer->Add("velocity", MakeBoxContainer<double>(1, velocity));
    return dictContainer;
}

void
PositionObservationApp::Observe(Ptr<const MobilityModel> observation)
{
    if (IsRunning()) {
        auto position = observation->GetPosition();
        auto totalVelocity = observation->GetVelocity().GetLength();
        Send(CreateDictContainer(position, totalVelocity), 0);
        NS_LOG_INFO("Observation app " << GetId().applicationId << " sent to agent app:");
        NS_LOG_INFO("\tPosition: " << position);
        NS_LOG_INFO("\tVelocity: " << totalVelocity);
    }
}

void
PositionObservationApp::RegisterCallbacks()
{
    NS_LOG_FUNCTION(this);
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(GetNode()->GetId()) + "/$ns3::MobilityModel/CourseChange",
                                  MakeCallback(&PositionObservationApp::Observe, this));
}

NS_OBJECT_ENSURE_REGISTERED(PositionObservationApp);
}