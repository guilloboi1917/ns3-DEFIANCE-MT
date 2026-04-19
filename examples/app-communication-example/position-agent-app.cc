#include "position-agent-app.h"

#include "ns3/agent-application.h"
#include "ns3/base-test.h"

#include <string>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("PositionAgentApp");

TypeId
PositionAgentApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PositionAgentApp")
                            .SetParent<AgentApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PositionAgentApp>();
    return tid;
}

void
PositionAgentApp::OnRecvObs(uint remoteAppId)
{
    NS_LOG_INFO("Received observation from observation interface " << remoteAppId << ":");
    auto lastPosition = m_obsDataStruct.GetNewestByID(remoteAppId)
                            ->data->Get("position")
                            ->GetObject<OpenGymBoxContainer<float>>();
    NS_LOG_INFO("\t Last position: " << lastPosition->GetValue(0) << ":"
                                     << lastPosition->GetValue(1));
    // Note that the 10 last observations are stored per default.
    // To change this, configure the attribute MaxObservationHistoryLength of the agent application!
    NS_LOG_INFO("\t Minimum of coordinates of the 10 last positions: "
                << m_obsDataStruct.AggregateNewest(remoteAppId, 10)["position"].GetMin());
    NS_LOG_INFO("\t Maximum of coordinates of the 10 last positions: "
                << m_obsDataStruct.AggregateNewest(remoteAppId, 10)["position"].GetMax());
    NS_LOG_INFO("\t Last velocity: " << m_obsDataStruct.GetNewestByID(remoteAppId)
                                            ->data->Get("velocity")
                                            ->GetObject<OpenGymBoxContainer<double>>()
                                            ->GetValue(0));
    NS_LOG_INFO("\t Average of 5 last velocities: "
                << m_obsDataStruct.AggregateNewest(remoteAppId, 5)["velocity"].GetAvg());

    // Send last positions of both observation apps to action app
    auto dictContainer = CreateObject<OpenGymDictContainer>();
    for (int observationAppId = 0; observationAppId < 2; observationAppId++)
    {
        Ptr<OpenGymBoxContainer<float>> positionBox;
        if (m_obsDataStruct.HistoryExists(observationAppId))
        {
            positionBox = m_obsDataStruct.GetNewestByID(observationAppId)
                              ->data->Get("position")
                              ->GetObject<OpenGymBoxContainer<float>>();
        }
        else
        {
            positionBox = MakeBoxContainer<float>(2, 0, 0);
        }
        dictContainer->Add("position-" + std::to_string(observationAppId), positionBox);
    }
    SendAction(dictContainer);
}

void
PositionAgentApp::OnRecvReward(uint remoteAppId)
{
    NS_LOG_INFO("Received reward from reward interface " << remoteAppId << ":");
    NS_LOG_INFO("\t Last distance: "
                << m_rewardDataStruct.AggregateNewest(remoteAppId)["distance"].GetAvg());
}

Ptr<OpenGymSpace>
PositionAgentApp::GetObservationSpace()
{
    return {};
}

Ptr<OpenGymSpace>
PositionAgentApp::GetActionSpace()
{
    return {};
}

NS_OBJECT_ENSURE_REGISTERED(PositionAgentApp);
} // namespace ns3
