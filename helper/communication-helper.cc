#include "communication-helper.h"

#include "ns3/simple-channel-interface.h"

namespace ns3
{

TypeId
CommunicationHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::CommunicationHelper")
                            .SetParent<Object>()
                            .SetGroupName("defiance")
                            .AddConstructor<CommunicationHelper>();
    return tid;
}

CommunicationHelper::CommunicationHelper(bool checkForSimpleInterfaces)
    : m_checkForSimpleInterfaces(checkForSimpleInterfaces)
{
}

void
CommunicationHelper::SetObservationApps(RlApplicationContainer observationApps)
{
    m_observationApps = observationApps;
}

void
CommunicationHelper::SetRewardApps(RlApplicationContainer rewardApps)
{
    m_rewardApps = rewardApps;
}

void
CommunicationHelper::SetAgentApps(RlApplicationContainer agentApps)
{
    m_agentApps = agentApps;
}

void
CommunicationHelper::SetActionApps(RlApplicationContainer actionApps)
{
    m_actionApps = actionApps;
}

Ptr<RlApplication>
CommunicationHelper::GetApp(RlApplicationId id)
{
    switch (id.applicationType)
    {
    case OBSERVATION:
        return m_observationApps.Get(id.applicationId);
    case REWARD:
        return m_rewardApps.Get(id.applicationId);
    case AGENT:
        return m_agentApps.Get(id.applicationId);
    case ACTION:
        return m_actionApps.Get(id.applicationId);
    default:
        NS_ABORT_MSG("Unsupported applicationType" << id.applicationType);
    }
}

std::pair<Ptr<ChannelInterface>, Ptr<ChannelInterface>>
CommunicationHelper::CreateChannelInterfaces(Ptr<RlApplication> clientApp,
                                             Ptr<RlApplication> serverApp,
                                             const CommunicationAttributes& attributes) const
{
    auto socketAttributes = dynamic_cast<const SocketCommunicationAttributes*>(&attributes);
    if (!socketAttributes ||
        (m_checkForSimpleInterfaces && clientApp->GetNode() == serverApp->GetNode()))
    {
        auto clientInterface = CreateObject<SimpleChannelInterface>();
        clientInterface->SetPropagationDelay(attributes.delay);
        auto serverInterface = CreateObject<SimpleChannelInterface>();
        serverInterface->SetPropagationDelay(attributes.delay);
        clientInterface->Connect(serverInterface);
        return {clientInterface, serverInterface};
    }
    return {CreateObject<SocketChannelInterface>(clientApp->GetNode(),
                                                 socketAttributes->clientAddress,
                                                 socketAttributes->protocol),
            CreateObject<SocketChannelInterface>(serverApp->GetNode(),
                                                 socketAttributes->serverAddress,
                                                 socketAttributes->protocol)};
}

void
Connect(const Ptr<ChannelInterface> a, const Ptr<ChannelInterface> b)
{
    a->Connect(b);
}

std::pair<Ptr<ChannelInterface>, Ptr<ChannelInterface>>
CommunicationHelper::Connect(Ptr<RlApplication> clientApp,
                             Ptr<RlApplication> serverApp,
                             const CommunicationAttributes& attributes) const
{
    const auto& [clientInterface, serverInterface] =
        CreateChannelInterfaces(clientApp, serverApp, attributes);
    Simulator::Schedule(m_connectTime,
                        MakeCallback(&ns3::Connect).Bind(clientInterface, serverInterface));
    return {clientInterface, serverInterface};
}

void
CommunicationHelper::AddCommunication(const std::vector<CommunicationPair>& communicationPairs)
{
    for (const auto& communicationPair : communicationPairs)
    {
        auto serverApp = GetApp(communicationPair.serverId);
        auto clientApp = GetApp(communicationPair.clientId);

        const auto& [clientInterface, serverInterface] =
            Connect(clientApp, serverApp, communicationPair.attributes);
        clientApp->AddInterface(communicationPair.serverId, clientInterface);
        serverApp->AddInterface(communicationPair.clientId, serverInterface);
    }
}

void
CommunicationHelper::DeleteCommunication(RlApplicationId localAppId,
                                         RlApplicationId remoteAppId,
                                         uint localInterfaceId,
                                         uint remoteInterfaceId)
{
    auto localApp = GetApp(localAppId);
    auto remoteApp = GetApp(remoteAppId);
    localApp->DeleteInterface(remoteAppId, localInterfaceId);
    remoteApp->DeleteInterface(localAppId, remoteInterfaceId);
}

void
CommunicationHelper::SetIds()
{
    for (uint i = 0; i < m_observationApps.GetN(); i++)
    {
        m_observationApps.Get(i)->SetId(RlApplicationId{OBSERVATION, i});
    }
    for (uint i = 0; i < m_rewardApps.GetN(); i++)
    {
        m_rewardApps.Get(i)->SetId(RlApplicationId{REWARD, i});
    }
    for (uint i = 0; i < m_agentApps.GetN(); i++)
    {
        m_agentApps.Get(i)->SetId(RlApplicationId{AGENT, i});
    }
    for (uint i = 0; i < m_actionApps.GetN(); i++)
    {
        m_actionApps.Get(i)->SetId(RlApplicationId{ACTION, i});
    }
}

void
CommunicationHelper::Configure()
{
    for (size_t i = 0; i < m_observationApps.GetN(); i++)
    {
        m_observationApps.Get(i)->Setup();
    }
    for (size_t i = 0; i < m_rewardApps.GetN(); i++)
    {
        m_rewardApps.Get(i)->Setup();
    }
    for (size_t i = 0; i < m_agentApps.GetN(); i++)
    {
        m_agentApps.Get(i)->Setup();
    }
    for (size_t i = 0; i < m_actionApps.GetN(); i++)
    {
        m_actionApps.Get(i)->Setup();
    }
}

} // namespace ns3
