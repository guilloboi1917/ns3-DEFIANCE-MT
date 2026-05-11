#include "uav-handover-agent-application.h"

#include "ns3/base-test.h"
#include "ns3/lte-helper.h"

#include <cstdint>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavHandoverAgentApplication");

extern uint32_t g_currentCwnd;

UavHandoverAgentApplication::UavHandoverAgentApplication()
    : AgentApplication()
{
    m_reward = 0;
}

UavHandoverAgentApplication::~UavHandoverAgentApplication()
{
}

TypeId
UavHandoverAgentApplication::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UavHandoverAgentApplication")
            .SetParent<AgentApplication>()
            .SetGroupName("defiance")
            .AddConstructor<UavHandoverAgentApplication>()
            .AddAttribute("NumUes",
                          "Number of user equipments in the simulation.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&UavHandoverAgentApplication::m_numUes),
                          MakeUintegerChecker<uint>())
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&UavHandoverAgentApplication::m_numBs),
                          MakeUintegerChecker<uint>())
            .AddAttribute("StepTime",
                          "Time between each step in the simulation.",
                          UintegerValue(420),
                          MakeUintegerAccessor(&UavHandoverAgentApplication::m_stepTime),
                          MakeUintegerChecker<uint>())
            .AddAttribute("MaxCwnd",
                          "Maximum congestion window size for observation scaling.",
                          UintegerValue(65535),
                          MakeUintegerAccessor(&UavHandoverAgentApplication::m_maxCwnd),
                          MakeUintegerChecker<uint>());
    return tid;
}

void
UavHandoverAgentApplication::Setup()
{
    AgentApplication::Setup();

    m_observation = GetResetObservation();
    m_reward = GetResetReward();
}

void
UavHandoverAgentApplication::OnRecvObs(uint id)
{
    NS_LOG_FUNCTION(this << id);
    m_observation = m_obsDataStruct.GetNewestByID(id)->data;

    // Add the current TCP congestion window to the observation dict
    // (the eNB observation apps don't have access to UE-side cwnd)
    auto obsDict = DynamicCast<OpenGymDictContainer>(m_observation);
    if (obsDict)
    {
        auto cwndContainer = MakeBoxContainer<int32_t>(1, static_cast<int32_t>(g_currentCwnd));
        obsDict->Set("cwnd", cwndContainer);
    }

    if (Simulator::Now() - m_lastInferredActionTime >= MilliSeconds(m_stepTime))
    {
        NS_LOG_INFO("Infering action");
        NS_LOG_INFO("Observation: " << m_observation);
        NS_LOG_INFO("Reward: " << m_reward);
        InferAction();
        m_lastInferredActionTime = Simulator::Now();
    }
}

void
UavHandoverAgentApplication::OnRecvReward(uint id)
{
    NS_LOG_FUNCTION(this << id);
    auto reward = m_rewardDataStruct.GetNewestByID(id)->data;
    auto rewardValue = DynamicCast<OpenGymBoxContainer<double>>(reward->Get("reward"))->GetValue(0);
    m_reward = rewardValue;
}

void
UavHandoverAgentApplication::InitiateAction(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);
    auto dictAction = CreateObject<OpenGymDictContainer>();
    dictAction->Add("newCellId", action);
    SendAction(dictAction);
}

Ptr<OpenGymSpace>
UavHandoverAgentApplication::GetObservationSpace()
{
    auto dictSpace = CreateObject<OpenGymDictSpace>();

    // RSRP per BS
    auto rsrpsSpace = CreateObject<OpenGymBoxSpace>(-1,
                                                    97,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<int32_t>());
    // RSRQ per BS
    auto rsrqsSpace = CreateObject<OpenGymBoxSpace>(-1,
                                                    34,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<int32_t>());
    // SINR per BS (in dB)
    auto sinrsSpace = CreateObject<OpenGymBoxSpace>(0,
                                                    50,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<double>());
    // Current cell ID (discrete)
    auto cellIdSpace =
        CreateObject<OpenGymDiscreteSpace>(m_numBs + 1); // 0 = no-op, 1..numBs = cell IDs

    // Current congestion window size (single value)
    // MaxCwnd is typically 65535 for TCP; use int32_t (uint is not supported by gym)
    auto cwndSpace = CreateObject<OpenGymBoxSpace>(0,
                                                   m_maxCwnd,
                                                   std::vector<uint32_t>{1},
                                                   TypeNameGet<int32_t>());

    dictSpace->Add("rsrps", rsrpsSpace);
    dictSpace->Add("rsrqs", rsrqsSpace);
    dictSpace->Add("sinrs", sinrsSpace);
    dictSpace->Add("cellId", cellIdSpace);
    dictSpace->Add("cwnd", cwndSpace);
    return dictSpace;
}

Ptr<OpenGymSpace>
UavHandoverAgentApplication::GetActionSpace()
{
    // Discrete action space: 0 = no-op, 1..numBs = target cell ID
    Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace>(m_numBs + 1);
    return discrete;
}

Ptr<OpenGymDictContainer>
UavHandoverAgentApplication::GetResetObservation() const
{
    auto newObservation = CreateObject<OpenGymDictContainer>();

    auto rsrps = MakeBoxContainer<int32_t>(m_numBs);
    auto rsrqs = MakeBoxContainer<int32_t>(m_numBs);
    auto sinrs = MakeBoxContainer<double>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrps->AddValue(-1);
        rsrqs->AddValue(-1);
        sinrs->AddValue(0.0);
    }
    auto cellId = CreateObject<OpenGymDiscreteContainer>();
    cellId->SetValue(0);
    auto cwnd = MakeBoxContainer<int32_t>(1, int32_t(0));

    newObservation->Add("rsrps", rsrps);
    newObservation->Add("rsrqs", rsrqs);
    newObservation->Add("sinrs", sinrs);
    newObservation->Add("cellId", cellId);
    newObservation->Add("cwnd", cwnd);
    return newObservation;
}

float
UavHandoverAgentApplication::GetResetReward()
{
    return 0.0;
}
