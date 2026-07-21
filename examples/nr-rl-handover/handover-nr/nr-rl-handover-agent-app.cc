#include "nr-rl-handover-agent-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-helper.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlHandoverAgentApp");

NrRlHandoverAgentApp::NrRlHandoverAgentApp()
    : AgentApplication()
{
    m_reward = 0;
}

NrRlHandoverAgentApp::~NrRlHandoverAgentApp()
{
}

TypeId
NrRlHandoverAgentApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlHandoverAgentApp")
            .SetParent<AgentApplication>()
            .SetGroupName("defiance")
            .AddConstructor<NrRlHandoverAgentApp>()
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrRlHandoverAgentApp::m_numBs),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

void
NrRlHandoverAgentApp::Setup()
{
    AgentApplication::Setup();
    m_observation = GetResetObservation();
    m_reward = GetResetReward();
    m_lastInferredActionTime = Seconds(0);
    NS_LOG_INFO("NrRlHandoverAgentApp setup complete");
}

void
NrRlHandoverAgentApp::OnRecvObs(uint id)
{
    NS_LOG_FUNCTION(this << id);
    m_observation = m_obsDataStruct.GetNewestByID(id)->data;

    // Observations already contain all metrics (rsrp, cwnd, bbr, etc.)
    // from the observation app — no need to augment here.

    // Inference is throttled by measurement report frequency (~120-480ms).
    NS_LOG_INFO("Inferring handover action at t=" << Simulator::Now().GetSeconds() << "s");
    InferAction();
    m_lastInferredActionTime = Simulator::Now();
}

void
NrRlHandoverAgentApp::OnRecvReward(uint id)
{
    NS_LOG_FUNCTION(this << id);
    auto reward = m_rewardDataStruct.GetNewestByID(id)->data;
    auto rewardValue =
        DynamicCast<OpenGymBoxContainer<double>>(reward->Get("reward"))->GetValue(0);
    m_reward = rewardValue;
    NS_LOG_INFO("Received reward: " << m_reward);
}

void
NrRlHandoverAgentApp::InitiateAction(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    // Package discrete action as dict for the action app
    auto dictAction = CreateObject<OpenGymDictContainer>();
    dictAction->Add("newCellId", action);
    SendAction(dictAction);
}

Ptr<OpenGymSpace>
NrRlHandoverAgentApp::GetObservationSpace()
{
    auto dictSpace = CreateObject<OpenGymDictSpace>();

    // --- Per-cell measurements ---
    // RSRP per BS in dBm (-140 = unknown, typical range [-44, -140], but real values can go lower)
    auto rsrpSpace = CreateObject<OpenGymBoxSpace>(-160.0,
                                                    -40.0,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<double>());
    // RSRQ per BS in dB (-20 = unknown, typical range [-3, -20], but real values can go much lower)
    auto rsrqSpace = CreateObject<OpenGymBoxSpace>(-100.0,
                                                    -3.0,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<double>());

    // // --- PHY metrics ---
    // auto tbsSpace = CreateObject<OpenGymBoxSpace>(0,
    //                                                100000,
    //                                                std::vector<uint32_t>{1},
    //                                                TypeNameGet<int32_t>());

    // --- Current cell UL SINR (scalar, only meaningful for serving cell) ---
    auto sinrSpace = CreateObject<OpenGymBoxSpace>(-40,
                                                    50,
                                                    std::vector<uint32_t>{1},
                                                    TypeNameGet<double>());

    // --- Deltas (change since last observation) ---
    auto rsrpDeltaSpace = CreateObject<OpenGymBoxSpace>(-60.0,
                                                         60.0,
                                                         std::vector<uint32_t>{m_numBs},
                                                         TypeNameGet<double>());
    auto rsrqDeltaSpace = CreateObject<OpenGymBoxSpace>(-60.0,
                                                         60.0,
                                                         std::vector<uint32_t>{m_numBs},
                                                         TypeNameGet<double>());
    auto sinrDeltaSpace = CreateObject<OpenGymBoxSpace>(-20,
                                                         20,
                                                         std::vector<uint32_t>{1},
                                                         TypeNameGet<double>());

    // --- Add all to dict ---
    dictSpace->Add("rsrps", rsrpSpace);
    dictSpace->Add("rsrqs", rsrqSpace);
    dictSpace->Add("sinr", sinrSpace);
    dictSpace->Add("rsrpDelta", rsrpDeltaSpace);
    dictSpace->Add("rsrqDelta", rsrqDeltaSpace);
    dictSpace->Add("sinrDelta", sinrDeltaSpace);
    // dictSpace->Add("tbs", tbsSpace);

    // --- Action mask (0/1 per action: 0=no-op, 1..numBs=target cell) ---
    auto actionMaskSpace = CreateObject<OpenGymBoxSpace>(
        0.0,
        1.0,
        std::vector<uint32_t>{m_numBs + 1},
        TypeNameGet<double>());
    dictSpace->Add("action_mask", actionMaskSpace);

    return dictSpace;
}

Ptr<OpenGymSpace>
NrRlHandoverAgentApp::GetActionSpace()
{
    // Discrete action: 0 = no-op, 1..numBs = target cell ID
    return CreateObject<OpenGymDiscreteSpace>(m_numBs + 1);
}

Ptr<OpenGymDictContainer>
NrRlHandoverAgentApp::GetResetObservation() const
{
    auto obs = CreateObject<OpenGymDictContainer>();

    // Zero-initialized per-cell measurements
    auto rsrps = MakeBoxContainer<double>(m_numBs);
    auto rsrqs = MakeBoxContainer<double>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrps->AddValue(-140.0);   // -140 = not measured (dBm)
        rsrqs->AddValue(-20.0);    // -20 = not measured (dB)
    }

    // Current cell UL SINR (scalar)
    auto sinr = MakeBoxContainer<double>(1, -40.0);

    auto rrcState = CreateObject<OpenGymDiscreteContainer>();
    rrcState->SetValue(0);

    // PHY metrics
    auto tbs = MakeBoxContainer<int32_t>(1, 0);

    // Deltas (zero-initialized, reset resets history)
    auto rsrpDelta = MakeBoxContainer<double>(m_numBs);
    auto rsrqDelta = MakeBoxContainer<double>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrpDelta->AddValue(0.0);
        rsrqDelta->AddValue(0.0);
    }
    auto sinrDelta = MakeBoxContainer<double>(1, 0.0);

    obs->Add("rsrps", rsrps);
    obs->Add("rsrqs", rsrqs);
    obs->Add("sinr", sinr);
    obs->Add("rsrpDelta", rsrpDelta);
    obs->Add("rsrqDelta", rsrqDelta);
    obs->Add("sinrDelta", sinrDelta);
    // obs->Add("tbs", tbs);

    return obs;
}

float
NrRlHandoverAgentApp::GetResetReward()
{
    return 0.0f;
}

} // namespace ns3
