#include "harl-tcp-handover-agent-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-helper.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("HarlTcpHandoverAgentApp");

HarlTcpHandoverAgentApp::HarlTcpHandoverAgentApp()
    : AgentApplication()
{
    m_reward = 0;
}

HarlTcpHandoverAgentApp::~HarlTcpHandoverAgentApp()
{
}

TypeId
HarlTcpHandoverAgentApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::HarlTcpHandoverAgentApp")
            .SetParent<AgentApplication>()
            .SetGroupName("defiance")
            .AddConstructor<HarlTcpHandoverAgentApp>()
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&HarlTcpHandoverAgentApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NumUes",
                          "Number of user equipments in the simulation.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&HarlTcpHandoverAgentApp::m_numUes),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MaxCwnd",
                          "Maximum congestion window size (bytes) for observation scaling. "
                          "BBR uses byte-level cwnd = pacing_rate * RTT; ~1M at 50Mbps/100ms.",
                          UintegerValue(1000000),
                          MakeUintegerAccessor(&HarlTcpHandoverAgentApp::m_maxCwnd),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MaxRate",
                          "Maximum delivery rate (bps) for observation scaling.",
                          UintegerValue(100000000),
                          MakeUintegerAccessor(&HarlTcpHandoverAgentApp::m_maxRate),
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

void
HarlTcpHandoverAgentApp::Setup()
{
    AgentApplication::Setup();
    m_observation = GetResetObservation();
    m_reward = GetResetReward();
    m_lastInferredActionTime = Seconds(0);
    NS_LOG_INFO("HarlTcpHandoverAgentApp setup complete");
}

void
HarlTcpHandoverAgentApp::OnRecvObs(uint id)
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
HarlTcpHandoverAgentApp::OnRecvReward(uint id)
{
    NS_LOG_FUNCTION(this << id);
    auto reward = m_rewardDataStruct.GetNewestByID(id)->data;
    auto rewardValue =
        DynamicCast<OpenGymBoxContainer<double>>(reward->Get("reward"))->GetValue(0);
    m_reward = rewardValue;
    NS_LOG_INFO("Received reward: " << m_reward);
}

void
HarlTcpHandoverAgentApp::InitiateAction(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);

    // Package discrete action as dict for the action app
    auto dictAction = CreateObject<OpenGymDictContainer>();
    dictAction->Add("newCellId", action);
    SendAction(dictAction);
}

Ptr<OpenGymSpace>
HarlTcpHandoverAgentApp::GetObservationSpace()
{
    auto dictSpace = CreateObject<OpenGymDictSpace>();

    // --- Per-cell measurements ---
    // RSRP per BS (0-97 range in 3GPP mapping, -1 = unknown)
    auto rsrpSpace = CreateObject<OpenGymBoxSpace>(-1,
                                                    97,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<int32_t>());
    // RSRQ per BS (-1 = unknown, 0-34 in 3GPP mapping)
    auto rsrqSpace = CreateObject<OpenGymBoxSpace>(-1,
                                                    34,
                                                    std::vector<uint32_t>{m_numBs},
                                                    TypeNameGet<int32_t>());

    // --- Cell ID ---
    auto cellIdSpace = CreateObject<OpenGymDiscreteSpace>(m_numBs + 1);

    // --- RRC state ---
    auto rrcStateSpace = CreateObject<OpenGymDiscreteSpace>(14);

    // --- TCP metrics ---
    auto cwndSpace = CreateObject<OpenGymBoxSpace>(0,
                                                   m_maxCwnd,
                                                   std::vector<uint32_t>{1},
                                                   TypeNameGet<int32_t>());
    auto rttSpace = CreateObject<OpenGymBoxSpace>(0,
                                                  10000, // 10s max RTT
                                                  std::vector<uint32_t>{1},
                                                  TypeNameGet<int32_t>());

    // --- BBR metrics ---
    auto deliveryRateSpace = CreateObject<OpenGymBoxSpace>(0,
                                                           m_maxRate,
                                                           std::vector<uint32_t>{1},
                                                           TypeNameGet<int32_t>());
    // --- UAV position and velocity ---
    auto posSpace = CreateObject<OpenGymBoxSpace>(-5000.0,
                                                   5000.0,
                                                   std::vector<uint32_t>{3},
                                                   TypeNameGet<double>());
    auto velSpace = CreateObject<OpenGymBoxSpace>(-200.0,
                                                   200.0,
                                                   std::vector<uint32_t>{3},
                                                   TypeNameGet<double>());

    // --- PHY metrics ---
    auto mcsSpace = CreateObject<OpenGymBoxSpace>(0,
                                                   31,
                                                   std::vector<uint32_t>{1},
                                                   TypeNameGet<int32_t>());
    auto txPowerSpace = CreateObject<OpenGymBoxSpace>(-50.0,
                                                      50.0,
                                                      std::vector<uint32_t>{1},
                                                      TypeNameGet<double>());

    // --- Current cell UL SINR (scalar, only meaningful for serving cell) ---
    auto sinrSpace = CreateObject<OpenGymBoxSpace>(-40,
                                                    50,
                                                    std::vector<uint32_t>{1},
                                                    TypeNameGet<double>());

    // --- Add all to dict ---
    dictSpace->Add("rsrps", rsrpSpace);
    dictSpace->Add("rsrqs", rsrqSpace);
    dictSpace->Add("sinr", sinrSpace);
    dictSpace->Add("cellId", cellIdSpace);
    dictSpace->Add("rrcState", rrcStateSpace);
    dictSpace->Add("position", posSpace);
    dictSpace->Add("velocity", velSpace);
    dictSpace->Add("mcs", mcsSpace);
    dictSpace->Add("txPower", txPowerSpace);
    dictSpace->Add("cwnd", cwndSpace);
    dictSpace->Add("rtt", rttSpace);
    dictSpace->Add("deliveryRate", deliveryRateSpace);

    return dictSpace;
}

Ptr<OpenGymSpace>
HarlTcpHandoverAgentApp::GetActionSpace()
{
    // Discrete action: 0 = no-op, 1..numBs = target cell ID
    return CreateObject<OpenGymDiscreteSpace>(m_numBs + 1);
}

Ptr<OpenGymDictContainer>
HarlTcpHandoverAgentApp::GetResetObservation() const
{
    auto obs = CreateObject<OpenGymDictContainer>();

    // Zero-initialized per-cell measurements
    auto rsrps = MakeBoxContainer<int32_t>(m_numBs);
    auto rsrqs = MakeBoxContainer<int32_t>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrps->AddValue(-1);   // -1 = not measured
        rsrqs->AddValue(-1);   // -1 = not measured
    }

    // Current cell UL SINR (scalar)
    auto sinr = MakeBoxContainer<double>(1, -40.0);

    auto cellId = CreateObject<OpenGymDiscreteContainer>();
    cellId->SetValue(0);

    auto rrcState = CreateObject<OpenGymDiscreteContainer>();
    rrcState->SetValue(0);

    // UAV position and velocity
    auto pos = MakeBoxContainer<double>(3, 0.0, 0.0, 0.0);
    auto vel = MakeBoxContainer<double>(3, 0.0, 0.0, 0.0);

    // PHY metrics
    auto mcs = MakeBoxContainer<int32_t>(1, 0);
    auto txPower = MakeBoxContainer<double>(1, 0.0);

    auto cwnd = MakeBoxContainer<int32_t>(1, 0);
    auto rtt = MakeBoxContainer<int32_t>(1, 0);
    auto deliveryRate = MakeBoxContainer<int32_t>(1, 0);

    obs->Add("rsrps", rsrps);
    obs->Add("rsrqs", rsrqs);
    obs->Add("sinr", sinr);
    obs->Add("cellId", cellId);
    obs->Add("rrcState", rrcState);
    obs->Add("position", pos);
    obs->Add("velocity", vel);
    obs->Add("mcs", mcs);
    obs->Add("txPower", txPower);
    obs->Add("cwnd", cwnd);
    obs->Add("rtt", rtt);
    obs->Add("deliveryRate", deliveryRate);

    return obs;
}

float
HarlTcpHandoverAgentApp::GetResetReward()
{
    return 0.0f;
}

} // namespace ns3
