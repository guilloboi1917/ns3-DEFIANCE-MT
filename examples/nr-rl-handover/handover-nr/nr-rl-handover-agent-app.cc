#include "nr-rl-handover-agent-app.h"

#include "ns3/base-test.h"

#include <cstdint>
#include <vector>

#include <string>

// External globals from the scenario (defined in nr-rl-handover-scenario.cc).

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
                          "Number of base stations/cells in the simulation.",
                          UintegerValue(9),
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
    NS_LOG_INFO("NrRlHandoverAgentApp setup complete, numBs=" << m_numBs << " topN=" << m_topN);
}

void
NrRlHandoverAgentApp::OnRecvObs(uint id)
{
    NS_LOG_FUNCTION(this << id);
    auto data = m_obsDataStruct.GetNewestByID(id)->data;

    // Data is a Dict wrapping flat Box ("obs" key). Store directly.
    m_observation = data;

    NS_LOG_INFO("Inferring handover action at t=" << Simulator::Now().GetSeconds() << "s");
    InferAction();
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

    // action is a DiscreteContainer (0..m_topN): 0 = no-op, 1..m_topN = handover
    auto dictAction = CreateObject<OpenGymDictContainer>();
    dictAction->Add("actionIndex", action);
    SendAction(dictAction);
}

Ptr<OpenGymSpace>
NrRlHandoverAgentApp::GetObservationSpace()
{
    // Dict wrapping 32-dim Box (key "obs") to match Send() data format
    auto dictSpace = CreateObject<OpenGymDictSpace>();

    //   [0-1] serving_rsrp/rsrq, [2-4] slot_rsrp[0..2], [5-7] rsrp_delta[0..2],
    //   [8] dl_sinr, [9] time_since_ho, [10] norm_goodput, [11] ho_count_10s,
    //   [12-14] ul_sinr/ul_rb_util/ul_sched_ue,
    //   [15-19] d_serving_rsrp/sinr/rsrq, d_norm_goodput, d_margin,
    //   [20-22] d_slot_rsrp[0..2], [23-25] heading_x/y/z,
    //   [26-28] pos/ISD, [29-31] pos2s/ISD
    std::vector<float> low = {-160.0f, -100.0f,
                              -160.0f, -160.0f, -160.0f,
                              -60.0f, -60.0f, -60.0f,
                              -40.0f, 0.0f, 0.0f, 0.0f,
                              -40.0f, 0.0f, 0.0f,
                              -20.0f, -20.0f, -20.0f, -2.0f, -20.0f,
                              -20.0f, -20.0f, -20.0f,
                              -1.0f, -1.0f, -1.0f,
                              0.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0.0f};
    std::vector<float> high = {-40.0f, -3.0f,
                               -40.0f, -40.0f, -40.0f,
                               60.0f, 60.0f, 60.0f,
                               50.0f, 10.0f, 2.0f, 10.0f,
                               50.0f, 1.0f, 1.0f,
                               20.0f, 20.0f, 20.0f, 2.0f, 20.0f,
                               20.0f, 20.0f, 20.0f,
                               1.0f, 1.0f, 1.0f,
                               3.0f, 3.0f, 3.0f,
                               3.0f, 3.0f, 3.0f};

    std::vector<uint32_t> shape = {32};
    auto boxSpace = CreateObject<OpenGymBoxSpace>(low, high, shape, TypeNameGet<double>());
    dictSpace->Add("obs", boxSpace);
    return dictSpace;
}

Ptr<OpenGymSpace>
NrRlHandoverAgentApp::GetActionSpace()
{
    // Top-N action space: 0 = no-op, 1..m_topN = handover to ranked cell
    return CreateObject<OpenGymDiscreteSpace>(m_topN + 1);
}

Ptr<OpenGymDictContainer>
NrRlHandoverAgentApp::GetResetObservation() const
{
    // 32-dim sentinel vector wrapped in Dict("obs")
    auto box = MakeBoxContainer<double>(32);
    // [0-1] serving_rsrp, serving_rsrq
    box->AddValue(-140.0);
    box->AddValue(-20.0);
    // [2-4] slot_rsrp[0..2]
    for (uint32_t i = 0; i < 3; i++)
    {
        box->AddValue(-140.0);
    }
    // [5-7] rsrp_delta[0..2]
    for (uint32_t i = 0; i < 3; i++)
    {
        box->AddValue(0.0);
    }
    // [8] sinr, [9] time_since_ho, [10] norm_goodput, [11] ho_count_10s
    box->AddValue(-40.0);
    box->AddValue(10.0);
    box->AddValue(0.0);
    box->AddValue(0.0);
    // [12-14] UL block (sentinels)
    box->AddValue(-40.0);
    box->AddValue(0.0);
    box->AddValue(0.0);
    // [15-19] time-delta block
    for (uint32_t i = 0; i < 5; i++)
    {
        box->AddValue(0.0);
    }
    // [20-22] d_slot_rsrp[0..2]
    for (uint32_t i = 0; i < 3; i++)
    {
        box->AddValue(0.0);
    }
    // [23-25] heading, [26-31] pos + pos2s (zeros; within bounds)
    for (uint32_t i = 0; i < 9; i++)
    {
        box->AddValue(0.0);
    }

    auto obs = CreateObject<OpenGymDictContainer>();
    obs->Add("obs", box);
    return obs;
}

float
NrRlHandoverAgentApp::GetResetReward()
{
    return 0.0f;
}

std::map<std::string, std::string>
NrRlHandoverAgentApp::GetExtraInfo()
{
    auto info = AgentApplication::GetExtraInfo();
    // The act-app set this during the previous action processing; with the
    // agent-app's NotifyCurrentState it ships to Python (info channel) so the
    // replay can store the executed action (0 = stay) instead of the intent.
    extern int g_effectiveAction;
    info["effective_action"] = std::to_string(g_effectiveAction);
    return info;
}

} // namespace ns3
