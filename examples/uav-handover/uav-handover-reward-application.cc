#include "uav-handover-reward-application.h"

#include "ns3/base-test.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-net-device.h"
#include "ns3/lte-rlc-sap.h"
#include "ns3/lte-rlc.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"

#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavHandoverRewardApplication");

TypeId
UavHandoverRewardApplication::GetTypeId()
{
    static TypeId tid =
        TypeId("UavHandoverRewardApplication")
            .SetParent<RewardApplication>()
            .AddConstructor<UavHandoverRewardApplication>()
            .AddAttribute("SimulationTime",
                          "The total simulation time",
                          TimeValue(Seconds(100)),
                          MakeTimeAccessor(&UavHandoverRewardApplication::m_simTime),
                          MakeTimeChecker())
            .AddAttribute("StepTime",
                          "Interval for calculation reward",
                          TimeValue(MilliSeconds(1000)),
                          MakeTimeAccessor(&UavHandoverRewardApplication::m_calculationInterval),
                          MakeTimeChecker())
            .AddAttribute("HandoverPenalty",
                          "Penalty applied to reward when a handover occurs in the step",
                          DoubleValue(1000.0),
                          MakeDoubleAccessor(&UavHandoverRewardApplication::m_handoverPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("LteHelper",
                          "The LteHelper object to use for rewards.",
                          PointerValue(),
                          MakePointerAccessor(&UavHandoverRewardApplication::m_lteHelper),
                          MakePointerChecker<LteHelper>());
    return tid;
}

extern NetDeviceContainer g_ueLteDevs;
extern uint32_t g_totalHandovers;

void
UavHandoverRewardApplication::SendReward()
{
    auto imsi = g_ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetImsi();

    auto throughput = m_lteHelper->GetRlcStats()->GetDlRxData(imsi, 3) /
                      m_calculationInterval.GetSeconds() / m_simTime.GetSeconds();

    // Detect how many valid handovers occurred since the last reward computation
    // g_totalHandovers only increments for valid handovers (not no-op, not same-cell).
    uint32_t handoverCount = g_totalHandovers - m_lastTotalHandovers;
    m_lastTotalHandovers = g_totalHandovers;

    // Apply per-handover penalty (each handover reduces the reward)
    double reward = throughput - (m_handoverPenalty * handoverCount);

    auto rewardContainer = MakeDictBoxContainer<double>(1, "reward", reward);
    NS_LOG_INFO("Reward (throughput: " << throughput
                                       << ", handovers: " << handoverCount
                                       << ", penalty: " << (m_handoverPenalty * handoverCount)
                                       << "): " << reward);
    Send(rewardContainer);
    Simulator::Schedule(m_calculationInterval, &UavHandoverRewardApplication::SendReward, this);
}

void
UavHandoverRewardApplication::RegisterCallbacks()
{
    if (!m_lteHelper->GetRlcStats())
    {
        m_lteHelper->EnableRlcTraces();
    }
    auto stats = m_lteHelper->GetRlcStats();
    stats->SetEpoch(m_calculationInterval);
    stats->SetStartTime(NanoSeconds(1));

    // Initialise handover tracking
    m_lastTotalHandovers = g_totalHandovers;

    Simulator::Schedule(m_calculationInterval, &UavHandoverRewardApplication::SendReward, this);
}
