/**
 * @file harl-tcp-handover-rwd-app.cc
 * @ingroup defiance
 *
 * @brief Reward application for the HARL TCP handover RL agent.
 *
 * Reward design:
 *   reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm + tbsBonus
 *
 *   normGoodput (EWMA-based adaptive reference):
 *      1.0                                    if goodput >= dynamicRef
 *      (goodput - dynamicMin) / (ref - min)   if dynamicMin <= goodput < dynamicRef
 *      (goodput - dynamicMin) / dynamicMin    if goodput < dynamicMin (negative)
 *
 *   dynamicRef = m_ewmaGoodput x 1.1   (lagging EWMA of actual goodput)
 *   dynamicMin = m_ewmaGoodput x 0.3
 *
 *   tbsBonus = small TBS-based bonus for cell quality (0 to 0.3)
 *
 *   rttPenalty:
 *      0.0                                    if rtt <= delayMinRtt
 *      (rtt - delayMinRtt) / (maxRtt - delayMinRtt)  if delayMinRtt < rtt < maxRtt
 *      1.0                                    if rtt >= maxAcceptableRtt
 *
 * - Goodput is measured from the PacketSink Rx trace on the remote host
 *   (UAV is the TCP sender, UL application-layer goodput).
 * - The EWMA-based reference replaces the old TBS-based one to avoid the
 *   inverted incentive where bad cells (low TBS) had easier targets.
 */

#include "harl-tcp-handover-rwd-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/packet.h"

#include <algorithm>
#include <cstdint>
#include <fstream>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in
// harl-tcp-scenario.cc)
extern NetDeviceContainer g_uavLteDevs;
extern NodeContainer g_uavContainer;
extern NodeContainer g_remoteHostContainer;
extern uint32_t g_totalHandovers;
extern bool g_tcpAlive;
extern std::vector<double> g_lastRsrpValues;
extern bool g_rlfTriggered;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("HarlTcpHandoverRewardApp");

HarlTcpHandoverRewardApp::HarlTcpHandoverRewardApp()
    : RewardApplication()
{
}

HarlTcpHandoverRewardApp::~HarlTcpHandoverRewardApp()
{
}

TypeId
HarlTcpHandoverRewardApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::HarlTcpHandoverRewardApp")
            .SetParent<RewardApplication>()
            .SetGroupName("defiance")
            .AddConstructor<HarlTcpHandoverRewardApp>()
            .AddAttribute("RemoteHostNodeId",
                          "Node ID of the remote host (PacketSink location).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&HarlTcpHandoverRewardApp::m_remoteHostNodeId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverPenalty",
                          "Reward penalty per handover (in normalized [0,1] units).",
                          DoubleValue(0.03),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_handoverPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReferenceRate",
                          "Reference UL data rate (bps) for throughput normalization. "
                          "Throughput >= this value yields reward=1.",
                          DoubleValue(5000000.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_referenceRateBps),
                          MakeDoubleChecker<double>())
            .AddAttribute(
                "MinimumAcceptableGoodput",
                "Minimum acceptable UL goodput (bps). "
                "Goodput below this yields negative reward; "
                "goodput between this and ReferenceRate ramps linearly 0->1.",
                DoubleValue(2500000.0),
                MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_minimumAcceptableGoodputBps),
                MakeDoubleChecker<double>())
            .AddAttribute("DelayMinRttMs",
                          "Lower bound RTT (ms) for delay penalty. "
                          "No penalty when RTT <= this value.",
                          DoubleValue(55.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_delayMinRttMs),
                          MakeDoubleChecker<double>())
            .AddAttribute("MaxAcceptableRttMs",
                          "Upper bound RTT (ms) for delay penalty. "
                          "Penalty clamped at 1 when RTT >= this value.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_maxAcceptableRttMs),
                          MakeDoubleChecker<double>())
            .AddAttribute("TcpFailurePenalty",
                          "Reward penalty per step when TCP connection is dead. "
                          "Applied when g_tcpAlive is false (e.g., connection "
                          "failed mid-simulation or socket closed).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_tcpFailurePenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("RlfPenalty",
                          "One-time reward penalty applied on the step when RLF "
                          "is detected (UAV drops from CONNECTED_NORMALLY after "
                          "TCP was established). Only fires once per episode.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_rlfPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("CalculationInterval",
                          "Interval (s) between reward calculations. Aligned with "
                          "ReportUeMeasurements cadence (200ms).",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&HarlTcpHandoverRewardApp::m_calculationInterval),
                          MakeTimeChecker())
            .AddAttribute("EwmaAlpha",
                          "EWMA smoothing factor for goodput-based adaptive reference. "
                          "0.2 = 5-step (~1s) window for faster reaction to throughput changes.",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_ewmaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("TbsBonusWeight",
                          "TBS throughput bonus weight per Mbps (capped at 0.3). "
                          "Rewards being on a physically capable cell.",
                          DoubleValue(0.003),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_tbsBonusWeight),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

void
HarlTcpHandoverRewardApp::RegisterCallbacks()
{
    // Determine remote host node ID if not explicitly set
    if (m_remoteHostNodeId == 0 && g_remoteHostContainer.GetN() > 0)
    {
        m_remoteHostNodeId = g_remoteHostContainer.Get(0)->GetId();
    }

    // Cache UAV node ID and reserve TBS buffer
    if (g_uavContainer.GetN() > 0)
    {
        m_uavNodeId = g_uavContainer.Get(0)->GetId();
        m_tbsHistory.reserve(512); // enough for ~500ms at 1ms TTI
    }

    // Connect to UL PHY transmission trace for TBS values
    if (m_uavNodeId > 0)
    {
        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(m_uavNodeId) +
                "/DeviceList/*/$ns3::LteUeNetDevice/"
                "ComponentCarrierMapUe/*/LteUePhy/"
                "UlPhyTransmission",
            MakeCallback(&HarlTcpHandoverRewardApp::ObserveUlPhyTransmission, this));
    }

    NS_LOG_INFO("Connecting to PacketSink Rx on remoteHost node " << m_remoteHostNodeId);

    // Connect to the PacketSink Rx trace on the remote host
    // This captures every packet the UAV's TCP sender delivers to the sink.
    std::string rxPath = "/NodeList/" + std::to_string(m_remoteHostNodeId) +
                         "/ApplicationList/*/$ns3::PacketSink/Rx";
    Config::ConnectWithoutContext(rxPath,
                                  MakeCallback(&HarlTcpHandoverRewardApp::ObserveSinkRx, this));

    // Connect to TCP RTT trace on the UAV node
    // Schedule after TCP sockets are created (~1.0s)
    uint32_t uavNodeId = (g_uavContainer.GetN() > 0) ? g_uavContainer.Get(0)->GetId() : 0;
    if (uavNodeId > 0)
    {
        Simulator::Schedule(Seconds(1.5), [this, uavNodeId]() {
            std::string rttPath =
                "/NodeList/" + std::to_string(uavNodeId) + "/$ns3::TcpL4Protocol/SocketList/0/RTT";
            Config::ConnectWithoutContext(
                rttPath,
                MakeCallback(&HarlTcpHandoverRewardApp::ObserveRtt, this));
            NS_LOG_INFO("RTT trace connected on UAV node " << uavNodeId);
        });
    }

    // Initialize handover tracking, RTT baseline, and one-shot penalty flags
    m_lastTotalHandovers = g_totalHandovers;
    m_currentRttMs = static_cast<int32_t>(m_delayMinRttMs);
    m_tcpPenaltyApplied = false;

    // Schedule first reward computation shortly after app start (aligns with first observation)
    // TCP is already running at this point (started at 1.5s), so goodput data is available.
    Simulator::Schedule(m_calculationInterval,
                        &HarlTcpHandoverRewardApp::SendReward,
                        this);

    NS_LOG_INFO("HarlTcpHandoverRewardApp registered: interval="
                << m_calculationInterval.GetMilliSeconds() << "ms"
                << ", refRate=" << m_referenceRateBps << "bps"
                << ", minAcceptableGoodput=" << m_minimumAcceptableGoodputBps << "bps"
                << ", delayMinRtt=" << m_delayMinRttMs << "ms"
                << ", maxAcceptableRtt=" << m_maxAcceptableRttMs << "ms");
}

void
HarlTcpHandoverRewardApp::ObserveSinkRx(Ptr<const Packet> packet, const Address& from)
{
    m_sinkBytesReceived += packet->GetSize();
}

void
HarlTcpHandoverRewardApp::ObserveRtt(Time oldRtt, Time newRtt)
{
    m_currentRttMs = static_cast<int32_t>(newRtt.GetMilliSeconds());
    NS_LOG_INFO("RTT updated: " << m_currentRttMs << "ms");
}

void
HarlTcpHandoverRewardApp::ObserveUlPhyTransmission(PhyTransmissionStatParameters param)
{
    int32_t tbs = static_cast<int32_t>(param.m_size);
    m_tbsHistory.push_back(tbs);
}

void
HarlTcpHandoverRewardApp::SendReward()
{
    // --- 1. Compute UL goodput (bps) from sink bytes this step ---
    double intervalSec = m_calculationInterval.GetSeconds();
    double goodputBps = 0.0;
    if (intervalSec > 0.0)
    {
        goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / intervalSec;
    }

    // --- 2. Compute adaptive reference rate from EWMA of actual goodput ---
    //     dynamicRef = m_ewmaGoodput × 1.1    (10% above recent throughput)
    //     dynamicMin = m_ewmaGoodput × 0.3    (30% of recent throughput)
    //
    //     The EWMA lags behind real goodput. After a good handover, goodput
    //     rises above the EWMA, pushing normGoodput above 1.0 for several
    //     steps until the EWMA catches up. This transient overshoot is the
    //     improvement signal that drives exploration toward better cells.
    //
    //     Compute TBS throughput for the bonus first (before clearing history)
    double tbsThroughputBps = 0.0;
    if (!m_tbsHistory.empty())
    {
        int64_t tbsSum = 0;
        for (auto tbs : m_tbsHistory)
        {
            tbsSum += tbs;
        }
        tbsThroughputBps = (static_cast<double>(tbsSum) / m_tbsHistory.size()) * 8000.0;
        m_tbsHistory.clear();
    }

    double dynamicRefBps = m_referenceRateBps;
    double dynamicMinBps = m_minimumAcceptableGoodputBps;
    if (m_ewmaGoodput > 0.0)
    {
        dynamicRefBps = m_ewmaGoodput * 1.1;
        dynamicMinBps = m_ewmaGoodput * 0.3;
    }

    // Update EWMA with current goodput (after computing ref, so ref uses last step's EWMA)
    if (m_ewmaGoodput <= 0.0)
    {
        m_ewmaGoodput = goodputBps;
    }
    else
    {
        m_ewmaGoodput = m_ewmaAlpha * goodputBps + (1.0 - m_ewmaAlpha) * m_ewmaGoodput;
    }

    // --- 3. Normalize throughput ---
    //     goodput >= dynamicRef                → 1.0
    //     dynamicMin <= goodput < dynamicRef    → 0..1 linearly
    //     goodput < dynamicMin                  → negative
    //
    //     When TCP is not alive, cap negative normGoodput at -0.2 so the
    //     agent isn't flooded with -1 per step while waiting for TCP retransmission.
    double normGoodput = 0.0;
    if (dynamicRefBps > 0.0 && dynamicMinBps > 0.0)
    {
        if (goodputBps >= dynamicRefBps)
        {
            normGoodput = 1.0;
        }
        else if (goodputBps >= dynamicMinBps)
        {
            normGoodput = (goodputBps - dynamicMinBps) / (dynamicRefBps - dynamicMinBps);
        }
        else
        {
            normGoodput = (goodputBps - dynamicMinBps) / dynamicMinBps;
        }
    }

    // When TCP is dead, set normGoodput to 0 (neutral) instead of negative.
    // The one-shot tcpPenalty (-0.5) already fires once. Additional negative
    // normGoodput would drown out the reward signal for 10-40 SYN retransmission steps.
    if (!g_tcpAlive)
    {
        normGoodput = 0.0;
    }

    // --- 3. Compute delay penalty (self-normalized ramp, no weight multiplier) ---
    //     rtt <= delayMinRtt    → 0.0
    //     rtt >= maxAcceptableRtt → 1.0
    //     delayMinRtt < rtt < maxRtt → linear 0..1
    //     If TCP is dead, RTT is stale — set penalty to 0.
    double rttPenalty = 0.0;
    if (g_tcpAlive)
    {
        double rangeMs = m_maxAcceptableRttMs - m_delayMinRttMs;
        if (rangeMs > 0.0)
        {
            if (m_currentRttMs >= m_maxAcceptableRttMs)
            {
                rttPenalty = 1.0;
            }
            else if (m_currentRttMs > m_delayMinRttMs)
            {
                rttPenalty = (m_currentRttMs - m_delayMinRttMs) / rangeMs;
            }
        }
    }

    // --- 4. Apply TCP failure penalty (one-shot) ---
    double tcpPenalty = 0.0;
    if (!g_tcpAlive && !m_tcpPenaltyApplied)
    {
        tcpPenalty = m_tcpFailurePenalty;
        m_tcpPenaltyApplied = true;
        NS_LOG_DEBUG("TCP not alive, applying one-shot penalty: " << m_tcpFailurePenalty);
    }

    // --- 5. Apply RLF penalty (one-time) ---
    double rlfTerm = 0.0;
    if (g_rlfTriggered)
    {
        rlfTerm = m_rlfPenalty;
        g_rlfTriggered = false;
        NS_LOG_DEBUG("RLF detected, applying penalty: " << m_rlfPenalty);
    }

    // Query current serving cell ID and RSRP
    uint32_t servingCellId = 0;
    double currentRsrp = -140.0;
    if (g_uavLteDevs.GetN() > 0)
    {
        auto ueNetDev = g_uavLteDevs.Get(0)->GetObject<LteUeNetDevice>();
        if (ueNetDev && ueNetDev->GetRrc())
        {
            servingCellId = ueNetDev->GetRrc()->GetCellId();
            if (servingCellId > 0 && servingCellId < g_lastRsrpValues.size())
            {
                currentRsrp = g_lastRsrpValues[servingCellId];
            }
        }
    }

    // --- 6. Handover penalty + RSRP delta bonus (triggered on handover) ---
    double handoverPenalty = 0.0;
    double rsrpDeltaBonus = 0.0;
    if (g_totalHandovers > m_lastTotalHandovers)
    {
        handoverPenalty = m_handoverPenalty;

        // RSRP delta: compare new serving cell to previous serving cell
        double deltaRsrp = currentRsrp - m_previousServingRsrp;
        // Map [-10, +10] dB → [-3.0, +2.0] reward, clamped.
        rsrpDeltaBonus = std::max(-3.0, std::min(2.0, deltaRsrp / 5.0));

        m_lastTotalHandovers = g_totalHandovers;
        NS_LOG_DEBUG("Handover detected: penalty=" << m_handoverPenalty
                      << " deltaRsrp=" << deltaRsrp << "dB rsrpDeltaBonus=" << rsrpDeltaBonus);
    }

    // Save current RSRP for next step's delta computation
    m_previousServingRsrp = currentRsrp;

    // --- 7. Compute TBS bonus (cell radio quality) ---
    double tbsBonus = 0.0;
    if (tbsThroughputBps > 0.0)
    {
        tbsBonus = std::min(0.3, m_tbsBonusWeight * tbsThroughputBps / 1e6);
    }

    // --- 8. Compute reward ---
    // double reward =
    //     normGoodput - rttPenalty -
    //     tcpPenalty - handoverPenalty + tbsBonus + rsrpDeltaBonus;
    double rsrpPerStepBonus = (currentRsrp + 110) / 60 * 0.5;
    double reward = normGoodput + rsrpPerStepBonus - handoverPenalty;

    // Wide clamp as safety net only
    reward = std::max(reward, -100.0);

    // Log reward components to CSV if logging is enabled
    if (g_logging)
    {
        std::ofstream rwdFile(g_outputDir + "rl_reward.csv", std::ios_base::app);
        rwdFile << Simulator::Now().GetSeconds() << ","
                << (goodputBps / 1e6) << ","
                << (dynamicRefBps / 1e6) << ","
                << (dynamicMinBps / 1e6) << ","
                << normGoodput << ","
                // << m_currentRttMs << ","
                // << rttPenalty << ","
                // << tcpPenalty << ","
                // << tbsBonus << ","
                << handoverPenalty << ","
                << rsrpDeltaBonus << ","
                << reward << std::endl;
    }

    NS_LOG_INFO("Reward: goodput=" << goodputBps << "bps"
                                   << " dynRef=" << dynamicRefBps << "bps"
                                   << " dynMin=" << dynamicMinBps << "bps"
                                   << " normGoodput=" << normGoodput << " rtt=" << m_currentRttMs
                                   << "ms"
                                   << " rttPenalty=" << rttPenalty << " tcpPenalty=" << tcpPenalty
                                   << " rlfTerm=" << rlfTerm << " tbsBonus=" << tbsBonus
                                   << " hoPenalty=" << handoverPenalty
                                   << " rsrpDelta=" << rsrpDeltaBonus
                                   << " reward=" << reward);

    // --- 10. Send reward to agent ---
    auto rewardContainer = MakeDictBoxContainer<double>(1, "reward", reward);
    Send(rewardContainer);

    // Reset step counter
    m_sinkBytesReceived = 0;

    // Schedule next reward computation
    Simulator::Schedule(m_calculationInterval, &HarlTcpHandoverRewardApp::SendReward, this);
}

} // namespace ns3
