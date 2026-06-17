/**
 * @file harl-tcp-handover-rwd-app.cc
 * @ingroup defiance
 *
 * @brief Reward application for the HARL TCP handover RL agent.
 *
 * Reward design:
 *   reward = normGoodput - hoPenalty*hoCount - rttPenalty - tcpPenalty - rlfTerm - pingPongTerm
 *
 *   normGoodput    = min(goodput / referenceRate, 1.0)
 *   rttPenalty     = rttWeight * max(0, rttMs - minRttMs) / minRttMs
 *   tcpPenalty     = tcpFailurePenalty            (per step when !g_tcpAlive)
 *   rlfTerm        = rlfPenalty                   (one-shot on RLF)
 *   pingPongTerm   = pingPongPenalty * hoCount    (if handovers within PingPongInterval)
 *
 * - Goodput is measured from the PacketSink Rx trace on the remote host
 *   (UAV is the TCP sender, UL application-layer goodput).
 * - All penalties are scaled to be comparable to normGoodput [0,1].
 */

#include "harl-tcp-handover-rwd-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/packet.h"

#include <algorithm>
#include <cstdint>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in harl-tcp-scenario.cc)
extern NetDeviceContainer g_uavLteDevs;
extern NodeContainer g_uavContainer;
extern NodeContainer g_remoteHostContainer;
extern uint32_t g_totalHandovers;
extern bool g_tcpAlive;
extern bool g_rlfTriggered;

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
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_handoverPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReferenceRate",
                          "Reference UL data rate (bps) for throughput normalization. "
                          "Throughput is divided by this value and clamped to [0,1].",
                          DoubleValue(30000000.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_referenceRateBps),
                          MakeDoubleChecker<double>())
            .AddAttribute("RttPenaltyWeight",
                          "Weight of the RTT inflation penalty term. "
                          "rttInflation = max(0, currentRtt - minRtt) / minRtt.",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_rttPenaltyWeight),
                          MakeDoubleChecker<double>())
            .AddAttribute("MinRttMs",
                          "Baseline RTT (ms) for inflation calculation. "
                          "Floor is ~10ms PGW-Server p2p + ~20ms LTE processing.",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_minRttMs),
                          MakeDoubleChecker<double>())
            .AddAttribute("TcpFailurePenalty",
                          "Reward penalty per step when TCP connection is dead. "
                          "Applied when g_tcpAlive is false (e.g., connection "
                          "failed mid-simulation or socket closed).",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_tcpFailurePenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("RlfPenalty",
                          "One-time reward penalty applied on the step when RLF "
                          "is detected (UAV drops from CONNECTED_NORMALLY after "
                          "TCP was established). Only fires once per episode.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_rlfPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("PingPongPenalty",
                          "Extra penalty per handover when consecutive handovers "
                          "occur within PingPongInterval. Discourages ping-pong "
                          "(rapid back-and-forth handovers).",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&HarlTcpHandoverRewardApp::m_pingPongPenalty),
                          MakeDoubleChecker<double>())
            .AddAttribute("PingPongInterval",
                          "Minimum time (s) between handovers to avoid ping-pong penalty.",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&HarlTcpHandoverRewardApp::m_pingPongInterval),
                          MakeTimeChecker());
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

    NS_LOG_INFO("Connecting to PacketSink Rx on remoteHost node " << m_remoteHostNodeId);

    // Connect to the PacketSink Rx trace on the remote host
    // This captures every packet the UAV's TCP sender delivers to the sink.
    std::string rxPath = "/NodeList/" + std::to_string(m_remoteHostNodeId) +
                         "/ApplicationList/*/$ns3::PacketSink/Rx";
    Config::ConnectWithoutContext(
        rxPath,
        MakeCallback(&HarlTcpHandoverRewardApp::ObserveSinkRx, this));

    // Connect to TCP RTT trace on the UAV node
    // Schedule after TCP sockets are created (~1.0s)
    uint32_t uavNodeId =
        (g_uavContainer.GetN() > 0) ? g_uavContainer.Get(0)->GetId() : 0;
    if (uavNodeId > 0)
    {
        Simulator::Schedule(Seconds(1.5), [this, uavNodeId]() {
            std::string rttPath = "/NodeList/" + std::to_string(uavNodeId) +
                                  "/$ns3::TcpL4Protocol/SocketList/0/RTT";
            Config::ConnectWithoutContext(
                rttPath,
                MakeCallback(&HarlTcpHandoverRewardApp::ObserveRtt, this));
            NS_LOG_INFO("RTT trace connected on UAV node " << uavNodeId);
        });
    }

    // Initialize handover tracking and RTT baseline
    m_lastTotalHandovers = g_totalHandovers;
    m_currentRttMs = static_cast<int32_t>(m_minRttMs);

    // Schedule first reward computation after apps start (~1.0s + margin)
    Simulator::Schedule(Seconds(1.5) + m_calculationInterval,
                        &HarlTcpHandoverRewardApp::SendReward,
                        this);

    NS_LOG_INFO("HarlTcpHandoverRewardApp registered: interval="
                << m_calculationInterval.GetMilliSeconds() << "ms"
                << ", refRate=" << m_referenceRateBps << "bps"
                << ", HOpenalty=" << m_handoverPenalty
                << ", RttWeight=" << m_rttPenaltyWeight
                << ", minRtt=" << m_minRttMs << "ms");
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
HarlTcpHandoverRewardApp::SendReward()
{
    // --- 1. Compute UL goodput (bps) from sink bytes this step ---
    double intervalSec = m_calculationInterval.GetSeconds();
    double goodputBps = 0.0;
    if (intervalSec > 0.0)
    {
        goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / intervalSec;
    }

    // --- 2. Normalize throughput to [0, 1] ---
    double normGoodput = 0.0;
    if (m_referenceRateBps > 0.0)
    {
        normGoodput = std::min(goodputBps / m_referenceRateBps, 1.0);
    }

    // --- 3. Count handovers since last reward ---
    uint32_t handoverCount = g_totalHandovers - m_lastTotalHandovers;

    // --- 3b. Ping-pong penalty: extra cost for handovers too close together ---
    double pingPongTerm = 0.0;
    if (handoverCount > 0)
    {
        Time now = Simulator::Now();
        if (m_lastHandoverTime > Seconds(0) &&
            now - m_lastHandoverTime < m_pingPongInterval)
        {
            pingPongTerm = m_pingPongPenalty * handoverCount;
            NS_LOG_DEBUG("Ping-pong: " << handoverCount << " handovers within "
                          << m_pingPongInterval.GetSeconds() << "s of last handover, penalty="
                          << pingPongTerm);
        }
        m_lastHandoverTime = now;
    }

    m_lastTotalHandovers = g_totalHandovers;

    // --- 4. Compute RTT inflation penalty ---
    double rttInflation = 0.0;
    if (m_minRttMs > 0.0 && m_currentRttMs > m_minRttMs)
    {
        rttInflation = (m_currentRttMs - m_minRttMs) / m_minRttMs;
    }
    double rttPenalty = m_rttPenaltyWeight * rttInflation;

    // --- 5. Apply TCP failure penalty ---
    double tcpPenalty = 0.0;
    if (!g_tcpAlive)
    {
        tcpPenalty = m_tcpFailurePenalty;
        NS_LOG_DEBUG("TCP not alive, applying penalty: " << m_tcpFailurePenalty);
    }

    // --- 6. Apply RLF penalty (one-time) ---
    double rlfTerm = 0.0;
    if (g_rlfTriggered)
    {
        rlfTerm = m_rlfPenalty;
        g_rlfTriggered = false; // one-shot: only penalise the step RLF occurs
        NS_LOG_DEBUG("RLF detected, applying penalty: " << m_rlfPenalty);
    }

    // --- 7. Compute reward ---
    double reward = normGoodput - (m_handoverPenalty * handoverCount) - rttPenalty - tcpPenalty - rlfTerm - pingPongTerm;

    // Wide clamp as safety net only (should not trigger after scaling)
    reward = std::max(reward, -100.0);

    NS_LOG_INFO("Reward: goodput=" << goodputBps << "bps"
                << " (" << (normGoodput * 100.0) << "%)"
                << " rtt=" << m_currentRttMs << "ms"
                << " rttInflation=" << rttInflation
                << " rttPenalty=" << rttPenalty
                << " handovers=" << handoverCount
                << " hoPenalty=" << (m_handoverPenalty * handoverCount)
                << " pingPong=" << pingPongTerm
                << " reward=" << reward);

    // --- 7. Send reward to agent ---
    auto rewardContainer = MakeDictBoxContainer<double>(1, "reward", reward);
    Send(rewardContainer);

    // Reset step counter
    m_sinkBytesReceived = 0;

    // Schedule next reward computation
    Simulator::Schedule(m_calculationInterval,
                        &HarlTcpHandoverRewardApp::SendReward,
                        this);
}

} // namespace ns3
