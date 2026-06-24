/**
 * @file harl-tcp-handover-rwd-app.cc
 * @ingroup defiance
 *
 * @brief Reward application for the HARL TCP handover RL agent.
 *
 * Reward design:
 *   reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm
 *
 *   normGoodput:
 *      1.0                                    if goodput >= referenceRate
 *      (goodput - minAcceptable) / (ref - min) if minAcceptable <= goodput < ref
 *      (goodput - minAcceptable) / minAcceptable   if goodput < minAcceptable (negative)
 *
 *   rttPenalty:
 *      0.0                                    if rtt <= delayMinRtt
 *      (rtt - delayMinRtt) / (maxRtt - delayMinRtt)  if delayMinRtt < rtt < maxRtt
 *      1.0                                    if rtt >= maxAcceptableRtt
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

// External globals from the scenario (outside namespace ns3 to match definitions in
// harl-tcp-scenario.cc)
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
                          "Interval (s) between reward calculations. Aligned with MS480 "
                          "measurement report interval (480ms).",
                          TimeValue(MilliSeconds(480)),
                          MakeTimeAccessor(&HarlTcpHandoverRewardApp::m_calculationInterval),
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

    // Initialize handover tracking and RTT baseline
    m_lastTotalHandovers = g_totalHandovers;
    m_currentRttMs = static_cast<int32_t>(m_delayMinRttMs);

    // Schedule first reward computation after apps start (~1.0s + margin)
    Simulator::Schedule(Seconds(1.5) + m_calculationInterval,
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
HarlTcpHandoverRewardApp::SendReward()
{
    // --- 1. Compute UL goodput (bps) from sink bytes this step ---
    double intervalSec = m_calculationInterval.GetSeconds();
    double goodputBps = 0.0;
    if (intervalSec > 0.0)
    {
        goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / intervalSec;
    }

    // --- 2. Normalize throughput ---
    //     goodput >= referenceRate             → 1.0
    //     minAcceptable <= goodput < refRate    → 0..1 linearly
    //     goodput < minAcceptable               → negative (0 at minAcceptable, -1 at 0 bps)
    double normGoodput = 0.0;
    if (m_referenceRateBps > 0.0 && m_minimumAcceptableGoodputBps > 0.0)
    {
        if (goodputBps >= m_referenceRateBps)
        {
            normGoodput = 1.0;
        }
        else if (goodputBps >= m_minimumAcceptableGoodputBps)
        {
            normGoodput = (goodputBps - m_minimumAcceptableGoodputBps) /
                          (m_referenceRateBps - m_minimumAcceptableGoodputBps);
        }
        else
        {
            normGoodput =
                (goodputBps - m_minimumAcceptableGoodputBps) / m_minimumAcceptableGoodputBps;
        }
    }

    // --- 3. Compute delay penalty (self-normalized ramp, no weight multiplier) ---
    //     rtt <= delayMinRtt    → 0.0
    //     rtt >= maxAcceptableRtt → 1.0
    //     delayMinRtt < rtt < maxRtt → linear 0..1
    double rttPenalty = 0.0;
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

    // Keep track of handovers for logging only (no penalty)
    m_lastTotalHandovers = g_totalHandovers;

    // --- 4. Apply TCP failure penalty ---
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

    // --- 5. Compute reward ---
    double reward = normGoodput - rttPenalty; // removing tcp penalty and rlf term for now should be captured by throughput/rtt

    // Wide clamp as safety net only (should not trigger after scaling)
    reward = std::max(reward, -100.0);

    NS_LOG_INFO("Reward: goodput=" << goodputBps << "bps"
                                   << " normGoodput=" << normGoodput << " rtt=" << m_currentRttMs
                                   << "ms"
                                   << " rttPenalty=" << rttPenalty << " tcpPenalty=" << tcpPenalty
                                   << " rlfTerm=" << rlfTerm << " reward=" << reward);

    // std::cout << "Reward: goodput=" << goodputBps << "bps"
    //           << " normGoodput=" << normGoodput << " rtt=" << m_currentRttMs << "ms"
    //           << " rttPenalty=" << rttPenalty << " tcpPenalty=" << tcpPenalty
    //           << " rlfTerm=" << rlfTerm << " reward=" << reward << std::endl;

    // --- 6. Send reward to agent ---
    auto rewardContainer = MakeDictBoxContainer<double>(1, "reward", reward);
    Send(rewardContainer);

    // Reset step counter
    m_sinkBytesReceived = 0;

    // Schedule next reward computation
    Simulator::Schedule(m_calculationInterval, &HarlTcpHandoverRewardApp::SendReward, this);
}

} // namespace ns3
