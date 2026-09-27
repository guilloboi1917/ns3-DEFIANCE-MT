/**
 * @file nr-rl-handover-rwd-app.cc
 * @ingroup defiance
 *
 * @brief Reward application for the NR RL handover agent (Deng-style formulation).
 *
 * Reward design (Deng et al. Paper 5):
 *   R = alpha * R_G + (1 - alpha) * R_H
 *
 *   R_G = 1 / (1 + beta_G * max(0, 1 - normGoodput))
 *   R_H = 1 / (1 + beta_H * w)   with w = exp(-age / N) inside the hangover
 *                                 window (age = steps since the handover, N =
 *                                 rlHandoverHangoverLength), w = 0 outside
 *
 * Anti-hoofing mechanisms:
 * 1. Decaying handover hangover: after each handover the tax weight decays
 *    exponentially inside a window of N steps — cost is front-loaded (the
 *    interruption hits right after the event), no cliff at the window edge,
 *    and the tax is predictable from the obs (time_since_ho).
 * 2. Ping-pong detection: beta_H multiplied on A->B->A patterns
 */

#include "nr-rl-handover-rwd-app.h"

#include "ns3/base-test.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/node-list.h"
#include "ns3/packet.h"

#include <algorithm>
#include <cstdint>
#include <fstream>

using namespace ns3;

// External globals from the scenario
extern std::string g_flowDirection;
extern uint32_t g_receiverNodeId;
extern NetDeviceContainer g_uavNrDevs;
extern NodeContainer g_uavContainer;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlHandoverRewardApp");

NrRlHandoverRewardApp::NrRlHandoverRewardApp()
    : RewardApplication()
{
    m_handoverHistory[0] = 0;
    m_handoverHistory[1] = 0;
    m_handoverHistory[2] = 0;
    m_handoverTimes[0] = 0.0;
    m_handoverTimes[1] = 0.0;
    m_handoverTimes[2] = 0.0;
}

NrRlHandoverRewardApp::~NrRlHandoverRewardApp()
{
}

TypeId
NrRlHandoverRewardApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlHandoverRewardApp")
            .SetParent<RewardApplication>()
            .SetGroupName("defiance")
            .AddConstructor<NrRlHandoverRewardApp>()
            .AddAttribute("AlphaGoodput",
                          "Weight for goodput in Deng-style reward [0,1]. "
                          "1-alpha is the handover penalty weight.",
                          DoubleValue(0.8),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_alphaGoodput),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BetaGoodput",
                          "Goodput sensitivity for Deng-style inverse reward. "
                          "Higher = goodput drops penalised more. R_G = 1/(1+beta*(1-normG)). "
                          "Applies only to the deng shape.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_betaGoodput),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("RewardGoodputShape",
                          "R_G functional shape: deng (inverse), linear (x), "
                          "exp_decay (1-exp(-alpha*x)), compl_pwr (1-(1-x)^p).",
                          StringValue("deng"),
                          MakeStringAccessor(&NrRlHandoverRewardApp::m_rewardGoodputShape),
                          MakeStringChecker())
            .AddAttribute("RewardGoodputAlpha",
                          "exp_decay rate: R_G = 1-exp(-alpha*normG). "
                          "Gradient alpha*exp(-alpha*x), maximal at low x.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_rewardGoodputAlpha),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("RewardGoodputP",
                          "compl_pwr exponent (0<p<1): R_G = 1-(1-x)^p. "
                          "Gradient p*(1-x)^(p-1), grows near x=1 (optimum-focused).",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_rewardGoodputP),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("BetaHandover",
                          "Handover sensitivity for Deng-style inverse penalty. "
                          "Higher = more HO-averse. R_H = 1/(1+beta*I_ho).",
                          DoubleValue(60.0),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_betaHandover),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("HandoverHangoverLength",
                          "Number of steps the handover reward penalty persists after "
                          "a handover (decaying window). Inside the window "
                          "the tax weight decays exponentially: w = exp(-age/N), so the "
                          "first hangover step carries the full tax (w=1) and the last "
                          "carries exp(-(N-1)/N). Default 4.",
                          UintegerValue(4),
                          MakeUintegerAccessor(&NrRlHandoverRewardApp::m_handoverHangoverLength),
                          MakeUintegerChecker<uint32_t>(1, 20))
            .AddAttribute("PingPongBetaMultiplier",
                          "Multiplier applied to betaHandover on ping-pong "
                          "(A->B->A) patterns. Default 5.0.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_pingPongBetaMultiplier),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("PingPongWindowMs",
                          "Maximum age of an A->B->A pattern for it to count as a "
                          "ping-pong: the two handovers back to the same cell must be at "
                          "most this far apart. 0 disables the age limit, in which case "
                          "any A->B->A sequence is flagged regardless of elapsed time "
                          "(the behaviour before 2026-09-20).",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlHandoverRewardApp::m_pingPongWindowMs),
                          MakeUintegerChecker<uint32_t>(0, 600000))
            .AddAttribute("RewardComposition",
                          "Reward combination: 'additive' = alpha*R_G + (1-alpha)*R_H "
                          "(constant baseline on calm steps, handover tax capped at "
                          "1-alpha per hangover step); 'multiplicative' = R_G * R_H "
                          "(no baseline, tax scales with the goodput being sacrificed).",
                          StringValue("additive"),
                          MakeStringAccessor(&NrRlHandoverRewardApp::m_rewardComposition),
                          MakeStringChecker())
            .AddAttribute("GoodputRefBps",
                          "Fixed upper-bound reference (bps) for normGoodput "
                          "normalization. Set to the OnOff data rate.",
                          DoubleValue(40e6),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_goodputRefBps),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("UseEwmaReference",
                          "If true, use the old EWMA-adaptive reference instead "
                          "of the fixed upper-bound reference.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrRlHandoverRewardApp::m_useEwmaReference),
                          MakeBooleanChecker())
            .AddAttribute("EwmaAlpha",
                          "EWMA smoothing factor for goodput-based adaptive reference. "
                          "0.2 = 5-step (~1s) window. Only used when UseEwmaReference=true.",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&NrRlHandoverRewardApp::m_ewmaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("CalculationInterval",
                          "Interval (s) between reward calculations.",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&NrRlHandoverRewardApp::m_calculationInterval),
                          MakeTimeChecker());
    return tid;
}

void
NrRlHandoverRewardApp::RegisterCallbacks()
{
    // Cache UAV node ID
    if (g_uavContainer.GetN() > 0)
    {
        m_uavNodeId = g_uavContainer.Get(0)->GetId();
        m_tbsHistory.reserve(512);
    }

    // --- TBS trace (direction-dependent, logging only) ---
    if (m_uavNodeId > 0)
    {
        std::string tbSizeTrace = (g_flowDirection == "dl")
            ? "ReportDownlinkTbSize"
            : "ReportUplinkTbSize";
        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(m_uavNodeId) +
                "/DeviceList/*/$ns3::NrUeNetDevice/"
                "ComponentCarrierMapUe/*/NrUePhy/" +
                tbSizeTrace,
            MakeCallback(&NrRlHandoverRewardApp::ObserveTbs, this));
    }

    // --- Sink Rx trace on receiving node ---
    NS_LOG_INFO("Connecting sink Rx on receiver node " << g_receiverNodeId);
    std::string rxPath = "/NodeList/" + std::to_string(g_receiverNodeId) +
                         "/ApplicationList/*/Rx";
    Config::ConnectWithoutContext(rxPath,
                                  MakeCallback(&NrRlHandoverRewardApp::ObserveSinkRx, this));

    // --- Handover tracking (for I_ho per-step indicator + ping-pong) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/NrUeRrc/HandoverEndOk",
        MakeCallback(&NrRlHandoverRewardApp::ObserveHandover, this));

    // Schedule first reward computation
    Simulator::Schedule(m_calculationInterval,
                        &NrRlHandoverRewardApp::SendReward,
                        this);

    NS_LOG_INFO("NrRlHandoverRewardApp registered: interval="
                << m_calculationInterval.GetMilliSeconds() << "ms"
                << ", alphaGoodput=" << m_alphaGoodput
                << ", betaHandover=" << m_betaHandover
                << ", hangover=" << m_handoverHangoverLength
                << ", pingPongMult=" << m_pingPongBetaMultiplier
                << ", pingPongWindowMs=" << m_pingPongWindowMs
                << ", rewardComposition=" << m_rewardComposition
                << ", flowDirection=" << g_flowDirection);
}

void
NrRlHandoverRewardApp::ObserveSinkRx(Ptr<const Packet> packet, const Address& from)
{
    m_sinkBytesReceived += packet->GetSize();
}

void
NrRlHandoverRewardApp::ObserveTbs(uint64_t imsi, uint64_t tbSize)
{
    m_tbsHistory.push_back(static_cast<int32_t>(tbSize));
}

void
NrRlHandoverRewardApp::ObserveHandover(const uint64_t imsi,
                                         const uint16_t cellId,
                                         const uint16_t rnti)
{
    // Set hangover counter (will decay each step in SendReward)
    m_handoverHangoverSteps = m_handoverHangoverLength;

    // Detect ping-pong: A->B->A pattern
    // handoverHistory[0] = oldest, [2] = most recent; the times are shifted with
    // the history so the ping-pong flag can be bounded in age.
    // Shift history left by one, append new cellId
    m_handoverHistory[0] = m_handoverHistory[1];
    m_handoverHistory[1] = m_handoverHistory[2];
    m_handoverHistory[2] = cellId;
    m_handoverTimes[0] = m_handoverTimes[1];
    m_handoverTimes[1] = m_handoverTimes[2];
    m_handoverTimes[2] = Simulator::Now().GetSeconds();
}

void
NrRlHandoverRewardApp::SendReward()
{
    // --- 1. Compute goodput (bps) from sink bytes this step ---
    double intervalSec = m_calculationInterval.GetSeconds();
    double goodputBps = 0.0;
    if (intervalSec > 0.0)
    {
        goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / intervalSec;
    }

    // --- 2. Reference for normalization ---
    // Fixed upper-bound (default): normG = goodput / refBps.
    // EWMA-adaptive (optional, old behavior): ref = EWMA*1.1, min = EWMA*0.3.
    double dynamicRefBps = m_goodputRefBps;
    double dynamicMinBps = 0.0;

    if (m_useEwmaReference)
    {
        if (m_ewmaGoodput > 0.0)
        {
            dynamicRefBps = m_ewmaGoodput * 1.1;
            dynamicMinBps = m_ewmaGoodput * 0.3;
        }
        else
        {
            dynamicRefBps = goodputBps;
            dynamicMinBps = goodputBps * 0.3;
        }

        // Update EWMA (after computing ref, so ref uses last step's EWMA)
        if (m_ewmaGoodput <= 0.0)
        {
            m_ewmaGoodput = goodputBps;
        }
        else
        {
            m_ewmaGoodput = m_ewmaAlpha * goodputBps + (1.0 - m_ewmaAlpha) * m_ewmaGoodput;
        }
    }

    // --- 3. normGoodput (raw) — clamped to [0, 1] for Deng inverse ---
    double normGoodputRaw = 0.0;
    if (m_useEwmaReference)
    {
        if (dynamicRefBps > 0.0 && dynamicMinBps > 0.0)
        {
            if (goodputBps >= dynamicRefBps)
            {
                normGoodputRaw = 1.0;
            }
            else if (goodputBps >= dynamicMinBps)
            {
                normGoodputRaw = (goodputBps - dynamicMinBps) / (dynamicRefBps - dynamicMinBps);
            }
            else
            {
                normGoodputRaw = (goodputBps - dynamicMinBps) / dynamicMinBps;
            }
        }
    }
    else if (m_goodputRefBps > 0.0)
    {
        normGoodputRaw = goodputBps / m_goodputRefBps;
    }
    double normGoodput = std::max(0.0, std::min(1.0, normGoodputRaw));

    // --- 4. R_G — goodput reward, functional shape selector ---
    //   deng:       1/(1+beta*(1-normG))  (inverse, beta_G steepness)
    //   linear:     normG
    //   exp_decay:  1-exp(-alpha*normG)  (concave, maximal gradient at low x)
    //   compl_pwr:  1-(1-normG)^p  (convex, gradient grows near x=1)
    double gap = 1.0 - normGoodput;
    double R_G = 0.0;
    if (m_rewardGoodputShape == "linear")
    {
        R_G = normGoodput;
    }
    else if (m_rewardGoodputShape == "exp_decay")
    {
        R_G = 1.0 - std::exp(-m_rewardGoodputAlpha * normGoodput);
    }
    else if (m_rewardGoodputShape == "compl_pwr")
    {
        double g = std::max(gap, 1e-6); // avoid 0^p singularity
        R_G = 1.0 - std::pow(g, m_rewardGoodputP);
    }
    else // "deng" (default)
    {
        R_G = 1.0 / (1.0 + m_betaGoodput * gap);
    }

    // --- 5. Handover indicator with decaying hangover ---
    double hoWeight = 0.0;
    // Sampled before the decrement so I_ho covers the full window (last step too).
    bool hoActive = (m_handoverHangoverSteps > 0);
    if (m_handoverHangoverSteps > 0)
    {
        uint32_t age = m_handoverHangoverLength - m_handoverHangoverSteps;
        hoWeight = std::exp(-static_cast<double>(age) / m_handoverHangoverLength);
        m_handoverHangoverSteps--;
    }

    // --- 6. Ping-pong detection: A->B->A pattern ---
    // handoverHistory[0] and [2] being equal means we bounced back. With a
    // non-zero window the pair must also be recent: two handovers back to the
    // same cell minutes apart are a legitimate return, not a ping-pong.
    bool pingPong = false;
    if (m_handoverHistory[0] > 0 && m_handoverHistory[2] > 0 &&
        m_handoverHistory[0] == m_handoverHistory[2] &&
        m_handoverHistory[0] != m_handoverHistory[1])
    {
        bool withinWindow = true;
        if (m_pingPongWindowMs > 0)
        {
            const double ageMs = (m_handoverTimes[2] - m_handoverTimes[0]) * 1000.0;
            withinWindow = (ageMs <= static_cast<double>(m_pingPongWindowMs));
        }
        pingPong = withinWindow;
    }

    // --- 7. R_H — Deng inverse handover reward with ping-pong multiplier ---
    double effectiveBeta = m_betaHandover;
    if (pingPong)
    {
        effectiveBeta *= m_pingPongBetaMultiplier;
    }
    double I_ho = hoActive ? 1.0 : 0.0;
    double R_H = 1.0 / (1.0 + effectiveBeta * hoWeight);

    // --- 8. Combined reward ---
    // additive: R = alpha*R_G + (1-alpha)*R_H — a constant (1-alpha)
    //   baseline on calm steps (policy-invariant in fixed-length episodes) and a
    //   handover tax capped at (1-alpha) per hangover step.
    // multiplicative:     R = R_G * R_H — no baseline; the handover tax scales
    //   with the goodput being sacrificed (up to the full R_G per hangover step);
    //   the dead-link floor is R_G(0) = 1/(1+beta_G).
    double reward = (m_rewardComposition == "multiplicative")
                        ? R_G * R_H
                        : m_alphaGoodput * R_G + (1.0 - m_alphaGoodput) * R_H;

    // --- 9. TBS throughput (logging only) ---
    if (!m_tbsHistory.empty())
    {
        m_tbsHistory.clear();
    }

    // --- 10. Log to CSV ---
    if (g_logging)
    {
        std::ofstream rwdFile(g_outputDir + "rl_reward.csv", std::ios_base::app);
        rwdFile << Simulator::Now().GetSeconds() << ","
                << (goodputBps / 1e6) << ","          // goodput (Mbps)
                << (dynamicRefBps / 1e6) << ","       // dynamicRef (Mbps)
                << (dynamicMinBps / 1e6) << ","       // dynamicMin (Mbps)
                << normGoodputRaw << ","              // normGoodput (raw)
                << normGoodput << ","                 // normGoodput (clamped)
                << R_G << ","                          // goodput reward term
                << I_ho << ","                         // handover indicator
                << R_H << ","                          // handover reward term
                << pingPong << ","                     // ping-pong flag
                << reward << std::endl;                // total reward
    }

    NS_LOG_INFO("Reward: goodput=" << goodputBps << "bps"
                << " normG=" << normGoodput
                << " R_G=" << R_G
                << " I_ho=" << I_ho
                << " R_H=" << R_H
                << " pingPong=" << pingPong
                << " reward=" << reward);

    // --- 11. Send reward to agent ---
    auto rewardContainer = MakeDictBoxContainer<double>(1, "reward", reward);
    Send(rewardContainer);

    // Reset step accumulator
    m_sinkBytesReceived = 0;

    // Schedule next reward computation
    Simulator::Schedule(m_calculationInterval, &NrRlHandoverRewardApp::SendReward, this);
}

} // namespace ns3
