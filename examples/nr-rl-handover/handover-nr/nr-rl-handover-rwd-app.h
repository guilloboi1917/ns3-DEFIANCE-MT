#include "ns3/reward-application.h"

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace ns3
{

class Address;
class Packet;

/**
 * @ingroup defiance
 * @brief Reward application for the NR RL handover agent (Deng-style formulation).
 *
 * Runs on the UAV node. Measures DL goodput via the PacketSink Rx trace.
 * Reward follows Deng et al. (Paper 5) weighted KPI formulation:
 *
 *   R = alpha * R_G + (1 - alpha) * R_H
 *
 *   R_G = 1 / (1 + beta_G * max(0, 1 - normGoodput))
 *   R_H = 1 / (1 + beta_H * I_ho)
 *
 * normGoodput uses a fixed upper-bound reference (m_goodputRefBps, the OnOff
 * data rate) by default. The old EWMA-adaptive reference is retained behind
 * m_useEwmaReference for easy reversal.
 *
 * Two mechanisms prevent excessive handovers:
 *
 * 1. Handover hangover: I_ho stays true for N steps after each handover
 *    (m_handoverHangoverLength, default 4). This makes the penalty persist
 *    beyond the exact step where the handover fired.
 *
 * 2. Ping-pong penalty: If the agent bounces back to the cell it just left
 *    (A->B->A pattern), beta_H is multiplied by m_pingPongBetaMultiplier
 *    (default 3.0) for that step only.
 */
class NrRlHandoverRewardApp : public RewardApplication
{
  public:
    NrRlHandoverRewardApp();
    ~NrRlHandoverRewardApp() override;

    static TypeId GetTypeId();

    void RegisterCallbacks() override;
    void SendReward();

    /** Callback: track every packet received by the PacketSink. */
    void ObserveSinkRx(Ptr<const Packet> packet, const Address& from);

    /** Callback: track transport block size (DL or UL depending on flowDirection). */
    void ObserveTbs(uint64_t imsi, uint64_t tbSize);

    /** Callback: track handover events for I_ho indicator and ping-pong detection. */
    void ObserveHandover(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti);

  private:
    // --- Deng-style reward parameters ---
    double m_alphaGoodput{0.8};          ///< Weight for goodput term (0..1)
    double m_betaGoodput{5.0};           ///< Goodput sensitivity for (1+beta*(1-normG))^-1 (deng shape only)
    double m_betaHandover{60.0};         ///< Handover sensitivity for (1+beta*I_ho)^-1
    std::string m_rewardComposition{"additive"}; ///< additive | multiplicative

    // --- R_G functional shape (deng | linear | exp_decay | compl_pwr) ---
    std::string m_rewardGoodputShape{"deng"}; ///< R_G mapping; deng = inverse, linear = x,
                                              ///< exp_decay = 1-exp(-alpha*x), compl_pwr = 1-(1-x)^p
    double m_rewardGoodputAlpha{3.0};    ///< exp_decay rate (gradient alpha*exp(-alpha*x))
    double m_rewardGoodputP{0.4};        ///< compl_pwr exponent (0<p<1; gradient grows near x=1)

    // --- Goodput normalization reference ---
    double m_goodputRefBps{40e6};        ///< Fixed reward reference (near achievable throughput)
    bool m_useEwmaReference{false};      ///< If true, use EWMA-adaptive reference (old behavior)
    double m_ewmaGoodput{0.0};           ///< EWMA of actual goodput (bps, EWMA path only)
    double m_ewmaAlpha{0.2};             ///< EWMA smoothing factor (EWMA path only)

    // --- Goodput tracking ---
    uint64_t m_sinkBytesReceived{0};     ///< Bytes received this step

    // --- Handover hangover ---
    uint32_t m_handoverHangoverLength{4}; ///< Steps I_ho stays true after a handover
    uint32_t m_handoverHangoverSteps{0};  ///< Remaining hangover steps

    // --- Ping-pong detection ---
    double m_pingPongBetaMultiplier{5.0}; ///< Multiply beta_H by this on ping-pong (attribute default 5.0)
    uint32_t m_handoverHistory[3]{};      ///< Last 3 handover target cell IDs

    // --- Handover-rate penalty (windowed signaling budget) ---
    // On each handover event, count handovers within the last
    // m_rateWindowMs; if the count exceeds m_rateBudget, multiply that step's
    // reward by max(1 - lambda*excess, floor). The threshold structure taxes
    // sustained churn (the attractor) while leaving sparse handovers free.
    bool m_ratePenaltyEnabled{false}; ///< Enable the windowed rate penalty
    uint32_t m_rateWindowMs{10000};   ///< Sliding window for the rate count (ms)
    uint32_t m_rateBudget{2};         ///< Free handovers per window before penalizing
    double m_ratePenaltyLambda{0.2};  ///< Marginal penalty per excess handover
    bool m_handoverThisStep{false};   ///< A handover fired in the current step
    std::deque<double> m_handoverTimes; ///< Handover timestamps in the sliding window (s)

    // --- Timing ---
    Time m_calculationInterval{MilliSeconds(200)}; ///< Reward step interval

    // --- TBS tracking (logging only) ---
    std::vector<int32_t> m_tbsHistory;   ///< TBS samples accumulated over step

    // --- Node IDs ---
    uint32_t m_uavNodeId{0};             ///< Node ID of the UAV
};

} // namespace ns3
