#include "ns3/reward-application.h"

#include <cstdint>
#include <vector>

namespace ns3
{

class Address;
class Packet;

/**
 * @ingroup defiance
 * @brief Reward application for the NR RL handover agent.
 *
 * Runs on the UAV node. Measures UL or DL throughput via the PacketSink Rx trace
 * depending on --flowDirection. The reward is:
 *
 *   reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm + tbsBonus
 *
 * where:
 *   normGoodput:
 *      1.0                                    if goodput >= dynamicRef
 *      (goodput - dynamicMin) / (ref - min)   if dynamicMin <= goodput < dynamicRef
 *      (goodput - dynamicMin) / dynamicMin    if goodput < dynamicMin (negative)
 *
 *   dynamicRef = m_ewmaGoodput x 1.1   (EWMA of actual goodput, ~10% margin)
 *   dynamicMin = m_ewmaGoodput x 0.3   (30% of EWMA goodput)
 *
 *   This creates a lagging reference: after a good handover, goodput rises
 *   above the EWMA, pushing normGoodput above 1.0 for several steps until
 *   the EWMA catches up. This transient overshoot is the improvement signal.
 *
 *   tbsBonus = small bonus for being on a cell with high PHY potential
 *              (TBS throughput, capped at 0.3)
 *
 *   rttPenalty:
 *      0.0                                    if rtt <= delayMinRtt
 *      (rtt - delayMinRtt) / (maxRtt - delayMinRtt)  if delayMinRtt < rtt < maxRtt
 *      1.0                                    if rtt >= maxAcceptableRtt
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

    /** Callback: track current RTT from TCP socket. */
    void ObserveRtt(Time oldRtt, Time newRtt);

    /** Callback: track UL/DL PHY transmission stats (TBS) for adaptive reference rate. */
    void ObserveUlTbs(uint64_t imsi, uint64_t tbSize);

  private:
    double m_handoverPenalty{0.01};                 ///< Penalty per handover (norm units) — currently unused
    double m_referenceRateBps{5000000.0};           ///< Reference rate for throughput normalization (5 Mbps)
    double m_minimumAcceptableGoodputBps{2500000.0}; ///< Min acceptable goodput (R_min)
    double m_delayMinRttMs{55.0};                    ///< Lower bound RTT (ms) — zero delay penalty below this
    double m_maxAcceptableRttMs{100.0};              ///< Upper bound RTT (ms) — penalty clamped at 1 above this
    double m_tcpFailurePenalty{0.5};                 ///< Penalty per step when TCP is dead
    double m_rlfPenalty{1.0};                        ///< One-time penalty when RLF is detected
    Time m_calculationInterval{MilliSeconds(200)};   ///< Reward step interval (aligned with ReportUeMeasurements)
    uint32_t m_remoteHostNodeId{0};                  ///< Node ID of remote host for trace
    uint32_t m_lastTotalHandovers{0};                ///< Handover count at last reward step (tracking only)
    uint64_t m_sinkBytesReceived{0};                 ///< Bytes received this step
    int32_t m_currentRttMs{30};                      ///< Latest RTT sample (ms)

    // EWMA-based adaptive reference
    double m_ewmaGoodput{0.0};                       ///< EWMA of actual goodput (bps)
    double m_ewmaAlpha{0.2};                         ///< EWMA smoothing factor (5-step ~1s window)
    bool m_tcpPenaltyApplied{false};                 ///< True once the one-shot TCP penalty has been applied

    // TBS bonus (cell quality signal)
    std::vector<int32_t> m_tbsHistory;               ///< TBS samples accumulated over current step
    double m_tbsBonusWeight{0.003};                  ///< Bonus per Mbps of TBS throughput (capped at 0.3)
    uint32_t m_uavNodeId{0};                         ///< Node ID of the UAV (for trace connection)

    // Serving cell tracking for RSRP delta bonus
    double m_previousServingRsrp{0.0};               ///< Last known serving cell RSRP (dBm)
};

} // namespace ns3
