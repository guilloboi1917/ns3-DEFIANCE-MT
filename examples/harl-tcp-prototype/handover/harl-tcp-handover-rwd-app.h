#include "ns3/reward-application.h"

#include <cstdint>

namespace ns3
{

class Address;
class Packet;

/**
 * @ingroup defiance
 * @brief Reward application for the HARL TCP handover RL agent.
 *
 * Runs on the UAV node. Measures UL throughput via the PacketSink Rx trace
 * on the remote host (the UAV is the TCP sender). The reward is:
 *
 *   reward = throughput_norm
 *          - handoverPenalty * handoverCount
 *          - rttPenaltyWeight * rttInflation
 *
 * where:
 *   throughput_norm = min(sinkGoodput_bps / referenceRate_bps, 1.0)
 *   rttInflation    = max(0, currentRtt_ms - minRtt_ms) / minRtt_ms
 *
 * The RTT penalty discourages bufferbloat: when the LTE UL is congested,
 * packets queue at the RLC layer and RTT inflates even without packet loss.
 * The minRtt floor is ~10 ms (PGW-Server p2p link) + ~20 ms LTE processing.
 */
class HarlTcpHandoverRewardApp : public RewardApplication
{
  public:
    HarlTcpHandoverRewardApp();
    ~HarlTcpHandoverRewardApp() override;

    static TypeId GetTypeId();

    void RegisterCallbacks() override;
    void SendReward();

    /** Callback: track every packet received by the remote PacketSink. */
    void ObserveSinkRx(Ptr<const Packet> packet, const Address& from);

    /** Callback: track current RTT from TCP socket. */
    void ObserveRtt(Time oldRtt, Time newRtt);

  private:
    Time m_calculationInterval{MilliSeconds(100)}; ///< Step interval for reward
    uint32_t m_remoteHostNodeId{0};                 ///< Node ID of remote host for trace
    double m_handoverPenalty{0.01};                 ///< Penalty per handover (norm units)
    double m_rttPenaltyWeight{0.05};                ///< Weight of RTT inflation penalty
    double m_minRttMs{30.0};                        ///< Baseline RTT (ms) — 10ms PGW + ~20ms LTE
    double m_referenceRateBps{30000000.0};          ///< Reference UL rate for normalization (30 Mbps)
    double m_tcpFailurePenalty{0.5};                ///< Penalty per step when TCP is dead
    double m_rlfPenalty{1.0};                       ///< One-time penalty when RLF is detected
    double m_pingPongPenalty{0.05};                 ///< Extra penalty per handover within pingPongInterval
    Time m_pingPongInterval{Seconds(1)};            ///< Min time between handovers to avoid ping-pong penalty
    Time m_lastHandoverTime{Seconds(0)};            ///< Sim time of last detected handover
    uint32_t m_lastTotalHandovers{0};               ///< Handover count at last reward step
    uint64_t m_sinkBytesReceived{0};                ///< Bytes received this step
    int32_t m_currentRttMs{30};                     ///< Latest RTT sample (ms)
};

} // namespace ns3
