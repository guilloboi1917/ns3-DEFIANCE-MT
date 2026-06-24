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
 *   reward = normGoodput - rttPenalty - tcpPenalty - rlfTerm
 *
 * where:
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
 * Throughput reward has a dead zone below minimumAcceptableGoodput (negative reward).
 * RTT penalty uses a self-normalized ramp with an optional lower bound (no weight multiplier).
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
    double m_handoverPenalty{0.01};                 ///< Penalty per handover (norm units) — currently unused
    double m_referenceRateBps{5000000.0};           ///< Reference UL rate for throughput normalization (5 Mbps)
    double m_minimumAcceptableGoodputBps{2500000.0}; ///< Min acceptable goodput (R_min) — below this, reward is negative
    double m_delayMinRttMs{55.0};                    ///< Lower bound RTT (ms) — zero delay penalty below this
    double m_maxAcceptableRttMs{100.0};              ///< Upper bound RTT (ms) — penalty clamped at 1 above this
    double m_tcpFailurePenalty{0.5};                 ///< Penalty per step when TCP is dead
    double m_rlfPenalty{1.0};                        ///< One-time penalty when RLF is detected
    Time m_calculationInterval{MilliSeconds(480)};   ///< Reward step interval (aligned with MS480)
    uint32_t m_remoteHostNodeId{0};                  ///< Node ID of remote host for trace
    uint32_t m_lastTotalHandovers{0};                ///< Handover count at last reward step (tracking only)
    uint64_t m_sinkBytesReceived{0};                 ///< Bytes received this step
    int32_t m_currentRttMs{30};                      ///< Latest RTT sample (ms)
};

} // namespace ns3
