#include "ns3/ipv4.h"
#include "ns3/reward-application.h"

#include <cstdint>

namespace ns3
{
class LteHelper;

/**
 * @ingroup defiance
 * @brief Child class of RewardApplication that sends rewards based on throughput
 *        with a penalty for handovers.
 */
class UavHandoverRewardApplication : public RewardApplication
{
  public:
    UavHandoverRewardApplication() {};
    ~UavHandoverRewardApplication() override {};
    static TypeId GetTypeId();
    /// Send the reward to the agent
    void SendReward();
    void RegisterCallbacks() override;

  private:
    Time m_calculationInterval{MilliSeconds(1000)};
    Time m_simTime{Seconds(100)};
    Ptr<LteHelper> m_lteHelper;
    double m_handoverPenalty{1000.0};
    uint32_t m_lastTotalHandovers{0};
};
} // namespace ns3
