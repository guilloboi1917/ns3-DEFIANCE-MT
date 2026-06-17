#include "ns3/agent-application.h"
#include "ns3/lte-helper.h"

#include <cstdint>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Agent application for the HARL TCP handover RL agent.
 *
 * Runs on the UAV node. Receives observations from HarlTcpHandoverObservationApp,
 * computes handover actions, and dispatches them via HarlTcpHandoverActionApp.
 *
 * Observation space (dict):
 *   - rsrps: Box(int32, shape=(numBs,), range [-1, 97])
 *   - sinrs: Box(double, shape=(numBs,), range [-40, 50])
 *   - cellId: Discrete(numBs+1)
 *   - rrcState: Discrete(14)
 *   - cwnd: Box(int32, shape=(1,), range [0, 65535])
 *   - rtt: Box(int32, shape=(1,), range [0, 10000])
 *   - deliveryRate: Box(int32, shape=(1,), range [0, 100000000])
 *
 * Action space: Discrete(numBs + 1)
 *   - 0 = No-op
 *   - 1..numBs = Target cell ID
 */
class HarlTcpHandoverAgentApp : public AgentApplication
{
  public:
    HarlTcpHandoverAgentApp();
    ~HarlTcpHandoverAgentApp() override;

    static TypeId GetTypeId();

    void Setup() override;
    void OnRecvObs(uint id) override;
    void OnRecvReward(uint id) override;
    void InitiateAction(Ptr<OpenGymDataContainer> action) override;

    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymSpace> GetActionSpace() override;

    /** Initial (reset) observation. */
    Ptr<OpenGymDictContainer> GetResetObservation() const;
    /** Initial (reset) reward. */
    float GetResetReward();

  private:
    uint32_t m_numBs;            ///< Number of base stations
    uint32_t m_numUes;           ///< Number of UEs
    uint32_t m_stepTime;         ///< Step interval in ms
    uint32_t m_maxCwnd;          ///< Max cwnd for observation scaling (default 65535)
    uint32_t m_maxRate;          ///< Max delivery rate for obs scaling (default 100 Mbps)
    Time m_lastInferredActionTime{Seconds(0)}; ///< Last time an action was inferred
};

} // namespace ns3
