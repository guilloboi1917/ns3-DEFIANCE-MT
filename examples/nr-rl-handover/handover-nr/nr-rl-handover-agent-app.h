#include "ns3/agent-application.h"

#include <cstdint>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Agent application for the NR RL handover agent.
 *
 * Runs on the UAV node. Receives observations from NrRlHandoverObservationApp,
 * computes handover actions, and dispatches them via NrRlHandoverActionApp.
 *
 * Observation space (dict):
 *   - rsrps: Box(double, shape=(numBs,), range [-160, -40])
 *   - rsrqs: Box(double, shape=(numBs,), range [-100, -3])
 *   - sinr: Box(double, shape=(1,), range [-40, 50])
 *   - rsrpDelta: Box(double, shape=(numBs,), range [-60, 60])
 *   - rsrqDelta: Box(double, shape=(numBs,), range [-60, 60])
 *   - sinrDelta: Box(double, shape=(1,), range [-20, 20])
 *   - tbs: Box(int32, shape=(1,), range [0, 100000])
 *   - action_mask: Box(double, shape=(numBs+1,), range [0, 1])
 *
 * Action space: Discrete(numBs + 1)
 *   - 0 = No-op
 *   - 1..numBs = Target cell ID
 */
class NrRlHandoverAgentApp : public AgentApplication
{
  public:
    NrRlHandoverAgentApp();
    ~NrRlHandoverAgentApp() override;

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
    Time m_lastInferredActionTime{Seconds(0)}; ///< Last time an action was inferred
};

} // namespace ns3
