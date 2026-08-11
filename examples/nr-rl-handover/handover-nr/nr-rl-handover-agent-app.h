#include "ns3/agent-application.h"

#include <cstdint>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Agent application for the NR RL handover agent (Top-N design).
 *
 * Runs on the UAV node. Receives flat Box observations (29 dims) from
 * NrRlHandoverObservationApp, calls InferAction() -> Python/RLlib, and
 * dispatches actions (Discrete(6)) to NrRlHandoverActionApp.
 *
 * Observation space: Flat Box(double, shape=(29,))
 *   [0] serving_rsrp, [1] serving_rsrq,
 *   [2..6] slot_rsrp[0..4], [7..11] slot_rsrq[0..4],
 *   [12..16] rsrp_delta[0..4], [17..21] rsrq_delta[0..4],
 *   [22] sinr, [23] heading_x, [24] heading_z, [25] speed,
 *   [26] tbs, [27] time_since_ho, [28] norm_goodput
 *
 * Action space: Discrete(6)
 *   0 = no-op, 1..5 = handover to top-5 ranked cell (by RSRP)
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

    /**
     * Send the act-app's executed action index to Python via the existing
     * info channel (extraInfo -> envStateMsg.info). 0 = stay (noop or blocked).
     */
    std::map<std::string, std::string> GetExtraInfo() override;

    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymSpace> GetActionSpace() override;

    /** Initial (reset) observation — flat 28-dim Box wrapped in Dict. */
    Ptr<OpenGymDictContainer> GetResetObservation() const;
    /** Initial (reset) reward. */
    float GetResetReward();

  private:
    uint32_t m_numBs{9}; ///< Number of base stations/cells
    uint32_t m_topN{5};  ///< Number of ranked cells in Top-N action space
};

} // namespace ns3
