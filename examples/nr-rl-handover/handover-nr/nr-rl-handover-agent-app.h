#include "ns3/agent-application.h"

#include <cstdint>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Agent application for the NR RL handover agent (Top-N design).
 *
 * Runs on the UAV node. Receives flat Box observations (32 dims) from
 * NrRlHandoverObservationApp, calls InferAction() -> Python/RLlib, and
 * dispatches actions (Discrete(4)) to NrRlHandoverActionApp.
 *
 * Observation space: Flat Box(double, shape=(32,))
 *   [0-1] serving_rsrp / serving_rsrq
 *   [2-4] slot_rsrp[0..2], [5-7] rsrp_delta[0..2]
 *   [8] dl_sinr, [9] time_since_ho, [10] norm_goodput, [11] ho_count_10s
 *   [12-14] ul_sinr / ul_rb_util / ul_sched_ue
 *   [15-19] time-delta block, [20-22] d_slot_rsrp[0..2]
 *   [23-25] heading_x/y/z, [26-28] pos/ISD, [29-31] pos2s/ISD
 *
 * Action space: Discrete(4)
 *   0 = no-op, 1..3 = handover to the top-3 ranked NON-serving cell
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

    /** Initial (reset) observation — flat 32-dim Box wrapped in Dict. */
    Ptr<OpenGymDictContainer> GetResetObservation() const;
    /** Initial (reset) reward. */
    float GetResetReward();

  private:
    uint32_t m_numBs{9}; ///< Number of base stations/cells
    uint32_t m_topN{3};  ///< Ranked NON-serving cells (Top-N fixed at 3)
};

} // namespace ns3
