#include "ns3/action-application.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-enb-rrc.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Action application for the HARL TCP handover RL agent.
 *
 * Runs on the UAV node. Receives handover action dicts from the agent app,
 * validates preconditions, and executes the handover via g_lteHelper.
 *
 * Validation checks:
 * 1. Is the UE in CONNECTED_NORMALLY state?
 * 2. Was no-op action (0) taken?
 * 3. Is target cell same as current cell?
 * 4. Does the serving eNB have the UE's UeManager?
 */
class HarlTcpHandoverActionApp : public ActionApplication
{
  public:
    HarlTcpHandoverActionApp();
    ~HarlTcpHandoverActionApp() override;

    static TypeId GetTypeId();

    /**
     * Execute a handover action.
     *
     * @param action Dict containing "newCellId" (DiscreteContainer)
     */
    void ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action) override;

  private:
    uint32_t m_numBs;                    ///< Number of base stations
    std::string m_handoverAlgorithm;     ///< "agent" or "a3" or "noop"
    double m_handoverMargin{3.0};        ///< RSRP margin (3GPP range), -999 disables gating
};

} // namespace ns3
