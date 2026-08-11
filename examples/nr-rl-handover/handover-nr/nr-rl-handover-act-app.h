#include "ns3/action-application.h"
#include "ns3/nr-module.h"

#include <cstdint>
#include <string>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Action application for the NR RL handover agent (Top-N design).
 *
 * Runs on the UAV node. Receives action dicts from the agent app containing
 * an "actionIndex" (0..5), maps it to a target cellId via the global
 * g_topNCells array (populated by the ObsApp each step), validates
 * preconditions, and executes the handover via g_nrHelper.
 *
 * Validation checks:
 * 1. Is the UE in CONNECTED_NORMALLY state?
 * 2. Is a handover already in progress?
 * 3. Is the action a no-op (actionIndex == 0)?
 * 4. Is the target cell valid and different from current cell?
 */
class NrRlHandoverActionApp : public ActionApplication
{
  public:
    NrRlHandoverActionApp();
    ~NrRlHandoverActionApp() override;

    static TypeId GetTypeId();

    /**
     * Execute a handover action.
     *
     * @param action Dict containing "actionIndex" (DiscreteContainer, 0..topN)
     */
    void ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action) override;

  private:
    uint32_t m_numBs{9};                    ///< Number of base stations
    uint32_t m_topN{5};                     ///< Number of ranked cells
    std::string m_handoverAlgorithm;       ///< "agent" or "a3" or "noop"
    uint32_t m_handoverDebounceMs{0};      ///< Min interval between executed handovers (ms); 0 = off
    Time m_lastHandoverTime{Seconds(-1000.0)}; ///< Last executed handover time; never blocks the first
};

} // namespace ns3
