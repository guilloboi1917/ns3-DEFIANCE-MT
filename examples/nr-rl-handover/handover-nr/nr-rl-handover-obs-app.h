#include "ns3/observation-application.h"
#include "ns3/spectrum-value.h"

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Observation application for the NR RL handover agent.
 *
 * Runs on the UAV node and collects:
 * - RSRP/RSRQ per cell from ReportUeMeasurements (200ms, already averaged by UE PHY)
 * - UL SINR per cell from gNB-side UL SINR trace (EWMA-smoothed)
 * - Current serving cell ID
 * - Transport block size (TBS) from UL PHY transmission
 * - RTT
 *
 * Observations are built and sent immediately after each ReportUeMeasurements
 * batch completes (detected by simulation time advancing).
 *
 * Observation dict keys:
 *   rsrps, rsrqs, sinr, rsrpDelta, rsrqDelta, sinrDelta, tbs, action_mask
 */
class NrRlHandoverObservationApp : public ObservationApplication
{
  public:
    NrRlHandoverObservationApp();
    ~NrRlHandoverObservationApp() override;

    static TypeId GetTypeId();
    void DoInitialize() override;

    /** Register Config callbacks for measurement collection. */
    void RegisterCallbacks() override;

    // --- Callback handlers ---

    /** Handle ReportUeRsrpRsrq from the UAV PHY (per-cell RSRP/RSRQ in dBm). */
    void ObserveUeRsrpRsrq(uint16_t rnti,
                           uint16_t cellId,
                           double rsrp,
                           double rsrq,
                           bool isServingCell,
                           uint8_t componentCarrierId);

    /** Handle UL SINR (SRS-based) from gNB PHY (UlSinrTrace: SpectrumValue per RB). */
    void ObserveUlSinr(uint64_t imsi,
                       SpectrumValue& sinrSpectrum,
                       SpectrumValue& interferenceSpectrum);

    /**
     * Handle DL data SINR from UE PHY (DlDataSinr: per-slot linear value).
     * Used when flowDirection == "dl".
     */
    void ObserveDlSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint16_t bwpId);

    /** Handle RTT changes. */
    void ObserveRtt(Time oldRtt, Time newRtt);

    /** Handle UL transmission stats (TBS). */
    void ObserveUlTbs(uint64_t imsi, uint64_t tbSize);

  private:
    uint32_t m_numBs;            ///< Number of gNBs/cells in the scenario
    uint32_t m_stepTimeMs;       ///< Expected ReportUeMeasurements cadence in ms
    uint32_t m_uavNodeId;        ///< Node ID of the UAV (for Config paths)
    double m_handoverMargin{3.0}; ///< RSRP margin (dB) for action mask

    // EWMA smoothing factor for UL SINR (alpha=0.005 from pandas ewm analysis)
    double m_ewmaAlpha{0.005};

    // Per-cell measurement storage
    std::vector<double> m_rsrpValues;   ///< RSRP per cell in dBm (-140 = unknown, from ReportUeMeasurements)
    std::vector<double> m_rsrqValues;   ///< RSRQ per cell in dB  (-20 = unknown, from ReportUeMeasurements)
    std::vector<double> m_sinrValues;   ///< Raw SINR per cell in dB (-40 = unknown, from gNB UL SINR trace)
    std::vector<double> m_sinrSmoothed; ///< EWMA-smoothed SINR per cell

    // UE state
    uint32_t m_currentCellId{0};         ///< Current serving cell ID

    // TCP metrics
    int32_t m_currentRttMs{0};           ///< Current RTT (ms)

    // Physical layer metrics
    int64_t m_tbsSum{0};                 ///< Accumulated TBS bytes over current step
    uint32_t m_tbsCount{0};              ///< Number of TBS samples in current step

    // Timing
    Time m_lastReportTime{Seconds(0)}; ///< Timestamp of the last ReportUeMeasurements batch (to detect new cycles)

    // Previous-step snapshots for computing deltas
    std::vector<double> m_lastRsrpSnapshot;  ///< RSRP at last step (for delta, from ReportUeMeasurements)
    std::vector<double> m_lastRsrqSnapshot;  ///< RSRQ at last step (for delta, from ReportUeMeasurements)
    double m_lastSinrSnapshot{-40.0};        ///< EWMA-smoothed SINR at last step (for delta)

    /** Build the current observation dict from cached values. */
    Ptr<OpenGymDictContainer> BuildObservation();

    /** Send observation immediately (called from ObserveUeRsrpRsrq on batch completion). */
    void SendObservation();
};

} // namespace ns3
