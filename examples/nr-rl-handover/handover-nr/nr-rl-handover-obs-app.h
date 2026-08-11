#include "ns3/observation-application.h"
#include "ns3/spectrum-value.h"

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace ns3
{
class MobilityModel;
class Address;
class Packet;

/**
 * @ingroup defiance
 * @brief Observation application for the NR RL handover agent (Top-N design).
 *
 * Runs on the UAV node and collects:
 * - RSRP/RSRQ per cell from ReportUeMeasurements (200ms, averaged by UE PHY)
 * - Serving cell SINR (DL data SINR, EWMA-smoothed)
 * - Serving cell ID
 * - Transport block size (DL: ReportDownlinkTbSize)
 * - UAV heading (unit vector in XZ plane) and speed
 * - Time since last handover
 *
 * Observation is a flat Box vector (29 dims) wrapped in a Dict container
 * (key "obs") for Send() transport compatibility with the base class API.
 * Top-5 cell ranking is computed and stored in the global g_topNCells
 * array for the ActApp.
 *
 * Observation layout (29 dims):
 *   [0] serving_rsrp, [1] serving_rsrq,
 *   [2..6] slot_rsrp[0..4], [7..11] slot_rsrq[0..4],
 *   [12..16] rsrp_delta[0..4], [17..21] rsrq_delta[0..4],
 *   [22] sinr, [23] heading_x, [24] heading_z, [25] speed,
 *   [26] tbs, [27] time_since_ho, [28] norm_goodput
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

    /** Handle ReportUeRsrpRsrq from the UAV PHY (per-cell RSRP/RSRQ in dB/dBm). */
    void ObserveUeRsrpRsrq(uint16_t rnti,
                           uint16_t cellId,
                           double rsrp,
                           double rsrq,
                           bool isServingCell,
                           uint8_t componentCarrierId);

    /** Handle DL data SINR from UE PHY (DlDataSinr: per-slot linear value). */
    void ObserveDlSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint16_t bwpId);

    /** Handle transport block size (DL: ReportDownlinkTbSize). */
    void ObserveTbs(uint64_t imsi, uint64_t tbSize);

    /** Handle HandoverEndOk to track last handover time for time_since_ho. */
    void ObserveHandover(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti);

    /** Handle mobility CourseChange to cache velocity for heading/speed. */
    void ObserveCourseChange(std::string context, Ptr<const MobilityModel> model);

    /** Handle PacketSink Rx on the receiver node (for normalized goodput). */
    void ObserveSinkRx(Ptr<const Packet> packet, const Address& from);

  private:
    uint32_t m_numBs;                  ///< Number of gNBs/cells in the scenario
    uint32_t m_topN{5};                ///< Number of ranked cells in Top-N action space
    uint32_t m_stepTimeMs;             ///< Expected ReportUeMeasurements cadence in ms
    uint32_t m_uavNodeId;              ///< Node ID of the UAV (for Config paths)

    // EWMA smoothing factor for serving cell SINR
    double m_sinrEwmaAlpha{0.1};

    // Emit real avg TBS at obs index 26, or a constant (0.0) when disabled.
    // Disabling tests removing the goodput-reward-proxy feature without
    // changing the obs layout (easy revert: set back to true).
    bool m_useTbsObservation{true};

    // Per-cell RSRP/RSRQ (needed for Top-N ranking)
    std::vector<double> m_rsrpValues;  ///< RSRP per cell in dBm (-140 = unknown)
    std::vector<double> m_rsrqValues;  ///< RSRQ per cell in dB  (-20 = unknown)

    // Serving cell state
    uint32_t m_currentCellId{0};       ///< Current serving cell ID
    double m_servingRsrp{-140.0};      ///< Serving cell RSRP (dBm)
    double m_servingRsrq{-20.0};       ///< Serving cell RSRQ (dB)
    double m_servingSinr{-40.0};       ///< Serving cell SINR, EWMA-smoothed (dB)

    // Physical layer metrics (accumulated over step)
    int64_t m_tbsSum{0};               ///< Accumulated TBS bytes over current step
    uint32_t m_tbsCount{0};            ///< Number of TBS samples in current step

    // Handover timing
    Time m_lastHandoverTime{Seconds(0)}; ///< Timestamp of last HandoverEndOk

    // Windowed handover count (obs index 29: `ho_count_10s`). Mirrors the
    // reward app's HandoverRateWindowMs so the agent can observe how close it
    // is to the rate penalty threshold (credit assignment for the penalty).
    uint32_t m_hoRateWindowMs{10000}; ///< Sliding window for the count (ms)
    std::deque<double> m_hoTimes;     ///< Handover timestamps within the window (s)

    // Mobility cache — stored as separate components to avoid Vector include
    double m_velocityX{0.0}; ///< Current velocity X component (m/s)
    double m_velocityY{0.0}; ///< Current velocity Y component (m/s)
    double m_velocityZ{0.0}; ///< Current velocity Z component (m/s)

    /// One observation-window snapshot used by the 1 s time-delta features.
    /// RSRP is stored per PHYSICAL cell (cellId-keyed) so time deltas compare
    /// each cell against its own 1 s-old value — rank re-orderings do not alias
    /// the trend (d_slot_rsrp[k] stays the trend of the same physical cell even
    /// as the ranking shifts). Memory: 5 snapshots x numBs doubles.
    struct ObsSnapshot
    {
        double servingRsrq{-20.0};   ///< Serving RSRQ (dB)
        double servingSinr{-40.0};   ///< Serving SINR (dB)
        double normGoodput{0.0};     ///< Windowed normalized goodput
        double margin{-140.0};       ///< serving_rsrp - best_slot_rsrp (dB)
        std::vector<double> cellRsrp; ///< RSRP per cell (size m_numBs); -140 = unknown
    };
    static constexpr uint32_t OBS_WINDOW_STEPS = 5; ///< 1.0 s trend window at 0.2 s steps
    ObsSnapshot m_obsHistory[OBS_WINDOW_STEPS];     ///< Ring buffer of recent snapshots
    uint32_t m_obsHistoryIndex{0};                  ///< Next write slot in m_obsHistory
    uint32_t m_obsHistoryCount{0};                  ///< Number of valid snapshots

    // Goodput normalization (fixed upper-bound reference, OnOff data rate)
    double m_goodputRefBps{40e6};   ///< Reward reference (near achievable throughput)
    uint64_t m_sinkBytesReceived{0}; ///< Sink bytes since last observation
    Time m_lastObservationTime{Seconds(0)}; ///< Timestamp of last sent observation (window boundary)

    // Timing
    Time m_lastReportTime{Seconds(0)}; ///< Timestamp of last ReportUeMeasurements batch

    /** Compute top-N cells by RSRP and populate g_topNCells. */
    void ComputeTopNCells();

    /** Build the current observation as a flat Box (29 dims), wrapped in Dict. */
    Ptr<OpenGymDictContainer> BuildObservation();

    /** Send observation (called on ReportUeMeasurements batch completion). */
    void SendObservation();
};

} // namespace ns3
