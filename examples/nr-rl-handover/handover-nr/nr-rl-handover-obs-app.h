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
 * - Serving cell SINR (DL data SINR, step-averaged over the 200 ms obs step)
 * - Serving cell ID
 * - UAV heading (unit vector) and speed
 * - Time since last handover
 *
 * Observation is a flat Box vector (32 dims) wrapped in a Dict container
 * (key "obs") for Send() transport compatibility with the base class API.
 * The top-3 cell ranking is computed and stored in the global g_topNCells
 * array for the ActApp (Top-N fixed at 3: obs ranks exactly the cells the
 * action space can hand over to).
 *
 * Observation layout (32 dims):
 *   [0-1]   serving_rsrp / serving_rsrq
 *   [2-4]   slot_rsrp[0..2], [5-7] rsrp_delta[0..2]
 *   [8]     dl_sinr (mean over the 200 ms step), [9] time_since_ho,
 *   [10] norm_goodput, [11] ho_count_10s
 *   [12-14] ul_sinr / ul_rb_util / ul_sched_ue
 *   [15-19] time-delta block (d_serving_rsrp/sinr/rsrq, d_norm_goodput, d_margin)
 *   [20-22] d_slot_rsrp[0..2]
 *   [23-25] heading_x / heading_y / heading_z
 *   [26-28] pos_x / pos_y / pos_z (position / ISD)
 *   [29-31] pos2s_x / pos2s_y / pos2s_z (position 2 s ahead / ISD)
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
    static constexpr uint32_t kTopN = 3;    ///< Ranked NON-serving cells (action + obs Top-N)

    uint32_t m_numBs;                  ///< Number of gNBs/cells in the scenario
    uint32_t m_stepTimeMs;             ///< Expected ReportUeMeasurements cadence in ms
    uint32_t m_uavNodeId;              ///< Node ID of the UAV (for Config paths)


    bool m_useTbsObservation{true};
    // Per-cell RSRP/RSRQ (needed for Top-N ranking)
    std::vector<double> m_rsrpValues;  ///< RSRP per cell in dBm (-140 = unknown)
    std::vector<double> m_rsrqValues;  ///< RSRQ per cell in dB  (-20 = unknown)

    // Serving cell state
    uint32_t m_currentCellId{0};       ///< Current serving cell ID
    double m_servingRsrp{-140.0};      ///< Serving cell RSRP (dBm)
    double m_servingRsrq{-20.0};       ///< Serving cell RSRQ (dB)
    double m_sinrSum{0.0};            ///< Step accumulator: per-slot DL data SINR sum (dB)
    uint32_t m_sinrCount{0};           ///< Step accumulator: DL SINR sample count

    // Physical layer metrics (accumulated over step)
    int64_t m_tbsSum{0};               ///< Accumulated TBS bytes over current step
    uint32_t m_tbsCount{0};            ///< Number of TBS samples in current step

    // Handover timing
    Time m_lastHandoverTime{Seconds(0)}; ///< Timestamp of last HandoverEndOk

    // Windowed handover count (obs index 11: `ho_count_10s`).
    uint32_t m_hoRateWindowMs{10000}; ///< Sliding window for the count (ms)
    std::deque<double> m_hoTimes;     ///< Handover timestamps within the window (s)

    // Mobility cache — stored as separate components to avoid Vector include
    double m_velocityX{0.0}; ///< Current velocity X component (m/s)
    double m_velocityY{0.0}; ///< Current velocity Y component (m/s)
    double m_velocityZ{0.0}; ///< Current velocity Z component (m/s)

    /// Observation-window snapshot for the 1 s time-delta features. RSRP is
    /// keyed by physical cell so a rank re-ordering does not alias the trend.
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

    /** Build the current observation as a flat Box (32 dims), wrapped in Dict. */
    Ptr<OpenGymDictContainer> BuildObservation();

    /** Send observation (called on ReportUeMeasurements batch completion). */
    void SendObservation();
};

} // namespace ns3
