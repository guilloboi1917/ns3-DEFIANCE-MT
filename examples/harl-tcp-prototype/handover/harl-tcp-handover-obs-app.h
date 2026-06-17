#include "ns3/lte-common.h"
#include "ns3/lte-rrc-sap.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/mobility-module.h"
#include "ns3/observation-application.h"
#include "ns3/tcp-rate-ops.h"

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Observation application for the HARL TCP handover RL agent.
 *
 * Runs on the UAV node and collects:
 * - RSRP/RSRQ per cell from eNB measurement reports
 * - SINR per cell from SRS-based UL SINR reports
 * - Current serving cell ID and RRC state
 * - UAV position (x, y, z) and velocity (vx, vy, vz)
 * - MCS index and UAV Tx power
 * - TCP congestion window (cwnd)
 * - RTT
 * - BBR-specific metrics: delivery rate, BW estimate, inflight/BDP ratio, BBR state
 *
 * Observations are packaged as an OpenGymDictContainer and sent to the agent
 * at every StepTime interval.
 */
class HarlTcpHandoverObservationApp : public ObservationApplication
{
  public:
    HarlTcpHandoverObservationApp();
    ~HarlTcpHandoverObservationApp() override;

    static TypeId GetTypeId();
    void DoInitialize() override;

    /** Register Config callbacks for measurement collection. */
    void RegisterCallbacks() override;

    // --- Callback handlers ---

    /** Handle RecvMeasurementReport from the serving eNB. */
    void ObserveMeasurementReport(uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti,
                                  LteRrcSap::MeasurementReport report);

    /** Handle ReportUeSinr (SRS-based UL SINR) from eNBs. */
    void ObserveUlSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint8_t ccId);

    /** Handle RRC state transitions. */
    void ObserveRrcState(std::string context,
                         uint64_t imsi,
                         uint16_t cellId,
                         uint16_t rnti,
                         LteUeRrc::State oldState,
                         LteUeRrc::State newState);

    /** Handle TCP congestion window changes. */
    void ObserveCwnd(uint32_t oldCwnd, uint32_t newCwnd);

    /** Handle RTT changes. */
    void ObserveRtt(Time oldRtt, Time newRtt);

    // --- BBR-specific callbacks ---

    /** Handle TCP rate sample updates (delivery rate). */
    void ObserveRateSample(const TcpRateOps::TcpRateSample& sample);

    // --- UAV state callbacks ---

    /** Handle UL transmission stats (MCS index). */
    void ObserveUlPhyTransmission(PhyTransmissionStatParameters param);

    /** Handle PUSCH Tx power reports. */
    void ObserveUeTxPower(uint16_t cellId, uint16_t rnti, double powerDbm);

  private:
    uint32_t m_numBs;            ///< Number of eNBs/cells in the scenario
    uint32_t m_stepTimeMs;       ///< Observation interval in ms
    uint32_t m_uavNodeId;        ///< Node ID of the UAV (for Config paths)

    // Per-cell measurement storage
    std::vector<int32_t> m_rsrpValues;   ///< RSRP per cell (-1 = unknown)
    std::vector<int32_t> m_rsrqValues;   ///< RSRQ per cell (-1 = unknown)
    std::vector<double> m_sinrValues;    ///< SINR per cell in dB (-40 = unknown)

    // UE state
    uint32_t m_currentCellId{0};         ///< Current serving cell ID
    uint16_t m_currentRrcState{0};       ///< Current RRC state

    // TCP metrics
    int32_t m_currentCwnd{0};            ///< Current congestion window (bytes)
    int32_t m_currentRttMs{0};           ///< Current RTT (ms)

    // TCP metrics
    int32_t m_deliveryRateBps{0};        ///< TCP delivery rate (bps)

    // UAV position and dynamics
    Ptr<MobilityModel> m_uavMobility;    ///< Cached pointer to UAV's mobility model
    double m_uavPosX{0.0};               ///< UAV position X (m)
    double m_uavPosY{0.0};               ///< UAV position Y (m)
    double m_uavPosZ{0.0};               ///< UAV position Z (m)
    double m_uavVelX{0.0};               ///< UAV velocity X (m/s)
    double m_uavVelY{0.0};               ///< UAV velocity Y (m/s)
    double m_uavVelZ{0.0};               ///< UAV velocity Z (m/s)

    // Physical layer metrics
    int32_t m_lastMcs{0};                ///< Last UL MCS index (0-28 for LTE)
    double m_lastTxPowerDbm{0.0};        ///< Last PUSCH Tx power (dBm)

    // Timing
    Time m_lastSendTime{Seconds(0)};



    /** Build the current observation dict from cached values. */
    Ptr<OpenGymDictContainer> BuildObservation();
};

} // namespace ns3
