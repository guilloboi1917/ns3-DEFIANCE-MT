#include "ns3/lte-rrc-sap.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/observation-application.h"

using namespace ns3;

/**
 * @ingroup defiance
 * @brief Child class of ObservationApp that observes measurement reports and SINR of UEs.
 *
 * Sends observations periodically every StepTime, using the latest available
 * measurement data. If no measurement report has been received, sentinel values
 * are used (RSRP/RSRQ=-1, SINR=0, cellId=0). This ensures the RL agent
 * continues to receive observations even when the UE loses connectivity.
 */
class UavHandoverObservationApplication : public ObservationApplication
{
  public:
    UavHandoverObservationApplication();
    ~UavHandoverObservationApplication() override;

    void DoInitialize() override;

    static TypeId GetTypeId();

    /**
     * Callback for eNB RecvMeasurementReport trace.
     */
    void Observe(uint64_t imsi, uint16_t rrc, uint16_t rnti, LteRrcSap::MeasurementReport msg);

    /**
     * Callback for eNB ReportUeSinr trace (SRS-based SINR).
     */
    void ObserveSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint8_t componentCarrierId);

    /**
     * Periodically send the current observation (even if no new measurement).
     */
    void SendPeriodicObservation();

    void RegisterCallbacks() override;

  private:
    uint32_t m_numBs;
    uint32_t m_stepTimeMs;
    std::vector<std::pair<int32_t, int32_t>> m_observations;
    std::vector<double> m_sinrValues;
    Time m_lastObservationTime = Seconds(0);
    bool m_hasEverSent{false};
    uint32_t m_currentCellId{0};
};

NS_OBJECT_ENSURE_REGISTERED(UavHandoverObservationApplication);
