#include "uav-handover-observation-application.h"

#include "ns3/base-test.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-enb-phy.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/net-device-container.h"

#include <cstdint>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UavHandoverObservationApplication");

extern NetDeviceContainer g_ueLteDevs;
extern NetDeviceContainer g_enbLteDevs;

UavHandoverObservationApplication::UavHandoverObservationApplication()
    : ObservationApplication()
{
}

UavHandoverObservationApplication::~UavHandoverObservationApplication()
{
}

void
UavHandoverObservationApplication::DoInitialize()
{
    ObservationApplication::DoInitialize();
    m_observations = std::vector<std::pair<int32_t, int32_t>>(m_numBs, std::make_pair(-1, -1));
    m_sinrValues = std::vector<double>(m_numBs, 0.0);
}

TypeId
UavHandoverObservationApplication::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::UavHandoverObservationApplication")
            .SetParent<ObservationApplication>()
            .SetGroupName("defiance")
            .AddConstructor<UavHandoverObservationApplication>()
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&UavHandoverObservationApplication::m_numBs),
                          MakeUintegerChecker<uint>())
            .AddAttribute("StepTime",
                          "Interval (ms) between periodic observation sends.",
                          UintegerValue(420),
                          MakeUintegerAccessor(&UavHandoverObservationApplication::m_stepTimeMs),
                          MakeUintegerChecker<uint>());
    return tid;
}

void
UavHandoverObservationApplication::Observe(uint64_t imsi,
                                           uint16_t cellId,
                                           uint16_t rnti,
                                           LteRrcSap::MeasurementReport report)
{
    // Measurement reports only from our primary UE
    if (imsi != g_ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetImsi())
    {
        return;
    }

    auto rsrp = report.measResults.measResultPCell.rsrpResult;
    auto rsrq = report.measResults.measResultPCell.rsrqResult;

    m_observations[cellId - 1] = std::make_pair(rsrp, rsrq);

    auto listEutra = report.measResults.measResultListEutra;
    for (auto it = listEutra.begin(); it != listEutra.end(); ++it)
    {
        auto secCellId = it->physCellId;
        m_observations[secCellId - 1] = std::make_pair(it->rsrpResult, it->rsrqResult);
    }

    m_currentCellId = cellId;

    if (Simulator::Now() == m_lastObservationTime)
    {
        NS_LOG_INFO("Already received observation for this time.");
    }
    else
    {
        // Send immediately when a new measurement report arrives
        auto rsrps = CreateObject<OpenGymBoxContainer<int32_t>>();
        auto rsrqs = CreateObject<OpenGymBoxContainer<int32_t>>();
        auto sinrs = CreateObject<OpenGymBoxContainer<double>>();

        for (uint32_t i = 0; i < m_numBs; i++)
        {
            rsrps->AddValue(m_observations[i].first);
            rsrqs->AddValue(m_observations[i].second);
            sinrs->AddValue(m_sinrValues[i]);
        }

        NS_LOG_INFO("RSRPs: " << rsrps << " RSRQs: " << rsrqs << " CellId: " << cellId
                              << " SINRs: " << sinrs);

        auto observationDict = CreateObject<OpenGymDictContainer>();
        observationDict->Add("rsrps", rsrps);
        observationDict->Add("rsrqs", rsrqs);
        observationDict->Add("sinrs", sinrs);
        auto cellIdContainer = CreateObject<OpenGymDiscreteContainer>();
        cellIdContainer->SetValue(cellId);
        observationDict->Add("cellId", cellIdContainer);
        m_lastObservationTime = Simulator::Now();
        m_hasEverSent = true;
        Send(observationDict);
    }
}

void
UavHandoverObservationApplication::ObserveSinr(uint16_t cellId,
                                               uint16_t rnti,
                                               double sinrLinear,
                                               uint8_t componentCarrierId)
{
    // Only observe SINR for our primary UE
    if (rnti != g_ueLteDevs.Get(0)->GetObject<LteUeNetDevice>()->GetRrc()->GetRnti())
    {
        return;
    }

    // Convert linear SINR to dB for observation
    if (sinrLinear > 0)
    {
        m_sinrValues[cellId - 1] = 10.0 * std::log10(sinrLinear);
    }
    else
    {
        m_sinrValues[cellId - 1] = 0.0;
    }
}

void
UavHandoverObservationApplication::SendPeriodicObservation()
{
    // Build and send observation with latest values (may be sentinels if no report)
    auto rsrps = CreateObject<OpenGymBoxContainer<int32_t>>();
    auto rsrqs = CreateObject<OpenGymBoxContainer<int32_t>>();
    auto sinrs = CreateObject<OpenGymBoxContainer<double>>();

    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrps->AddValue(m_observations[i].first);
        rsrqs->AddValue(m_observations[i].second);
        sinrs->AddValue(m_sinrValues[i]);
    }

    auto observationDict = CreateObject<OpenGymDictContainer>();
    observationDict->Add("rsrps", rsrps);
    observationDict->Add("rsrqs", rsrqs);
    observationDict->Add("sinrs", sinrs);
    auto cellIdContainer = CreateObject<OpenGymDiscreteContainer>();
    cellIdContainer->SetValue(m_currentCellId);
    observationDict->Add("cellId", cellIdContainer);

    m_lastObservationTime = Simulator::Now();
    m_hasEverSent = true;
    Send(observationDict);

    // Schedule next periodic observation
    Simulator::Schedule(MilliSeconds(m_stepTimeMs),
                        &UavHandoverObservationApplication::SendPeriodicObservation,
                        this);
}

void
UavHandoverObservationApplication::RegisterCallbacks()
{
    auto nodeId = GetNode()->GetId();

    // Connect to measurement reports from UEs
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(nodeId) +
            "/DeviceList/*/$ns3::LteEnbNetDevice/LteEnbRrc/RecvMeasurementReport",
        MakeCallback(&UavHandoverObservationApplication::Observe, this));

    // Connect to UE SINR reports at this eNB (SRS-based)
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(nodeId) +
            "/DeviceList/*/$ns3::LteEnbNetDevice/ComponentCarrierMap/0/LteEnbPhy/ReportUeSinr",
        MakeCallback(&UavHandoverObservationApplication::ObserveSinr, this));

    // Start periodic observation sending (so agent gets observations even without reports)
    Simulator::Schedule(MilliSeconds(m_stepTimeMs),
                        &UavHandoverObservationApplication::SendPeriodicObservation,
                        this);
}
