#include "nr-rl-handover-obs-app.h"

#include "ns3/base-test.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-value.h"

#include <cstdint>
#include <fstream>
#include <string>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in
// nr-rl-handover-scenario.cc)
extern std::string g_flowDirection;
extern uint32_t g_senderNodeId;
extern std::vector<double> g_lastRsrpValues;
extern std::vector<double> g_lastSinrValues;
extern std::vector<double> g_lastRsrqValues;
extern NetDeviceContainer g_uavNrDevs;
extern NetDeviceContainer g_gnbNrDevs;
extern bool g_tcpAlive;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlHandoverObservationApp");

NrRlHandoverObservationApp::NrRlHandoverObservationApp()
    : ObservationApplication()
{
}

NrRlHandoverObservationApp::~NrRlHandoverObservationApp()
{
}

TypeId
NrRlHandoverObservationApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlHandoverObservationApp")
            .SetParent<ObservationApplication>()
            .SetGroupName("defiance")
            .AddConstructor<NrRlHandoverObservationApp>()
            .AddAttribute("NumBs",
                          "Number of base stations/cells in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("UavNodeId",
                          "Node ID of the UAV for Config path registration.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_uavNodeId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("StepTimeMs",
                          "Informational: expected ReportUeMeasurements cadence (ms). "
                          "Observation is sent on PHY callbacks, not timer.",
                          UintegerValue(200),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_stepTimeMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverMargin",
                          "RSRP margin (3GPP range, ~1 dB per step). "
                          "Target must have RSRP > serving + margin. "
                          "-999 disables gating.",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&NrRlHandoverObservationApp::m_handoverMargin),
                          MakeDoubleChecker<double>());
    return tid;
}

void
NrRlHandoverObservationApp::DoInitialize()
{
    ObservationApplication::DoInitialize();

    m_rsrpValues = std::vector<double>(m_numBs, -140.0); // sentinel in typical RSRP range
    m_rsrqValues = std::vector<double>(m_numBs, -20.0);  // sentinel in typical RSRQ range
    m_sinrValues = std::vector<double>(m_numBs, -40.0);  // -40 dB = sentinel for "no measurement"
    m_sinrSmoothed = std::vector<double>(m_numBs, -40.0);
    m_currentCellId = 0;
    m_currentRttMs = 0;
    m_tbsSum = 0;
    m_tbsCount = 0;
    m_lastRsrpSnapshot = std::vector<double>(m_numBs, -140.0);
    m_lastRsrqSnapshot = std::vector<double>(m_numBs, -20.0);
    m_lastSinrSnapshot = -40.0;
}

void
NrRlHandoverObservationApp::RegisterCallbacks()
{
    uint32_t nodeId = GetNode()->GetId();

    // --- Connect to UAV PHY for per-cell RSRP/RSRQ (all detectable cells, dBm) ---
    // Direction-agnostic: always on UAV PHY
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "ReportUeMeasurements",
        MakeCallback(&NrRlHandoverObservationApp::ObserveUeRsrpRsrq, this));

    // --- Connect to SINR trace (DL or UL depending on flow direction) ---
    if (g_flowDirection == "dl")
    {
        // DL data SINR at UE PHY (per-slot, linear scale)
        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(m_uavNodeId) +
                "/DeviceList/*/$ns3::NrUeNetDevice/"
                "ComponentCarrierMapUe/*/NrUePhy/"
                "DlDataSinr",
            MakeCallback(&NrRlHandoverObservationApp::ObserveDlSinr, this));
    }
    else
    {
        // UL SINR at gNB PHY (based on SRS)
        for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); ++i)
        {
            auto gnbNode = g_gnbNrDevs.Get(i)->GetNode();
            uint32_t gnbNodeId = gnbNode->GetId();

            Config::ConnectWithoutContext(
                "/NodeList/" + std::to_string(gnbNodeId) +
                    "/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/*/NrGnbPhy/"
                    "UlSinrTrace",
                MakeCallback(&NrRlHandoverObservationApp::ObserveUlSinr, this));
        }
    }

    // --- Connect to UAV PHY TBS trace (UL or DL depending on flow direction) ---
    {
        std::string tbSizeTrace = (g_flowDirection == "dl")
            ? "ReportDownlinkTbSize"
            : "UlPhyTransmission";
        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(m_uavNodeId) +
                "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/" +
                tbSizeTrace,
            MakeCallback(&NrRlHandoverObservationApp::ObserveUlTbs, this));
    }

    // --- Schedule TCP RTT trace on the TCP sender node ---
    // TCP sockets are created at ~1.0s, schedule connection at 1.5s
    Simulator::Schedule(Seconds(1.5), [this]() {
        uint32_t senderNodeId = g_senderNodeId;
        std::string rttPath =
            "/NodeList/" + std::to_string(senderNodeId) +
            "/$ns3::TcpL4Protocol/SocketList/*/RTT";
        Config::ConnectWithoutContext(
            rttPath,
            MakeCallback(&NrRlHandoverObservationApp::ObserveRtt, this));

        NS_LOG_INFO("RTT trace connected on sender node " << senderNodeId);
    });

    NS_LOG_INFO("NrRlHandoverObservationApp callbacks registered on node " << nodeId
                << " flowDirection=" << g_flowDirection);

    // Observation is sent on every ReportUeMeasurements batch completion,
    // not via a separate timer — see ObserveUeRsrpRsrq().
}

void
NrRlHandoverObservationApp::ObserveUeRsrpRsrq(uint16_t rnti,
                                                 uint16_t cellId,
                                                 double rsrp,
                                                 double rsrq,
                                                 bool isServingCell,
                                                 uint8_t componentCarrierId)
{
    // Guard: ensure vectors are initialized
    if (m_rsrpValues.empty())
    {
        return;
    }

    // Filter for our primary UAV UE — query RNTI fresh each call (RNTI changes after handover)
    if (g_uavNrDevs.GetN() == 0)
    {
        return;
    }
    auto ueNetDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
    if (!ueNetDev)
    {
        return;
    }
    auto ueRrc = ueNetDev->GetRrc();
    if (!ueRrc)
    {
        return;
    }
    auto uavRnti = ueRrc->GetRnti();
    if (rnti != uavRnti)
    {
        return;
    }

    // Store RSRP/RSRQ (dBm) for this cell and update EWMA
    if (cellId > 0 && cellId <= m_numBs)
    {
        m_rsrpValues[cellId - 1] = rsrp;
        m_rsrqValues[cellId - 1] = rsrq;

        if (isServingCell)
        {
            m_currentCellId = cellId;
        }

        if (cellId < g_lastRsrpValues.size())
        {
            g_lastRsrpValues[cellId] = rsrp;
        }
        if (cellId < g_lastRsrqValues.size())
        {
            g_lastRsrqValues[cellId] = rsrq;
        }

        // Detect new ReportUeMeasurements cycle: all callbacks in one cycle fire at the
        // same sim time. When the sim time advances, the previous cycle's data is complete
        // (all detectable cells have been written into the arrays). Send the observation.
        Time now = Simulator::Now();
        if (now > m_lastReportTime && m_lastReportTime > Seconds(0))
        {
            SendObservation();
        }
        m_lastReportTime = now;
    }
}

void
NrRlHandoverObservationApp::ObserveUlSinr(uint64_t imsi,
                                             SpectrumValue& sinrSpectrum,
                                             SpectrumValue& /* interferenceSpectrum */)
{
    // Guard: ensure sinr vector is initialized
    if (m_sinrValues.empty())
    {
        return;
    }

    // Filter for our primary UAV UE by IMSI
    if (g_uavNrDevs.GetN() == 0)
    {
        return;
    }
    auto ueNetDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
    if (!ueNetDev || ueNetDev->GetImsi() != imsi)
    {
        return;
    }

    // Average the SpectrumValue over all RBs to get a scalar SINR in dB
    double sumSinr = 0.0;
    uint32_t numRb = 0;
    for (auto it = sinrSpectrum.ConstValuesBegin(); it != sinrSpectrum.ConstValuesEnd(); ++it)
    {
        if (*it > 0.0)
        {
            sumSinr += 10.0 * std::log10(*it);
            numRb++;
        }
    }
    if (numRb == 0)
    {
        return;
    }
    double sinrDb = sumSinr / static_cast<double>(numRb);

    uint32_t cellId = m_currentCellId;
    if (cellId > 0 && cellId <= m_numBs)
    {
        m_sinrValues[cellId - 1] = sinrDb;
        m_sinrSmoothed[cellId - 1] =
            (m_sinrSmoothed[cellId - 1] == -40.0)
                ? sinrDb
                : m_ewmaAlpha * sinrDb + (1.0 - m_ewmaAlpha) * m_sinrSmoothed[cellId - 1];

        if (cellId < g_lastSinrValues.size())
        {
            g_lastSinrValues[cellId] = sinrDb;
        }
    }
}

void
NrRlHandoverObservationApp::ObserveDlSinr(uint16_t cellId,
                                            uint16_t rnti,
                                            double sinrLinear,
                                            uint16_t bwpId)
{
    // Guard: ensure sinr vector is initialized
    if (m_sinrValues.empty())
    {
        return;
    }

    // Convert linear SINR to dB
    double sinrDb = (sinrLinear > 0.0) ? (10.0 * std::log10(sinrLinear)) : -40.0;

    if (cellId > 0 && cellId <= m_numBs)
    {
        m_sinrValues[cellId - 1] = sinrDb;
        m_sinrSmoothed[cellId - 1] =
            (m_sinrSmoothed[cellId - 1] == -40.0)
                ? sinrDb
                : m_ewmaAlpha * sinrDb + (1.0 - m_ewmaAlpha) * m_sinrSmoothed[cellId - 1];

        if (cellId < g_lastSinrValues.size())
        {
            g_lastSinrValues[cellId] = sinrDb;
        }
    }

    // Run the same serving-cell SINR / observation-send logic as ObserveUlSinr
    if (cellId == m_currentCellId)
    {
        // No additional per-serving-cell logic needed; m_sinrValues handles it
    }
}

void
NrRlHandoverObservationApp::ObserveRtt(Time oldRtt, Time newRtt)
{
    m_currentRttMs = static_cast<int32_t>(newRtt.GetMilliSeconds());
}

void
NrRlHandoverObservationApp::ObserveUlTbs(uint64_t imsi, uint64_t tbSize)
{
    m_tbsSum += static_cast<int64_t>(tbSize);
    m_tbsCount++;
}

/// Clamp a value to [lo, hi] (inline replacement for std::clamp, which requires C++17).
static double
Clamp(double val, double lo, double hi)
{
    return val < lo ? lo : (val > hi ? hi : val);
}

Ptr<OpenGymDictContainer>
NrRlHandoverObservationApp::BuildObservation()
{
    // --- Per-cell measurements (RSRP/RSRQ from ReportUeMeasurements — already averaged) ---
    auto rsrps = MakeBoxContainer<double>(m_numBs);
    auto rsrqs = MakeBoxContainer<double>(m_numBs);

    for (uint32_t i = 0; i < m_numBs; i++)
    {
        double rsrp = std::isnan(m_rsrpValues[i]) ? -140.0 : m_rsrpValues[i];
        double rsrq = std::isnan(m_rsrqValues[i]) ? -20.0 : m_rsrqValues[i];
        rsrps->AddValue(Clamp(rsrp, -160.0, -40.0));
        rsrqs->AddValue(Clamp(rsrq, -100.0, -3.0));
    }

    // Current serving cell UL SINR (scalar, EWMA-smoothed from 1ms traces)
    double currentSinr = -40.0;
    if (m_currentCellId > 0 && m_currentCellId <= m_numBs)
    {
        currentSinr = m_sinrSmoothed[m_currentCellId - 1];
    }
    auto sinrContainer =
        MakeBoxContainer<double>(1,
                                 Clamp(std::isnan(currentSinr) ? -40.0 : currentSinr, -40.0, 50.0));

    // --- Deltas (change since last observation) ---
    auto rsrpDelta = MakeBoxContainer<double>(m_numBs);
    auto rsrqDelta = MakeBoxContainer<double>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        double prevRsrp = (i < m_lastRsrpSnapshot.size()) ? m_lastRsrpSnapshot[i] : -200.0;
        double prevRsrq = (i < m_lastRsrqSnapshot.size()) ? m_lastRsrqSnapshot[i] : -200.0;
        double curRsrp = std::isnan(m_rsrpValues[i]) ? -140.0 : m_rsrpValues[i];
        double curRsrq = std::isnan(m_rsrqValues[i]) ? -20.0 : m_rsrqValues[i];
        double dRsrp = (curRsrp > -110.0 && prevRsrp > -110.0) ? curRsrp - prevRsrp : 0.0;
        double dRsrq = (curRsrq > -20.0 && prevRsrq > -20.0) ? curRsrq - prevRsrq : 0.0;
        rsrpDelta->AddValue(Clamp(dRsrp, -60.0, 60.0));
        rsrqDelta->AddValue(Clamp(dRsrq, -60.0, 60.0));
    }
    double sinrVal = std::isnan(currentSinr) ? -40.0 : currentSinr;
    double sinrDeltaVal =
        (sinrVal != -40.0 && !std::isnan(m_lastSinrSnapshot) && m_lastSinrSnapshot != -40.0)
            ? sinrVal - m_lastSinrSnapshot
            : 0.0;
    auto sinrDeltaContainer = MakeBoxContainer<double>(1, Clamp(sinrDeltaVal, -20.0, 20.0));

    // Update snapshots for next step
    m_lastRsrpSnapshot = m_rsrpValues;
    m_lastRsrqSnapshot = m_rsrqValues;
    m_lastSinrSnapshot = sinrVal;

    // --- PHY metrics ---
    int32_t avgTbs = (m_tbsCount > 0) ? static_cast<int32_t>(m_tbsSum / m_tbsCount) : 0;
    // auto tbsContainer = MakeBoxContainer<int32_t>(
    //     1,
    //     static_cast<int32_t>(Clamp(static_cast<double>(avgTbs), 0.0, 100000.0)));
    m_tbsSum = 0;
    m_tbsCount = 0;

    // --- Build dict ---
    auto dict = CreateObject<OpenGymDictContainer>();
    dict->Add("rsrps", rsrps);
    dict->Add("rsrqs", rsrqs);
    dict->Add("sinr", sinrContainer);
    dict->Add("rsrpDelta", rsrpDelta);
    dict->Add("rsrqDelta", rsrqDelta);
    dict->Add("sinrDelta", sinrDeltaContainer);
    // dict->Add("tbs", tbsContainer);

    // --- Action mask (0/1 per action: 0=no-op, 1..numBs=target cell) ---
    auto actionMask = MakeBoxContainer<double>(m_numBs + 1);
    actionMask->AddValue(1.0); // no-op always valid

    if (m_handoverMargin > -999.0 && m_currentCellId > 0 && m_currentCellId <= m_numBs)
    {
        double servingRsrp = m_rsrpValues[m_currentCellId - 1];
        for (uint32_t i = 1; i <= m_numBs; i++)
        {
            bool valid = (i != m_currentCellId) && (m_rsrpValues[i - 1] > -110.0) &&
                         (servingRsrp > -110.0) &&
                         (m_rsrpValues[i - 1] > servingRsrp + m_handoverMargin);
            actionMask->AddValue(valid ? 1.0 : 0.0);
        }
    }
    else
    {
        // Margin disabled or current cell unknown:
        // block same-cell and cells below noise floor
        for (uint32_t i = 1; i <= m_numBs; i++)
        {
            bool valid = (i != m_currentCellId) && (m_rsrpValues[i - 1] > -110.0);
            actionMask->AddValue(valid ? 1.0 : 0.0);
        }
    }
    dict->Add("action_mask", actionMask);

    return dict;
}

void
NrRlHandoverObservationApp::SendObservation()
{
    // Capture TBS average before BuildObservation resets the accumulator
    int32_t avgTbs = (m_tbsCount > 0) ? static_cast<int32_t>(m_tbsSum / m_tbsCount) : 0;

    auto obs = BuildObservation();

    // Log observation to CSV if logging is enabled
    if (g_logging)
    {
        double servingRsrp = -140.0;
        double servingRsrq = -20.0;
        double currentSinr = -40.0;
        if (m_currentCellId > 0 && m_currentCellId <= m_numBs)
        {
            servingRsrp = m_rsrpValues[m_currentCellId - 1];
            servingRsrq = m_rsrqValues[m_currentCellId - 1];
            currentSinr = m_sinrSmoothed[m_currentCellId - 1];
        }
        std::ofstream obsFile(g_outputDir + "rl_obs.csv", std::ios_base::app);
        obsFile << Simulator::Now().GetSeconds() << "," << m_currentCellId << "," << servingRsrp
                << "," << servingRsrq << "," << currentSinr << "," << avgTbs << ","
                << (g_tcpAlive ? m_currentRttMs : 0);
        for (double rsrp : m_rsrpValues)
        {
            obsFile << "," << rsrp;
        }
        obsFile << std::endl;
    }

    Send(obs);
}

} // namespace ns3
