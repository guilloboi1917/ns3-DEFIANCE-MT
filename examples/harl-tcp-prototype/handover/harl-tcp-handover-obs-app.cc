#include "harl-tcp-handover-obs-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-enb-phy.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-phy.h"
#include "ns3/lte-ue-power-control.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/tcp-rate-ops.h"
#include "ns3/tcp-socket-base.h"

#include <cstdint>
#include <string>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in harl-tcp-scenario.cc)
extern std::vector<double> g_lastRsrpValues;
extern std::vector<double> g_lastSinrValues;
extern std::vector<double> g_lastRsrqValues;
extern NetDeviceContainer g_uavLteDevs;
extern NetDeviceContainer g_enbLteDevs;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("HarlTcpHandoverObservationApp");

HarlTcpHandoverObservationApp::HarlTcpHandoverObservationApp()
    : ObservationApplication()
{
}

HarlTcpHandoverObservationApp::~HarlTcpHandoverObservationApp()
{
}

TypeId
HarlTcpHandoverObservationApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::HarlTcpHandoverObservationApp")
            .SetParent<ObservationApplication>()
            .SetGroupName("defiance")
            .AddConstructor<HarlTcpHandoverObservationApp>()
            .AddAttribute("NumBs",
                          "Number of base stations/cells in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&HarlTcpHandoverObservationApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("UavNodeId",
                          "Node ID of the UAV for Config path registration.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&HarlTcpHandoverObservationApp::m_uavNodeId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("StepTimeMs",
                          "RL step interval (ms). Observation is sent at this cadence.",
                          UintegerValue(480),
                          MakeUintegerAccessor(&HarlTcpHandoverObservationApp::m_stepTimeMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverMargin",
                          "RSRP margin (3GPP range, ~1 dB per step). "
                          "Target must have RSRP > serving + margin. "
                          "-999 disables gating.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(
                              &HarlTcpHandoverObservationApp::m_handoverMargin),
                          MakeDoubleChecker<double>());
    return tid;
}

void
HarlTcpHandoverObservationApp::DoInitialize()
{
    ObservationApplication::DoInitialize();

    m_rsrpValues = std::vector<double>(m_numBs, -140.0); // sentinel in typical RSRP range
    m_rsrqValues = std::vector<double>(m_numBs, -20.0);  // sentinel in typical RSRQ range
    m_sinrValues = std::vector<double>(m_numBs, -40.0); // -40 dB = sentinel for "no measurement"
    m_currentCellId = 0;
    m_currentRrcState = 0;
    m_currentCwnd = 0;
    m_currentRttMs = 0;
    m_deliveryRateBps = 0;
    m_uavPosX = 0.0;
    m_uavPosY = 0.0;
    m_uavPosZ = 0.0;
    m_uavVelX = 0.0;
    m_uavVelY = 0.0;
    m_uavVelZ = 0.0;
    m_lastMcs = 0;
    m_lastTxPowerDbm = 0.0;
    m_lastRsrpSnapshot = std::vector<double>(m_numBs, -140.0);
    m_lastRsrqSnapshot = std::vector<double>(m_numBs, -20.0);
    m_lastSinrSnapshot = -40.0;
    m_lastSendTime = Seconds(0);


    // Cache pointer to UAV mobility model for direct position/velocity queries
    m_uavMobility = GetNode()->GetObject<MobilityModel>();
}

void
HarlTcpHandoverObservationApp::RegisterCallbacks()
{
    uint32_t nodeId = GetNode()->GetId();

    // --- Connect to UAV PHY for per-cell RSRP/RSRQ (all detectable cells, dBm) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/"
            "ReportUeMeasurements",
        MakeCallback(&HarlTcpHandoverObservationApp::ObserveUeRsrpRsrq, this));

    // --- Connect to eNB ReportUeSinr traces ---
    for (uint32_t i = 0; i < g_enbLteDevs.GetN(); ++i)
    {
        auto enbNode = g_enbLteDevs.Get(i)->GetNode();
        uint32_t enbNodeId = enbNode->GetId();

        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(enbNodeId) +
                "/DeviceList/*/$ns3::LteEnbNetDevice/ComponentCarrierMap/0/LteEnbPhy/"
                "ReportUeSinr",
            MakeCallback(&HarlTcpHandoverObservationApp::ObserveUlSinr, this));
    }

    // --- Connect to RRC state transitions on the UAV node ---
    Config::Connect("/NodeList/" + std::to_string(m_uavNodeId) +
                        "/DeviceList/*/$ns3::LteNetDevice/$ns3::LteUeNetDevice/LteUeRrc"
                        "/StateTransition",
                    MakeCallback(&HarlTcpHandoverObservationApp::ObserveRrcState, this));

    // --- Connect to UAV PHY traces (MCS, Tx power) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/"
            "UlPhyTransmission",
        MakeCallback(&HarlTcpHandoverObservationApp::ObserveUlPhyTransmission, this));

    Simulator::Schedule(Seconds(1.1), [this]() {
        Ptr<Node> uavNode = NodeList::GetNode(m_uavNodeId);
        Ptr<LteUeNetDevice> ueNetDev =
            uavNode->GetDevice(0)->GetObject<LteUeNetDevice>();
        if (ueNetDev)
        {
            Ptr<LteUePhy> uePhy = ueNetDev->GetPhy();
            Ptr<LteUePowerControl> powerCtrl = uePhy->GetUplinkPowerControl();
            if (powerCtrl)
            {
                powerCtrl->TraceConnectWithoutContext(
                    "ReportPuschTxPower",
                    MakeCallback(&HarlTcpHandoverObservationApp::ObserveUeTxPower, this));
            }
        }
    });

    // --- Schedule TCP/BPR trace connections after TCP apps start ---
    // TCP sockets are created at ~1.0s, schedule connection at 1.5s
    Simulator::Schedule(Seconds(1.5), [this, nodeId]() {
        // TCP congestion window
        std::string cwndPath =
            "/NodeList/" + std::to_string(m_uavNodeId) +
            "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow";
        Config::ConnectWithoutContext(cwndPath,
                                      MakeCallback(&HarlTcpHandoverObservationApp::ObserveCwnd,
                                                   this));

        // RTT
        std::string rttPath = "/NodeList/" + std::to_string(m_uavNodeId) +
                              "/$ns3::TcpL4Protocol/SocketList/0/RTT";
        Config::ConnectWithoutContext(rttPath,
                                      MakeCallback(&HarlTcpHandoverObservationApp::ObserveRtt,
                                                   this));

        // TCP rate sample (delivery rate)
        std::string ratePath =
            "/NodeList/" + std::to_string(m_uavNodeId) +
            "/$ns3::TcpL4Protocol/SocketList/0/RateOps/TcpRateSampleUpdated";
        Config::ConnectWithoutContext(
            ratePath,
            MakeCallback(&HarlTcpHandoverObservationApp::ObserveRateSample, this));

        NS_LOG_INFO("TCP/BBR traces connected for UAV node " << m_uavNodeId);
    });

    NS_LOG_INFO("HarlTcpHandoverObservationApp callbacks registered on node " << nodeId);

    // Kick off periodic observation send at stepTimeMs cadence
    Simulator::Schedule(MilliSeconds(m_stepTimeMs),
                        &HarlTcpHandoverObservationApp::SendObservation,
                        this);
}

void
HarlTcpHandoverObservationApp::ObserveUeRsrpRsrq(uint16_t rnti,
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
    if (g_uavLteDevs.GetN() == 0)
    {
        return;
    }
    auto ueNetDev = g_uavLteDevs.Get(0)->GetObject<LteUeNetDevice>();
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

    // Store RSRP/RSRQ (dBm) for this cell — lightweight, no send overhead
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
    }
}

void
HarlTcpHandoverObservationApp::ObserveUlSinr(uint16_t cellId,
                                             uint16_t rnti,
                                             double sinrLinear,
                                             uint8_t ccId)
{
    // Filter for our primary UAV UE
    if (g_uavLteDevs.GetN() == 0)
    {
        return;
    }
    auto ueNetDev = g_uavLteDevs.Get(0)->GetObject<LteUeNetDevice>();
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

    if (cellId > 0 && cellId <= m_numBs)
    {
        // Convert linear SINR to dB and update both local and global storage
        double sinrDb = (sinrLinear > 0) ? 10.0 * std::log10(sinrLinear) : -40.0;
        m_sinrValues[cellId - 1] = sinrDb;
        if (cellId < g_lastSinrValues.size())
        {
            g_lastSinrValues[cellId] = sinrDb; // 1-based index for global
        }
    }
}

void
HarlTcpHandoverObservationApp::ObserveRrcState(std::string context,
                                               uint64_t imsi,
                                               uint16_t cellId,
                                               uint16_t rnti,
                                               LteUeRrc::State oldState,
                                               LteUeRrc::State newState)
{
    // Guard: ensure sinr vector is initialized
    if (m_sinrValues.empty())
    {
        return;
    }

    // Filter for our primary UAV UE
    if (g_uavLteDevs.GetN() == 0)
    {
        return;
    }
    auto ueNetDev = g_uavLteDevs.Get(0)->GetObject<LteUeNetDevice>();
    if (!ueNetDev)
    {
        return;
    }
    auto uavImsi = ueNetDev->GetImsi();
    if (imsi != uavImsi)
    {
        return;
    }

    m_currentRrcState = static_cast<uint16_t>(newState);
    m_currentCellId = cellId;

    NS_LOG_INFO("RRC state: IMSI=" << imsi << " cellId=" << cellId
                << " " << oldState << " -> " << newState);
}

void
HarlTcpHandoverObservationApp::ObserveCwnd(uint32_t oldCwnd, uint32_t newCwnd)
{
    if (newCwnd > INT32_MAX)
    {
        NS_LOG_WARN("BBR cwnd > INT32_MAX, clamping to 0: " << newCwnd);
        m_currentCwnd = 0;
    }
    else
    {
        m_currentCwnd = static_cast<int32_t>(newCwnd);
    }
}

void
HarlTcpHandoverObservationApp::ObserveRtt(Time oldRtt, Time newRtt)
{
    m_currentRttMs = static_cast<int32_t>(newRtt.GetMilliSeconds());
}

void
HarlTcpHandoverObservationApp::ObserveRateSample(const TcpRateOps::TcpRateSample& sample)
{
    m_deliveryRateBps = static_cast<int32_t>(sample.m_deliveryRate.GetBitRate());
}

void
HarlTcpHandoverObservationApp::ObserveUlPhyTransmission(PhyTransmissionStatParameters param)
{
    m_lastMcs = static_cast<int32_t>(param.m_mcs);
    NS_LOG_INFO("UL MCS: " << m_lastMcs);
}

void
HarlTcpHandoverObservationApp::ObserveUeTxPower(uint16_t cellId,
                                                 uint16_t rnti,
                                                 double powerDbm)
{
    m_lastTxPowerDbm = powerDbm;
    NS_LOG_INFO("UE Tx power: " << m_lastTxPowerDbm << " dBm (cell " << cellId
                                 << ", RNTI " << rnti << ")");
}

/// Clamp a value to [lo, hi] (inline replacement for std::clamp, which requires C++17).
static double
Clamp(double val, double lo, double hi)
{
    return val < lo ? lo : (val > hi ? hi : val);
}

Ptr<OpenGymDictContainer>
HarlTcpHandoverObservationApp::BuildObservation()
{
    // --- Per-cell measurements ---
    auto rsrps = MakeBoxContainer<double>(m_numBs);
    auto rsrqs = MakeBoxContainer<double>(m_numBs);

    for (uint32_t i = 0; i < m_numBs; i++)
    {
        double rsrp = std::isnan(m_rsrpValues[i]) ? -140.0 : m_rsrpValues[i];
        double rsrq = std::isnan(m_rsrqValues[i]) ? -20.0 : m_rsrqValues[i];
        rsrps->AddValue(Clamp(rsrp, -160.0, -40.0));
        rsrqs->AddValue(Clamp(rsrq, -100.0, -3.0));
    }

    // Current serving cell UL SINR (scalar — only meaningful for serving cell)
    double currentSinr = -40.0;
    if (m_currentCellId > 0 && m_currentCellId <= m_numBs)
    {
        currentSinr = m_sinrValues[m_currentCellId - 1];
    }
    auto sinrContainer = MakeBoxContainer<double>(1, Clamp(std::isnan(currentSinr) ? -40.0 : currentSinr, -40.0, 50.0));

    // --- Deltas (change since last observation) ---
    auto rsrpDelta = MakeBoxContainer<double>(m_numBs);
    auto rsrqDelta = MakeBoxContainer<double>(m_numBs);
    for (uint32_t i = 0; i < m_numBs; i++)
    {
        double prevRsrp = (i < m_lastRsrpSnapshot.size()) ? m_lastRsrpSnapshot[i] : -200.0;
        double prevRsrq = (i < m_lastRsrqSnapshot.size()) ? m_lastRsrqSnapshot[i] : -200.0;
        // Guard: use rsrp/rsrq after sentinel replacement (done above), just guard delta from nan snapshot
        double curRsrp = std::isnan(m_rsrpValues[i]) ? -140.0 : m_rsrpValues[i];
        double curRsrq = std::isnan(m_rsrqValues[i]) ? -20.0 : m_rsrqValues[i];
        double dRsrp = (curRsrp > -110.0 && prevRsrp > -110.0) ? curRsrp - prevRsrp : 0.0;
        double dRsrq = (curRsrq > -20.0 && prevRsrq > -20.0) ? curRsrq - prevRsrq : 0.0;
        rsrpDelta->AddValue(Clamp(dRsrp, -60.0, 60.0));
        rsrqDelta->AddValue(Clamp(dRsrq, -60.0, 60.0));
    }
    double sinrVal = std::isnan(currentSinr) ? -40.0 : currentSinr;
    double sinrDeltaVal = (sinrVal != -40.0 && !std::isnan(m_lastSinrSnapshot) && m_lastSinrSnapshot != -40.0)
                              ? sinrVal - m_lastSinrSnapshot
                              : 0.0;
    auto sinrDeltaContainer = MakeBoxContainer<double>(1, Clamp(sinrDeltaVal, -20.0, 20.0));

    // Update snapshots for next step
    m_lastRsrpSnapshot = m_rsrpValues;
    m_lastRsrqSnapshot = m_rsrqValues;
    m_lastSinrSnapshot = sinrVal;

    // --- Cell ID ---
    auto cellIdContainer = CreateObject<OpenGymDiscreteContainer>();
    cellIdContainer->SetValue(m_currentCellId);

    // --- RRC state ---
    auto rrcStateContainer = CreateObject<OpenGymDiscreteContainer>();
    rrcStateContainer->SetValue(m_currentRrcState);

    // --- UAV position and velocity (queried live from MobilityModel) ---
    if (m_uavMobility)
    {
        Vector pos = m_uavMobility->GetPosition();
        Vector vel = m_uavMobility->GetVelocity();
        m_uavPosX = pos.x;
        m_uavPosY = pos.y;
        m_uavPosZ = pos.z;
        m_uavVelX = vel.x;
        m_uavVelY = vel.y;
        m_uavVelZ = vel.z;
    }
    auto posContainer = MakeBoxContainer<double>(3,
                                                  m_uavPosX,
                                                  m_uavPosY,
                                                  m_uavPosZ);
    auto velContainer = MakeBoxContainer<double>(3,
                                                  Clamp(m_uavVelX, -200.0, 200.0),
                                                  Clamp(m_uavVelY, -200.0, 200.0),
                                                  Clamp(m_uavVelZ, -200.0, 200.0));

    // --- PHY metrics ---
    auto mcsContainer = MakeBoxContainer<int32_t>(1, static_cast<int32_t>(Clamp(static_cast<double>(m_lastMcs), 0.0, 31.0)));
    auto txPowerContainer = MakeBoxContainer<double>(1, Clamp(m_lastTxPowerDbm, -50.0, 50.0));

    // --- TCP metrics ---
    auto rttContainer = MakeBoxContainer<int32_t>(1, static_cast<int32_t>(Clamp(static_cast<double>(m_currentRttMs), 0.0, 10000.0)));

    // --- Build dict ---
    auto dict = CreateObject<OpenGymDictContainer>();
    dict->Add("rsrps", rsrps);
    dict->Add("rsrqs", rsrqs);
    dict->Add("sinr", sinrContainer);
    dict->Add("rsrpDelta", rsrpDelta);
    dict->Add("rsrqDelta", rsrqDelta);
    dict->Add("sinrDelta", sinrDeltaContainer);
    dict->Add("cellId", cellIdContainer);
    dict->Add("position", posContainer);
    dict->Add("velocity", velContainer);
    dict->Add("mcs", mcsContainer);
    dict->Add("txPower", txPowerContainer);
    dict->Add("rtt", rttContainer);

    // --- Action mask (0/1 per action: 0=no-op, 1..numBs=target cell) ---
    auto actionMask = MakeBoxContainer<double>(m_numBs + 1);
    actionMask->AddValue(1.0);  // no-op always valid

    if (m_handoverMargin > -999.0 && m_currentCellId > 0 && m_currentCellId <= m_numBs)
    {
        double servingRsrp = m_rsrpValues[m_currentCellId - 1];
        for (uint32_t i = 1; i <= m_numBs; i++)
        {
            bool valid = (i != m_currentCellId) &&
                         (m_rsrpValues[i - 1] > -110.0) &&
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
            bool valid = (i != m_currentCellId) &&
                         (m_rsrpValues[i - 1] > -110.0);
            actionMask->AddValue(valid ? 1.0 : 0.0);
        }
    }
    dict->Add("action_mask", actionMask);

    return dict;
}

void
HarlTcpHandoverObservationApp::SendObservation()
{
    Send(BuildObservation());
    // Reschedule at stepTimeMs cadence
    Simulator::Schedule(MilliSeconds(m_stepTimeMs),
                        &HarlTcpHandoverObservationApp::SendObservation,
                        this);
}

} // namespace ns3
