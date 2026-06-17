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
extern std::vector<int32_t> g_lastRsrpValues;
extern std::vector<double> g_lastSinrValues;
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
                          MakeUintegerChecker<uint32_t>());
    return tid;
}

void
HarlTcpHandoverObservationApp::DoInitialize()
{
    ObservationApplication::DoInitialize();

    m_rsrpValues = std::vector<int32_t>(m_numBs, -1);
    m_rsrqValues = std::vector<int32_t>(m_numBs, -1);
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
    m_lastSendTime = Seconds(0);


    // Cache pointer to UAV mobility model for direct position/velocity queries
    m_uavMobility = GetNode()->GetObject<MobilityModel>();
}

void
HarlTcpHandoverObservationApp::RegisterCallbacks()
{
    uint32_t nodeId = GetNode()->GetId();

    // --- Connect to eNB measurement reports ---
    // Iterate over all eNBs to get per-cell measurement reports
    for (uint32_t i = 0; i < g_enbLteDevs.GetN(); ++i)
    {
        auto enbNode = g_enbLteDevs.Get(i)->GetNode();
        uint32_t enbNodeId = enbNode->GetId();

        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(enbNodeId) +
                "/DeviceList/*/$ns3::LteEnbNetDevice/LteEnbRrc/RecvMeasurementReport",
            MakeCallback(&HarlTcpHandoverObservationApp::ObserveMeasurementReport, this));

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
}

void
HarlTcpHandoverObservationApp::ObserveMeasurementReport(uint64_t imsi,
                                                        uint16_t cellId,
                                                        uint16_t rnti,
                                                        LteRrcSap::MeasurementReport report)
{
    // Guard: ensure vectors are initialized
    if (m_rsrpValues.empty())
    {
        return;
    }

    // Filter for our primary UAV UE only
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

    // Update serving cell RSRP and RSRQ
    auto rsrp = report.measResults.measResultPCell.rsrpResult;
    auto rsrq = report.measResults.measResultPCell.rsrqResult;
    if (cellId > 0 && cellId <= m_numBs)
    {
        m_rsrpValues[cellId - 1] = rsrp;
        m_rsrqValues[cellId - 1] = rsrq;
        if (cellId < g_lastRsrpValues.size())
        {
            g_lastRsrpValues[cellId] = rsrp;
        }
    }

    // Update neighbor cell measurements
    auto listEutra = report.measResults.measResultListEutra;
    for (auto it = listEutra.begin(); it != listEutra.end(); ++it)
    {
        auto neighborCellId = it->physCellId;
        if (neighborCellId > 0 && neighborCellId <= m_numBs)
        {
            m_rsrpValues[neighborCellId - 1] = it->rsrpResult;
            m_rsrqValues[neighborCellId - 1] = it->rsrqResult;
            if (neighborCellId < g_lastRsrpValues.size())
            {
                g_lastRsrpValues[neighborCellId] = it->rsrpResult;
            }
        }
    }

    m_currentCellId = cellId;

    // --- Send observation to agent (event-driven) ---
    Send(BuildObservation());

    NS_LOG_INFO("Measurement report: cellId=" << cellId
                << " RSRP=" << rsrp);
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
    m_currentCwnd = static_cast<int32_t>(newCwnd);
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

Ptr<OpenGymDictContainer>
HarlTcpHandoverObservationApp::BuildObservation()
{
    // --- Per-cell measurements ---
    auto rsrps = CreateObject<OpenGymBoxContainer<int32_t>>();
    auto rsrqs = CreateObject<OpenGymBoxContainer<int32_t>>();

    for (uint32_t i = 0; i < m_numBs; i++)
    {
        rsrps->AddValue(m_rsrpValues[i]);
        rsrqs->AddValue(m_rsrqValues[i]);
    }

    // Current serving cell UL SINR (scalar — only meaningful for serving cell)
    double currentSinr = -40.0;
    if (m_currentCellId > 0 && m_currentCellId <= m_numBs)
    {
        currentSinr = m_sinrValues[m_currentCellId - 1];
    }
    auto sinrContainer = MakeBoxContainer<double>(1, currentSinr);

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
                                                  m_uavVelX,
                                                  m_uavVelY,
                                                  m_uavVelZ);

    // --- PHY metrics ---
    auto mcsContainer = MakeBoxContainer<int32_t>(1, m_lastMcs);
    auto txPowerContainer = MakeBoxContainer<double>(1, m_lastTxPowerDbm);

    // --- TCP metrics ---
    auto cwndContainer = MakeBoxContainer<int32_t>(1, m_currentCwnd);
    auto rttContainer = MakeBoxContainer<int32_t>(1, m_currentRttMs);

    // --- BBR metrics ---
    auto deliveryRateContainer = MakeBoxContainer<int32_t>(1, m_deliveryRateBps);

    // --- Build dict ---
    auto dict = CreateObject<OpenGymDictContainer>();
    dict->Add("rsrps", rsrps);
    dict->Add("rsrqs", rsrqs);
    dict->Add("sinr", sinrContainer);
    dict->Add("cellId", cellIdContainer);
    dict->Add("rrcState", rrcStateContainer);
    dict->Add("position", posContainer);
    dict->Add("velocity", velContainer);
    dict->Add("mcs", mcsContainer);
    dict->Add("txPower", txPowerContainer);
    dict->Add("cwnd", cwndContainer);
    dict->Add("rtt", rttContainer);
    dict->Add("deliveryRate", deliveryRateContainer);

    return dict;
}

} // namespace ns3
