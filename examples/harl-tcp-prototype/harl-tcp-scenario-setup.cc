/**
 * Setup file for the HARL TCP Scenario
 * Supports two topologies:
 *   "simple"  — 2 eNodeBs on a line, UAV shuttles between them
 *   "hexgrid" — hexagonal grid of 3-sector macro sites, UAV follows a helix
 * 1 aerial UE, 1 remote server. The UE runs a TCP BulkSendApplication to the remote server.
 */

#include "handover/harl-tcp-handover-act-app.h"
#include "handover/harl-tcp-handover-agent-app.h"
#include "handover/harl-tcp-handover-obs-app.h"
#include "handover/harl-tcp-handover-rwd-app.h"

#include "ns3/applications-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-common.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/rl-application-helper.h"
#include "ns3/tcp-bbr.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-rate-ops.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/three-gpp-antenna-model-oriented.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

std::string pathToNs3 = std::getenv("NS3_HOME");
std::string g_outputDir;

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HarlTcpScenarioSetup");

int16_t g_currentRnti = 1;
int16_t g_currentCellId = 1;

// Application Connection succeed/fail callbacks
extern bool g_tcpConnected;
extern bool g_tcpAlive;
extern uint64_t g_totalRxBytes;
extern uint32_t g_totalRetransmissions;
extern double g_rttSumMs;
extern uint32_t g_rttSamples;

void
NotifyConnectionSucceeded(Ptr<Socket> socket, const Address& local, const Address& remote)
{
    g_tcpConnected = true;
    g_tcpAlive = true;
    std::cout << "TCP connection succeeded at time " << Simulator::Now().GetSeconds() << "s"
              << std::endl;
}

void
NotifyConnectionFailed(Ptr<Socket> socket, const Address& local, const Address& remote)
{
    g_tcpAlive = false;
    std::cout << "TCP connection failed at time " << Simulator::Now().GetSeconds() << "s"
              << std::endl;
}

void
NotifyTcpStateChange(const TcpSocket::TcpStates_t oldState,
                     const TcpSocket::TcpStates_t newState)
{
    if (newState == TcpSocket::CLOSED || newState == TcpSocket::LAST_ACK)
    {
        g_tcpAlive = false;
        std::cout << "TCP socket closed (state=" << newState
                  << ") at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    }
}

// CWND tracing callback — fires on every CWND change (every ACK)
void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    std::ofstream cwndFile(g_outputDir + "harl-tcp-cwnd.csv", std::ios_base::app);
    cwndFile << Simulator::Now().GetSeconds() << "," << newCwnd << std::endl;
}

void
TcpRttChange(Time oldValue, Time newValue)
{
    // Skip default/backed-off RTT values before connection is established.
    // m_lastRtt starts at Seconds(3) and may carry RTO-backed-off values
    // during handshake (up to 12s+), which would corrupt the average.
    if (!g_tcpConnected)
    {
        return;
    }
    double rttMs = newValue.GetMilliSeconds();
    g_rttSumMs += rttMs;
    g_rttSamples++;
    if (g_logging)
    {
        std::ofstream rttFile(g_outputDir + "harl-tcp-rtt.csv", std::ios_base::app);
        rttFile << Simulator::Now().GetSeconds() << "," << rttMs << std::endl;
    }
}

void
BbrPacingGainChange(double oldValue, double newValue)
{
    std::ofstream pacingGainFile(g_outputDir + "harl-tcp-pacing-gain.csv", std::ios_base::app);
    pacingGainFile << Simulator::Now().GetSeconds() << "," << newValue << std::endl;
}

void
BbrCwndGainChange(double oldValue, double newValue)
{
    std::ofstream cwndGainFile(g_outputDir + "harl-tcp-cwnd-gain.csv", std::ios_base::app);
    cwndGainFile << Simulator::Now().GetSeconds() << "," << newValue << std::endl;
}

void
TcpRateSampleChange(const TcpRateOps::TcpRateSample& sample)
{
    std::ofstream rateFile(g_outputDir + "harl-tcp-rate.csv", std::ios_base::app);
    rateFile << Simulator::Now().GetSeconds() << "," << sample.m_deliveryRate.GetBitRate()
             << std::endl;
}

// Track handovers
void
UavRrcStateChange(std::string context,
                  uint64_t imsi,
                  uint16_t cellId,
                  uint16_t rnti,
                  LteUeRrc::State oldState,
                  LteUeRrc::State newState)
{
    // Only log for UAV UE
    if (imsi != 1)
    {
        return;
    }
    std::cout << "RRC state change for UE " << imsi << ", RNTI " << rnti << ": " << oldState
              << " -> " << newState << " at time " << Simulator::Now().GetSeconds() << "s"
              << std::endl;
    g_currentRnti = rnti; // Update the global RNTI for the UAV UE
    g_currentCellId = cellId;

    // Detect RLF: UE drops from CONNECTED_NORMALLY to a non-connected state
    // (not due to a handover, which transitions through CONNECTED_HANDOVER).
    if (oldState == LteUeRrc::CONNECTED_NORMALLY &&
        newState != LteUeRrc::CONNECTED_HANDOVER &&
        g_tcpConnected)
    {
        g_rlfTriggered = true;
        g_tcpAlive = false;
        std::cout << "RLF detected at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    }
}

void
HandoverOk(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti)
{
    std::cout << "Handover OK for UE " << imsi << ", RNTI " << rnti << " to cell " << cellId
              << " at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_totalHandovers++;
    g_handoverInProgress = false;
    if (g_logging)
    {
        std::ofstream hoFile(g_outputDir + "harl-tcp-handovers.csv", std::ios_base::app);
        hoFile << Simulator::Now().GetSeconds() << "," << cellId << std::endl;
    }
}

void
HandoverError(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti)
{
    std::cout << "Handover FAILED for UE " << imsi << ", RNTI " << rnti << " at cell " << cellId
              << " time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_handoverInProgress = false;
}

void
ReportUeMeasurements(uint16_t cellId,
                     uint16_t rnti,
                     double rsrp,
                     double sinr,
                     uint8_t componentCarrierId)
{
    std::ofstream rsrpFile(g_outputDir + "rsrp_sinr.csv", std::ios_base::app);
    double sinrDb = 10 * std::log10(sinr);
    rsrpFile << Simulator::Now().GetSeconds() << "," << (int)cellId << "," << (int)rnti << ","
             << rsrp << "," << sinrDb << std::endl;
}

// UL SINR reported by the eNB (based on SRS) — logged per-RNTI
void
ReportUlSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint8_t componentCarrierId)
{
    if (rnti != g_currentRnti)
    {
        return;
    }
    if (cellId != g_currentCellId)
    {
        return;
    }
    std::ofstream ulSinrFile(g_outputDir + "ul_sinr.csv", std::ios_base::app);
    double sinrDb = 10 * std::log10(sinrLinear);
    ulSinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << sinrDb
               << std::endl;
}

void
ReportPhyTransmissionStatParameter(PhyTransmissionStatParameters param)
{
    std::ofstream mcsFile(g_outputDir + "mcs.csv", std::ios_base::app);
    mcsFile << Simulator::Now().GetSeconds() << "," << (int)param.m_mcs << std::endl;
}

void
ReportUeTxPower(uint16_t cellId, uint16_t rnti, double powerDbm)
{
    std::ofstream txPowerFile(g_outputDir + "ue_tx_power.csv", std::ios_base::app);
    txPowerFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << powerDbm
                << std::endl;
}

void
MobilityCourseChange(std::string context, Ptr<const MobilityModel> model)
{
    std::ofstream mobilityFile(g_outputDir + "mobility.csv", std::ios_base::app);
    mobilityFile << Simulator::Now().GetSeconds() << "," << model->GetPosition() << std::endl;
}

void
SinkRxPacket(Ptr<const Packet> packet, const Address& address)
{
    uint32_t size = packet->GetSize();
    g_totalRxBytes += size;
    if (g_logging)
    {
        std::ofstream sinkFile(g_outputDir + "sink-packets.csv", std::ios_base::app);
        sinkFile << Simulator::Now().GetSeconds() << "," << size << std::endl;
    }
}

void
SourceTxPacket(Ptr<const Packet> packet)
{
    std::ofstream sourceBulkSenderFile(g_outputDir + "source-packets.csv", std::ios_base::app);
    sourceBulkSenderFile << Simulator::Now().GetSeconds() << "," << packet->GetSize() << std::endl;
}

void
SourceRetransmissionPacket(const Ptr<const Packet> packet,
                           const TcpHeader& header,
                           const Address& localAddr,
                           const Address& peerAddr,
                           const Ptr<const TcpSocketBase> socket)
{
    g_totalRetransmissions++;
    if (g_logging)
    {
        std::ofstream retransmissionFile(g_outputDir + "retransmissions.csv", std::ios_base::app);
        retransmissionFile << Simulator::Now().GetSeconds() << "," << packet->GetSize() << std::endl;
    }
}

// ------------------------------------------------------------------------- //
// Helper: bounding box over eNB positions (used by hexgrid topology)
// ------------------------------------------------------------------------- //
struct BoundingBox
{
    double minX;
    double maxX;
    double minY;
    double maxY;
};

BoundingBox
ComputeEnbBoundingBox(NodeContainer enbNodes, double padding)
{
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Vector pos = enbNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        minX = std::min(minX, pos.x);
        maxX = std::max(maxX, pos.x);
        minY = std::min(minY, pos.y);
        maxY = std::max(maxY, pos.y);
    }
    minX -= padding;
    maxX += padding;
    minY -= padding;
    maxY += padding;

    return {minX, maxX, minY, maxY};
}

// ------------------------------------------------------------------------- //
// scenarioSetup
// ------------------------------------------------------------------------- //
inline void
scenarioSetup(double ueSpeed = 20.0,            // m/s
              double simDuration = 50.0,        // seconds
              double intersiteDistance = 500.0, // m
              double enbDowntilt = 10.0,        // degrees
              uint32_t seed = 0,                // Seed for RNG
              uint32_t runId = 0,
              std::string trialName = "1",
              std::string tcpVariant = "TcpBbr",
              std::string uavMobility = "random-mobility",
              std::string topology = "hexgrid",
              double startHeight = 80.0,
              double endHeight = 300.0,
              double helixRadius = 80.0,
              uint32_t bbrWindowLength = 10,
              uint32_t addStaticUes = 0,
              bool fullBufferInterference = false,
              double aerialUeRatio = 0.0,
              bool logging = true,
              bool rlMode = false,
              std::string handoverAlgorithm = "a3",
              uint32_t stepTime = 100,
              uint32_t delay = 0,
              double handoverPenalty = 0.1,
              double referenceRateBps = 50000000.0,
              double rttPenaltyWeight = 0.2,
              double minRttMs = 30.0,
              double tcpFailurePenalty = 5.0,
              double rlfPenalty = 10.0,
              double handoverMargin = 3.0)
{
    g_outputDir = pathToNs3 + "/contrib/defiance/examples/harl-tcp-prototype/output/";

    // Clear data files
    if (logging)
    {
        std::filesystem::create_directories(g_outputDir);
        for (auto f : {"harl-tcp-cwnd.csv",
                       "harl-tcp-handovers.csv",
                       "harl-tcp-rate.csv",
                       "harl-tcp-cwnd-gain.csv",
                       "harl-tcp-pacing-gain.csv",
                       "harl-tcp-rtt.csv",
                       "rsrp_sinr.csv",
                       "mcs.csv",
                       "mobility.csv",
                       "sink-packets.csv",
                       "source-packets.csv",
                       "retransmissions.csv",
                       "ul_sinr.csv",
                       "ue_tx_power.csv"})
        {
            std::ofstream fout(g_outputDir + f);
            // truncate on open
        }
    }

    g_lteHelper = CreateObject<LteHelper>();
    g_epcHelper = CreateObject<PointToPointEpcHelper>();
    g_lteHelper->SetEpcHelper(g_epcHelper);
    auto lteHelper = g_lteHelper;
    auto epcHelper = g_epcHelper;
    lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");

    // --- Handover algorithm ---
    if (handoverAlgorithm == "agent" || rlMode)
    {
        // RL controls handover; disable automatic handover
        lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
    }
    else if (handoverAlgorithm == "a3")
    {
        lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    }
    else if (handoverAlgorithm == "noop")
    {
        lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
    }
    else
    {
        NS_FATAL_ERROR("Unknown handover algorithm: " << handoverAlgorithm
                                                      << ". Use a3, noop, or agent.");
    }
    lteHelper->SetEnbAntennaModelType("ns3::ThreeGppAntennaModelOriented");
    lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel");
    // Add downtilt
    lteHelper->SetEnbAntennaModelAttribute(
        "Downtilt",
        DoubleValue(enbDowntilt)); // enbDowntilt degree downtilt, typical for macro eNodeBs

    // ---- UL Power Control (open-loop, fractional path loss compensation) ------- //
    // Aerial UEs have lower path loss (LOS) than ground UEs of the same distance.
    // Open-loop power control naturally backs off their transmit power:
    //   P_tx = min(Pcmax, 10*log10(RBs) + (PoNominal+PoUe) + alpha*PL)
    // With Po=-80, alpha=1.0:
    //   Aerial UE (PL=87 dB, 2 RBs): P_tx = 3 -80 + 87 = 10 dBm
    //   Ground UE (PL=112 dB, 2 RBs): P_tx = 3 -80 + 112 = 35 -> capped at 23 dBm
    // Absolute mode avoids the accumulation drain from TPC=0 in PfFfMacScheduler.
    Config::SetDefault("ns3::LteUePowerControl::AccumulationEnabled", BooleanValue(false));
    Config::SetDefault("ns3::LteUePowerControl::ClosedLoop", BooleanValue(true));
    Config::SetDefault("ns3::LteUePowerControl::Alpha", DoubleValue(1.0));
    Config::SetDefault("ns3::LteUePowerControl::PoNominalPusch", IntegerValue(-80));
    Config::SetDefault("ns3::LteUePowerControl::PoUePusch", IntegerValue(0));

    // ---- Fractional Frequency Reuse ------------------------------------------- //
    // Three-zone FFR: center (all RBs), medium (partial RBs), edge (reuse-3 RBs).
    // Aerial UEs (high RSRP, moderate RSRQ due to LOS interference) fall into the
    // medium zone, which uses a subset of RBs and lower UL power. This reduces
    // their interference to adjacent cells by 4-5 dB.
    lteHelper->SetFfrAlgorithmType("ns3::LteFfrSoftAlgorithm");
    lteHelper->SetFfrAlgorithmAttribute("CenterRsrqThreshold", UintegerValue(30));
    lteHelper->SetFfrAlgorithmAttribute("EdgeRsrqThreshold", UintegerValue(25));
    lteHelper->SetFfrAlgorithmAttribute("CenterAreaPowerOffset",
                                        UintegerValue(LteRrcSap::PdschConfigDedicated::dB_6));
    lteHelper->SetFfrAlgorithmAttribute("MediumAreaPowerOffset",
                                        UintegerValue(LteRrcSap::PdschConfigDedicated::dB_1dot77));
    lteHelper->SetFfrAlgorithmAttribute("EdgeAreaPowerOffset",
                                        UintegerValue(LteRrcSap::PdschConfigDedicated::dB3));
    lteHelper->SetFfrAlgorithmAttribute("CenterAreaTpc", UintegerValue(1));
    lteHelper->SetFfrAlgorithmAttribute("MediumAreaTpc", UintegerValue(2));
    lteHelper->SetFfrAlgorithmAttribute("EdgeAreaTpc", UintegerValue(3));

    // ---- LTE device configuration -------------------------------------- //
    double enbTxPowerDbm = 49.0; // dBm, corresponds to 10W, typical for macro eNodeBs
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    double ueTxPowerDbm = 23.0; // dBm, corresponds to 200mW, typical UE transmit power
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(ueTxPowerDbm));
    Config::SetDefault(
        "ns3::LteUePhy::NoiseFigure",
        DoubleValue(11.0)); // Noise figure in dB, increased to simulate external interference
    Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(80));
    Config::SetDefault("ns3::LteHelper::UsePdschForCqiGeneration", BooleanValue(false));

    // 3GPP TR 36.777 UMa-AV (Urban Macro - Aerial Vehicle)
    lteHelper->SetPathlossModelType(ThreeGppUmaAvPropagationLossModel::GetTypeId());
    lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.6e9)); // Corresponds to band 7
    lteHelper->SetPathlossModelAttribute("ShadowingEnabled", BooleanValue(true));
    Ptr<ThreeGppUmaAvChannelConditionModel> condModel =
        CreateObject<ThreeGppUmaAvChannelConditionModel>();
    lteHelper->SetPathlossModelAttribute("ChannelConditionModel", PointerValue(condModel));

    // Rooftop reflection model (TR 36.777 Alt 1): boosts signal when the
    // scattered path off building rooftops hits the eNB main lobe, while the
    // direct path goes through the sidelobe.
    lteHelper->SetFadingModel("ns3::RooftopReflectionLossModel");
    lteHelper->SetFadingModelAttribute("BuildingHeight", DoubleValue(20.0));
    lteHelper->SetFadingModelAttribute("Downtilt", DoubleValue(enbDowntilt));

    // Set the EARFCN (frequency band) and Bandwidth (in RBs)
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(3100));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(21100));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth",
                                     UintegerValue(100)); // Set DL bandwidth to 100 RBs
    lteHelper->SetEnbDeviceAttribute("UlBandwidth",
                                     UintegerValue(100)); // Set UL bandwidth to 100 RBs

    // Set the RLC UM and AM buffer size to match the BDP with headroom ~10% - 100% BDP
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(180000));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(180000));

    if (tcpVariant == "TcpNewReno")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpNewReno::GetTypeId()));
    }
    else if (tcpVariant == "TcpCubic")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpCubic::GetTypeId()));
    }
    else if (tcpVariant == "TcpBbr")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpBbr::GetTypeId()));
        Config::SetDefault("ns3::TcpBbr::BwWindowLength", UintegerValue(bbrWindowLength));
        Config::SetDefault("ns3::TcpBbr::RttWindowLength", TimeValue(Seconds(bbrWindowLength)));
        Config::SetDefault("ns3::TcpBbr::ProbeRttDuration", TimeValue(MilliSeconds(200)));
    }
    else if (tcpVariant == "TcpVegas")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpVegas::GetTypeId()));
    }
    else if (tcpVariant == "TcpWestwoodPlus")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                           TypeIdValue(TcpWestwoodPlus::GetTypeId()));
    }
    else if (tcpVariant == "TcpVeno")
    {
        Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpVeno::GetTypeId()));
    }
    else
    {
        NS_FATAL_ERROR("Unknown TCP variant: " << tcpVariant);
    }

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    g_remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = g_remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHost);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer interfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    g_uavContainer.Create(1);

    // ---- eNB setup: simple vs hexgrid ---------------------------------- //
    if (topology == "hexgrid")
    {
        uint32_t numMacroCells = 7;
        uint32_t nMacroEnbSitesX = 2;
        g_enbContainer.Create(3 * numMacroCells);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_enbContainer);

        Ptr<LteHexGridEnbTopologyHelper> hexHelper = CreateObject<LteHexGridEnbTopologyHelper>();
        hexHelper->SetLteHelper(lteHelper);
        hexHelper->SetAttribute("InterSiteDistance", DoubleValue(intersiteDistance));
        hexHelper->SetAttribute("SiteHeight", DoubleValue(25.0));
        hexHelper->SetAttribute("GridWidth", UintegerValue(nMacroEnbSitesX));
        hexHelper->SetAttribute("EnableWraparound", BooleanValue(true));

        g_enbLteDevs = hexHelper->SetPositionAndInstallEnbDevice(g_enbContainer);

        lteHelper->AddX2Interface(g_enbContainer);

        // ---- UAV helix mobility (hexgrid) ------------------------------ //
        BoundingBox bbox = ComputeEnbBoundingBox(g_enbContainer, intersiteDistance * 0.2);
        double uavCenterY = (bbox.minY + bbox.maxY) / 2.0;
        double midX = (bbox.minX + bbox.maxX) / 2.0;
        double halfSpanX = helixRadius;

        if (uavMobility == "helix")
        {
            double startZ = startHeight;
            double endZ = endHeight;
            double dwellTime = 2.0;
            double totalTravelDist = ueSpeed * (simDuration - dwellTime);

            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);

            const double dTheta = 2.0 * M_PI / 16.0;
            double cumDist = 0.0;
            double theta = 0.0;

            double x0 = midX - halfSpanX * std::cos(theta);
            double y0 = uavCenterY + helixRadius * std::sin(theta);
            double z0 = startZ;
            Vector prev(x0, y0, z0);
            wpMob->AddWaypoint(Waypoint(Seconds(0.0), prev));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), prev));

            while (cumDist < totalTravelDist)
            {
                theta += dTheta;
                double frac = cumDist / totalTravelDist;
                double x = midX - halfSpanX * std::cos(theta);
                double y = uavCenterY + helixRadius * std::sin(theta);
                double z = startZ + (endZ - startZ) * std::min(frac, 1.0);
                Vector pos(x, y, z);

                double segLen = CalculateDistance(pos, prev);
                double newCum = cumDist + segLen;
                double time =
                    (newCum > totalTravelDist) ? simDuration : dwellTime + newCum / ueSpeed;

                wpMob->AddWaypoint(Waypoint(Seconds(time), pos));
                cumDist = newCum;
                prev = pos;
            }

            // std::cout << "Helix path: X=[" << midX - halfSpanX << ", " << midX + halfSpanX
            //           << "], Y ~" << uavCenterY << " +/-" << helixRadius << ", Z=[" << startZ
            //           << ", " << endZ << "]"
            //           << ", speed=" << ueSpeed << " m/s, travelDist=" << totalTravelDist
            //           << " m, arc=" << cumDist << " m" << std::endl;
        }
        else if (uavMobility == "random-waypoint")
        {
            double dwellTime = 2.0;
            Ptr<UniformRandomVariable> rbx = CreateObject<UniformRandomVariable>();
            rbx->SetAttribute("Min", DoubleValue(bbox.minX));
            rbx->SetAttribute("Max", DoubleValue(bbox.maxX));
            Ptr<UniformRandomVariable> rby = CreateObject<UniformRandomVariable>();
            rby->SetAttribute("Min", DoubleValue(bbox.minY));
            rby->SetAttribute("Max", DoubleValue(bbox.maxY));
            Ptr<UniformRandomVariable> rbz = CreateObject<UniformRandomVariable>();
            rbz->SetAttribute("Min", DoubleValue(startHeight));
            rbz->SetAttribute("Max", DoubleValue(endHeight));

            Vector startPos((bbox.minX + bbox.maxX) / 2.0,
                            (bbox.minY + bbox.maxY) / 2.0,
                            startHeight);
            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);
            wpMob->AddWaypoint(Waypoint(Seconds(0.0), startPos));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), startPos));

            Vector currentPos = startPos;
            double currentTime = dwellTime;
            while (currentTime < simDuration) // generate beyond stop time
            {
                Vector nextPos;
                double dist;
                do
                {
                    nextPos = Vector(rbx->GetValue(), rby->GetValue(), rbz->GetValue());
                    dist = CalculateDistance(currentPos, nextPos);
                } while (dist < 20.0 || dist > 100.0); // waypoints 20-100m apart

                double travelTime = dist / ueSpeed;
                currentTime += travelTime;
                wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                currentPos = nextPos;
            }

            // std::cout << "Random waypoint path: bounding box X=[" << bbox.minX << ", " << bbox.maxX
            //           << "], Y=[" << bbox.minY << ", " << bbox.maxY << "], Z=[" << startHeight
            //           << ", " << endHeight << "], speed=" << ueSpeed << " m/s" << std::endl;
        }
        else // "constant"
        {
            MobilityHelper uavMob;
            uavMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            uavMob.Install(g_uavContainer);
            Ptr<ConstantPositionMobilityModel> uavPos =
                g_uavContainer.Get(0)->GetObject<ConstantPositionMobilityModel>();
            uavPos->SetPosition(Vector(midX - halfSpanX, uavCenterY, startHeight));
        }
    }
    else // "simple"
    {
        g_enbContainer.Create(2);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_enbContainer);
        Ptr<ConstantPositionMobilityModel> enbMobility1 =
            g_enbContainer.Get(0)->GetObject<ConstantPositionMobilityModel>();
        enbMobility1->SetPosition(Vector(0.0, 0.0, 25.0));
        Ptr<ConstantPositionMobilityModel> enbMobility2 =
            g_enbContainer.Get(1)->GetObject<ConstantPositionMobilityModel>();
        enbMobility2->SetPosition(Vector(500.0, 0.0, 25.0));

        double startX = intersiteDistance * 0.3;
        double endX = intersiteDistance * 0.7;
        double uavY = 0.0;

        if (uavMobility == "helix")
        {
            Ptr<WaypointMobilityModel> uavMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(uavMob);
            double legDistance = endX - startX;
            uavMob->AddWaypoint(Waypoint(Seconds(0.0), Vector(startX, uavY, startHeight)));

            double totalTravelDistance = simDuration * ueSpeed;
            uint32_t numLegs = static_cast<uint32_t>(std::ceil(totalTravelDistance / legDistance));
            for (uint32_t i = 1; i <= numLegs; ++i)
            {
                double time = i * simDuration / numLegs;
                double xPos = (i % 2 == 0) ? startX : endX;
                uavMob->AddWaypoint(Waypoint(Seconds(time), Vector(xPos, uavY, startHeight)));
            }
        }
        else if (uavMobility == "random-waypoint")
        {
            BoundingBox bbox = ComputeEnbBoundingBox(g_enbContainer, intersiteDistance * 0.2);
            double dwellTime = 2.0;
            Ptr<UniformRandomVariable> rbx = CreateObject<UniformRandomVariable>();
            rbx->SetAttribute("Min", DoubleValue(bbox.minX));
            rbx->SetAttribute("Max", DoubleValue(bbox.maxX));
            Ptr<UniformRandomVariable> rby = CreateObject<UniformRandomVariable>();
            rby->SetAttribute("Min", DoubleValue(bbox.minY));
            rby->SetAttribute("Max", DoubleValue(bbox.maxY));
            Ptr<UniformRandomVariable> rbz = CreateObject<UniformRandomVariable>();
            rbz->SetAttribute("Min", DoubleValue(startHeight));
            rbz->SetAttribute("Max", DoubleValue(endHeight));

            Vector startPos((bbox.minX + bbox.maxX) / 2.0,
                            (bbox.minY + bbox.maxY) / 2.0,
                            startHeight);
            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);
            wpMob->AddWaypoint(Waypoint(Seconds(0.0), startPos));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), startPos));

            Vector currentPos = startPos;
            double currentTime = dwellTime;
            while (currentTime < simDuration * 2) // generate far beyond stop time
            {
                Vector nextPos;
                double dist;
                do
                {
                    nextPos = Vector(rbx->GetValue(), rby->GetValue(), rbz->GetValue());
                    dist = CalculateDistance(currentPos, nextPos);
                } while (dist < 20.0 || dist > 100.0); // waypoints 20-100m apart

                double travelTime = dist / ueSpeed;
                currentTime += travelTime;
                wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                currentPos = nextPos;
            }

            std::cout << "Random waypoint path: bounding box X=[" << bbox.minX << ", " << bbox.maxX
                      << "], Y=[" << bbox.minY << ", " << bbox.maxY << "], Z=[" << startHeight
                      << ", " << endHeight << "], speed=" << ueSpeed << " m/s" << std::endl;
        }
        else // "constant"
        {
            MobilityHelper uavMob;
            uavMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            uavMob.Install(g_uavContainer);
            Ptr<ConstantPositionMobilityModel> uavPos =
                g_uavContainer.Get(0)->GetObject<ConstantPositionMobilityModel>();
            uavPos->SetPosition(Vector(startX, uavY, startHeight));
        }
    }

    RngSeedManager::SetSeed(seed == 0U ? time(nullptr) : seed);
    RngSeedManager::SetRun(runId == 0U ? 1 : runId);

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(46.0));
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(23.0));

    if (topology == "simple")
    {
        lteHelper->SetFfrAlgorithmAttribute("FrCellTypeId", UintegerValue(1));
        lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(0.0));
        g_enbLteDevs.Add(lteHelper->InstallEnbDevice(g_enbContainer.Get(0)));

        lteHelper->SetFfrAlgorithmAttribute("FrCellTypeId", UintegerValue(2));
        lteHelper->SetEnbAntennaModelAttribute("Orientation", DoubleValue(180.0));
        g_enbLteDevs.Add(lteHelper->InstallEnbDevice(g_enbContainer.Get(1)));
    }

    g_uavLteDevs = lteHelper->InstallUeDevice(g_uavContainer);

    internet.Install(g_uavContainer);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(g_uavLteDevs));
    if (topology == "hexgrid")
    {
        lteHelper->AttachToClosestEnb(g_uavLteDevs.Get(0), g_enbLteDevs);
    }
    else
    {
        lteHelper->Attach(g_uavLteDevs.Get(0), g_enbLteDevs.Get(0));
    }

    // ---- Full-buffer interfering UEs ---------------------------------------- //
    if (fullBufferInterference)
    {
        uint32_t numEnbs = g_enbContainer.GetN();

        g_interferingRemoteHostContainer.Create(1);
        Ptr<Node> intRemoteHost = g_interferingRemoteHostContainer.Get(0);
        InternetStackHelper intInternet;
        intInternet.Install(intRemoteHost);

        PointToPointHelper p2phInt;
        p2phInt.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        p2phInt.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2phInt.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
        NetDeviceContainer intInternetDevices = p2phInt.Install(pgw, intRemoteHost);
        Ipv4AddressHelper ipv4hInt;
        ipv4hInt.SetBase("3.0.0.0", "255.0.0.0");
        Ipv4InterfaceContainer intRemoteInterfaces = ipv4hInt.Assign(intInternetDevices);

        Ptr<Ipv4StaticRouting> intRemoteHostRouting =
            ipv4RoutingHelper.GetStaticRouting(intRemoteHost->GetObject<Ipv4>());
        intRemoteHostRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

        g_interferingUeContainer.Create(numEnbs);
        MobilityHelper intUeMob;
        intUeMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        intUeMob.Install(g_interferingUeContainer);

        Ptr<UniformRandomVariable> intUeHeightRng = CreateObject<UniformRandomVariable>();

        // std::cout << "Creating " << numEnbs
        //           << " interfering UEs (one per eNB, aerialRatio=" << aerialUeRatio
        //           << ", aerial height in [50, 300] m)" << std::endl;
        for (uint32_t i = 0; i < numEnbs; ++i)
        {
            Ptr<LteEnbNetDevice> enbDev = g_enbLteDevs.Get(i)->GetObject<LteEnbNetDevice>();
            Vector enbPos = enbDev->GetNode()->GetObject<MobilityModel>()->GetPosition();

            Ptr<ThreeGppAntennaModelOriented> antenna = DynamicCast<ThreeGppAntennaModelOriented>(
                enbDev->GetPhy()->GetDlSpectrumPhy()->GetAntenna());
            double orientation = antenna ? antenna->GetOrientation() : 0.0;

            double rad = orientation * M_PI / 180.0;
            Vector dir(std::cos(rad), std::sin(rad), 0.0);
            double ueHeight = (intUeHeightRng->GetValue() < aerialUeRatio)
                                  ? (50.0 + intUeHeightRng->GetValue() * 250.0)
                                  : 1.5;
            Vector uePos = enbPos + (100.0 * dir);
            uePos.z = ueHeight;
            g_interferingUeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(uePos);
        }

        g_interferingUeLteDevs = lteHelper->InstallUeDevice(g_interferingUeContainer);
        internet.Install(g_interferingUeContainer);
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(g_interferingUeLteDevs));

        uint16_t intPort = 60000;
        for (uint32_t i = 0; i < numEnbs; ++i)
        {
            lteHelper->Attach(g_interferingUeLteDevs.Get(i), g_enbLteDevs.Get(i));

            Ptr<Node> intUe = g_interferingUeContainer.Get(i);
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(intUe->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

            uint16_t port = intPort + i;

            Ipv4Address intRemoteAddr = intRemoteInterfaces.GetAddress(1);
            OnOffHelper ulTraffic("ns3::TcpSocketFactory", InetSocketAddress(intRemoteAddr, port));
            ulTraffic.SetAttribute("DataRate", DataRateValue(DataRate("100Kbps")));
            ulTraffic.SetAttribute("PacketSize", UintegerValue(512));
            ulTraffic.SetAttribute("OnTime",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            ulTraffic.SetAttribute("OffTime",
                                   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            auto ulApp = ulTraffic.Install(intUe);
            ulApp.Start(Seconds(1.0 + i * 0.1));

            PacketSinkHelper ulSink("ns3::TcpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
            auto ulSinkApp = ulSink.Install(intRemoteHost);
            ulSinkApp.Start(Seconds(1.0 + i * 0.1));

            Vector pos = g_enbContainer.Get(i)->GetObject<MobilityModel>()->GetPosition();
            // std::cout << "  Interfering UE " << i << " attached to eNB " << i << " at (" << pos.x
            //           << ", " << pos.y << ")" << std::endl;
        }
    }

    // ---- Static UEs (background TCP UL traffic) ------------------------ //
    if (addStaticUes > 0)
    {
        // Compute bounding box of eNB positions with padding
        BoundingBox bbox = ComputeEnbBoundingBox(g_enbContainer, intersiteDistance * 0.15);
        Ptr<UniformRandomVariable> staticUePosRng = CreateObject<UniformRandomVariable>();
        staticUePosRng->SetAttribute("Min", DoubleValue(0.0));
        staticUePosRng->SetAttribute("Max", DoubleValue(1.0));

        g_staticRemoteHostContainer.Create(1);
        Ptr<Node> staticRemoteHost = g_staticRemoteHostContainer.Get(0);
        InternetStackHelper staticInternet;
        staticInternet.Install(staticRemoteHost);

        PointToPointHelper p2phStatic;
        p2phStatic.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        p2phStatic.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2phStatic.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
        NetDeviceContainer staticInternetDevices = p2phStatic.Install(pgw, staticRemoteHost);
        Ipv4AddressHelper ipv4hStatic;
        ipv4hStatic.SetBase("2.0.0.0", "255.0.0.0");
        Ipv4InterfaceContainer staticRemoteInterfaces = ipv4hStatic.Assign(staticInternetDevices);

        Ptr<Ipv4StaticRouting> staticRemoteHostRouting =
            ipv4RoutingHelper.GetStaticRouting(staticRemoteHost->GetObject<Ipv4>());
        staticRemoteHostRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                   Ipv4Mask("255.0.0.0"),
                                                   1);

        g_staticUeContainer.Create(addStaticUes);
        MobilityHelper staticUeMob;
        staticUeMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        staticUeMob.Install(g_staticUeContainer);

        Ptr<UniformRandomVariable> staticUeRng = CreateObject<UniformRandomVariable>();
        staticUeRng->SetAttribute("Min", DoubleValue(0.0));
        staticUeRng->SetAttribute("Max", DoubleValue(1.0));
        uint32_t numAerial = 0;
        for (uint32_t i = 0; i < addStaticUes; ++i)
        {
            double x = bbox.minX + staticUeRng->GetValue() * (bbox.maxX - bbox.minX);
            double y = bbox.minY + staticUeRng->GetValue() * (bbox.maxY - bbox.minY);
            double ueHeight = (staticUeRng->GetValue() < aerialUeRatio)
                                  ? (50.0 + staticUeRng->GetValue() * 250.0)
                                  : 1.5;
            if (ueHeight > 1.5)
            {
                numAerial++;
            }
            g_staticUeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(
                Vector(x, y, ueHeight));
            // std::cout << "Static UE " << i << " position: (" << x << ", " << y << ", " << ueHeight
            //           << ")" << std::endl;
        }
        // std::cout << "Static UEs: " << numAerial << " aerial, " << (addStaticUes - numAerial)
        //           << " ground" << std::endl;

        g_staticUeLteDevs = lteHelper->InstallUeDevice(g_staticUeContainer);
        internet.Install(g_staticUeContainer);
        Ipv4InterfaceContainer staticUeIfaces =
            epcHelper->AssignUeIpv4Address(NetDeviceContainer(g_staticUeLteDevs));

        lteHelper->AttachToClosestEnb(g_staticUeLteDevs, g_enbLteDevs);

        uint16_t staticPort = 50001;
        for (uint32_t i = 0; i < addStaticUes; ++i)
        {
            Ptr<Node> staticUe = g_staticUeContainer.Get(i);
            uint16_t port = staticPort + i;

            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(staticUe->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

            OnOffHelper ulTcp("ns3::TcpSocketFactory",
                              InetSocketAddress(staticRemoteInterfaces.GetAddress(1), port));
            ulTcp.SetAttribute("DataRate", DataRateValue(DataRate("100Kbps")));
            ulTcp.SetAttribute("PacketSize", UintegerValue(512));
            ulTcp.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            ulTcp.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            auto ulApp = ulTcp.Install(staticUe);
            ulApp.Start(Seconds(1.0 + i * 0.1));

            PacketSinkHelper ulSink("ns3::TcpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
            auto ulSinkApp = ulSink.Install(staticRemoteHost);
            ulSinkApp.Start(Seconds(1.0 + i * 0.1));
        }
    }

    // Add measurement configuration
    LteRrcSap::ReportConfigEutra reportConfig;
    reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS120;
    for (auto it = g_enbLteDevs.Begin(); it != g_enbLteDevs.End(); ++it)
    {
        Ptr<NetDevice> netDevice = *it;
        Ptr<LteEnbNetDevice> enbLteNetDevice = netDevice->GetObject<LteEnbNetDevice>();
        Ptr<LteEnbRrc> enbRrc = enbLteNetDevice->GetRrc();
        enbRrc->AddUeMeasReportConfig(reportConfig);
    }

    uint16_t dlPort = 50000;
    Address remoteAddress(InetSocketAddress(interfaces.GetAddress(1), dlPort));

    Ptr<Node> uav = g_uavContainer.Get(0);
    Ptr<Ipv4StaticRouting> uavStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(uav->GetObject<Ipv4>());
    uavStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    PacketSinkHelper remotePacketSinkHelper("ns3::TcpSocketFactory", remoteAddress);
    auto remoteSink = remotePacketSinkHelper.Install(remoteHost);

    BulkSendHelper bulkSendHelper("ns3::TcpSocketFactory", remoteAddress);
    bulkSendHelper.SetAttribute("SendSize", UintegerValue(1024));
    auto uavApp = bulkSendHelper.Install(uav);
    uavApp.Start(Seconds(1.0));
    remoteSink.Start(Seconds(1.0));

    uint32_t uavNodeId = uav->GetId();
    uint32_t remoteHostNodeId = remoteHost->GetId();
    std::string bulkSenderConfigPath =
        "/NodeList/" + std::to_string(uavNodeId) + "/ApplicationList/*/$ns3::BulkSendApplication/";

    Config::ConnectWithoutContext(bulkSenderConfigPath + "ConnectionFailed",
                                  MakeCallback((&NotifyConnectionFailed)));
    Config::ConnectWithoutContext(bulkSenderConfigPath + "ConnectionSucceeded",
                                  MakeCallback((&NotifyConnectionSucceeded)));

    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/LteUeRrc/HandoverEndOk",
                                  MakeCallback(&HandoverOk));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/LteUeRrc/HandoverEndError",
                                  MakeCallback(&HandoverError));

    Simulator::Schedule(Seconds(1.1), []() {
        Ptr<Node> uav = g_uavContainer.Get(0);
        auto ueDev = uav->GetDevice(0)->GetObject<LteUeNetDevice>();
        auto rrc = ueDev->GetRrc();
        auto ipv4 = uav->GetObject<Ipv4>();
        std::cout << "DEBUG t=" << Simulator::Now().GetSeconds() << " rrcState=" << rrc->GetState()
                  << " cellId=" << rrc->GetCellId() << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal()
                  << std::endl;
    });
    // Schedule metric-accumulating traces (always connected, regardless of logging)
    Simulator::Schedule(Seconds(1.1), [uavNodeId, remoteHostNodeId, bulkSenderConfigPath]() {
        std::string retransPath = "/NodeList/" + std::to_string(uavNodeId) +
                                  "/$ns3::TcpL4Protocol/SocketList/0/Retransmission";
        Config::ConnectWithoutContext(retransPath, MakeCallback(&SourceRetransmissionPacket));

        std::string rxPath = "/NodeList/" + std::to_string(remoteHostNodeId) +
                             "/ApplicationList/*/$ns3::PacketSink/Rx";
        Config::ConnectWithoutContext(rxPath, MakeCallback(&SinkRxPacket));

        std::string rttPath =
            "/NodeList/" + std::to_string(uavNodeId) + "/$ns3::TcpL4Protocol/SocketList/0/RTT";
        Config::ConnectWithoutContext(rttPath, MakeCallback(&TcpRttChange));

        std::string tcpStatePath =
            "/NodeList/" + std::to_string(uavNodeId) + "/$ns3::TcpL4Protocol/SocketList/0/State";
        Config::ConnectWithoutContext(tcpStatePath,
                                      MakeCallback(&NotifyTcpStateChange));
    });

    if (logging)
    {
        Simulator::Schedule(Seconds(1.1), [uavNodeId, remoteHostNodeId, bulkSenderConfigPath]() {
            std::string path = "/NodeList/" + std::to_string(uavNodeId) +
                               "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow";
            Config::ConnectWithoutContext(path, MakeCallback(&CwndTracer));

            Config::ConnectWithoutContext(bulkSenderConfigPath + "Tx",
                                          MakeCallback(&SourceTxPacket));
        });

        Config::Connect("/NodeList/" + std::to_string(uavNodeId) +
                            "/$ns3::MobilityModel/CourseChange",
                        MakeCallback(&MobilityCourseChange));

        if (tcpVariant == "TcpBbr")
        {
            Simulator::Schedule(Seconds(1.1), [uavNodeId]() {
                std::string pacingGainPath =
                    "/NodeList/" + std::to_string(uavNodeId) +
                    "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/PacingGain";
                Config::ConnectWithoutContext(pacingGainPath, MakeCallback(&BbrPacingGainChange));

                std::string cwndGainPath =
                    "/NodeList/" + std::to_string(uavNodeId) +
                    "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/CwndGain";
                Config::ConnectWithoutContext(cwndGainPath, MakeCallback(&BbrCwndGainChange));

                std::string ratePath =
                    "/NodeList/" + std::to_string(uavNodeId) +
                    "/$ns3::TcpL4Protocol/SocketList/0/RateOps/TcpRateSampleUpdated";
                Config::ConnectWithoutContext(ratePath, MakeCallback(&TcpRateSampleChange));
            });
        }

        Config::ConnectWithoutContext(
            "/NodeList/" + std::to_string(uavNodeId) +
                "/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
            MakeCallback(&ReportUeMeasurements));

        Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                          "/DeviceList/*/$ns3::LteUeNetDevice/"
                                          "ComponentCarrierMapUe/*/LteUePhy/UlPhyTransmission",
                                      MakeCallback(&ReportPhyTransmissionStatParameter));

        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/LteEnbPhy/ReportUeSinr",
            MakeCallback(&ReportUlSinr));

        Simulator::Schedule(Seconds(1.1), [uavNodeId]() {
            Ptr<Node> uavNode = NodeList::GetNode(uavNodeId);
            Ptr<LteUeNetDevice> ueNetDev = uavNode->GetDevice(0)->GetObject<LteUeNetDevice>();
            if (ueNetDev)
            {
                Ptr<LteUePhy> uePhy = ueNetDev->GetPhy();
                Ptr<LteUePowerControl> powerCtrl = uePhy->GetUplinkPowerControl();
                powerCtrl->TraceConnectWithoutContext("ReportPuschTxPower",
                                                      MakeCallback(&ReportUeTxPower));
            }
        });

        Config::Connect("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteUeNetDevice/LteUeRrc"
                        "/StateTransition",
                        MakeCallback(&UavRrcStateChange));
    }

    // Add X2 Interface (already added inside hexgrid block)
    if (topology == "simple")
    {
        lteHelper->AddX2Interface(g_enbContainer);
    }

    // ---- RL Framework: Install handover RL apps on UAV node ---- //
    if (rlMode)
    {
        uint32_t numBs = g_enbContainer.GetN();
        uint32_t uavNodeId = g_uavContainer.Get(0)->GetId();
        g_lastRsrpValues.resize(numBs + 1, -1); // index by cellId (1-based)

        // std::cout << "Installing RL handover apps (NumBs=" << numBs << ", StepTime=" << stepTime
        //           << "ms"
        //           << ", delay=" << delay << "ms"
        //           << ", handoverPenalty=" << handoverPenalty << ", uavNodeId=" << uavNodeId << ")"
        //           << std::endl;

        uint32_t remoteHostNodeId = g_remoteHostContainer.Get(0)->GetId();

        RlApplicationHelper rlAppHelper(HarlTcpHandoverRewardApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
        rlAppHelper.SetAttribute("StopTime", TimeValue(Seconds(simDuration)));
        rlAppHelper.SetAttribute("RemoteHostNodeId", UintegerValue(remoteHostNodeId));
        rlAppHelper.SetAttribute("HandoverPenalty", DoubleValue(handoverPenalty));
        rlAppHelper.SetAttribute("ReferenceRate", DoubleValue(referenceRateBps));
        rlAppHelper.SetAttribute("RttPenaltyWeight", DoubleValue(rttPenaltyWeight));
        rlAppHelper.SetAttribute("MinRttMs", DoubleValue(minRttMs));
        rlAppHelper.SetAttribute("TcpFailurePenalty", DoubleValue(tcpFailurePenalty));
        rlAppHelper.SetAttribute("RlfPenalty", DoubleValue(rlfPenalty));
        auto rewardApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(HarlTcpHandoverAgentApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("NumUes", UintegerValue(1));
        auto agentApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(HarlTcpHandoverObservationApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("UavNodeId", UintegerValue(uavNodeId));
        auto obsApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(HarlTcpHandoverActionApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("HandoverAlgorithm", StringValue("agent"));
        rlAppHelper.SetAttribute("HandoverMargin", DoubleValue(handoverMargin));
        auto actApps = rlAppHelper.Install(g_uavContainer.Get(0));

        // --- CommunicationHelper: wire the apps together ---
        CommunicationHelper commHelper;
        commHelper.SetAgentApps(agentApps);
        commHelper.SetActionApps(actApps);
        commHelper.SetObservationApps(obsApps);
        commHelper.SetRewardApps(rewardApps);
        commHelper.SetIds();

        // Connect obs(0) -> agent(0), act(0) -> agent(0), reward(0) -> agent(0)
        // Since everything is on the UAV node, we use the configured delay
        commHelper.AddCommunication(
            {CommunicationPair{obsApps.GetId(0),
                               agentApps.GetId(0),
                               CommunicationAttributes{MilliSeconds(delay)}}});
        commHelper.AddCommunication(
            {CommunicationPair{actApps.GetId(0),
                               agentApps.GetId(0),
                               CommunicationAttributes{MilliSeconds(delay)}}});
        commHelper.AddCommunication(
            {CommunicationPair{rewardApps.GetId(0),
                               agentApps.GetId(0),
                               CommunicationAttributes{MilliSeconds(delay)}}});

        commHelper.Configure();

        // std::cout << "RL handover apps installed on UAV node " << uavNodeId << std::endl;
    }
}
