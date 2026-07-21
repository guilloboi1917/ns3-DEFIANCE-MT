/**
 * Setup file for the HARL TCP Scenario
 * Supports two topologies:
 *   "simple"  — 2 eNodeBs on a line, UAV shuttles between them
 *   "hexgrid" — hexagonal grid of 3-sector macro sites, UAV follows ascend-random or
 * random-waypoint 1 aerial UE, 1 remote server. The UE runs a TCP BulkSendApplication to the remote
 * server.
 */

#include "handover-nr/nr-rl-handover-act-app.h"
#include "handover-nr/nr-rl-handover-agent-app.h"
#include "handover-nr/nr-rl-handover-obs-app.h"
#include "handover-nr/nr-rl-handover-rwd-app.h"

#include "ns3/applications-module.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-value.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rl-application-helper.h"
#include "ns3/tcp-bbr.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-rate-ops.h"
#include "ns3/tcp-socket-base.h"

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

NS_LOG_COMPONENT_DEFINE("NrRlHandoverScenarioSetup");

int16_t g_currentRnti = 1;
int16_t g_currentCellId = 1;

// Application Connection succeed/fail callbacks
extern std::string g_flowDirection;
extern uint32_t g_senderNodeId;
extern uint32_t g_receiverNodeId;
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
NotifyTcpStateChange(const TcpSocket::TcpStates_t oldState, const TcpSocket::TcpStates_t newState)
{
    if (newState == TcpSocket::CLOSED || newState == TcpSocket::LAST_ACK)
    {
        g_tcpAlive = false;
        std::cout << "TCP socket closed (state=" << newState << ") at time "
                  << Simulator::Now().GetSeconds() << "s" << std::endl;
    }
}

// CWND tracing callback — fires on every CWND change (every ACK)
void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    std::ofstream cwndFile(g_outputDir + "nr-rl-cwnd.csv", std::ios_base::app);
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
        std::ofstream rttFile(g_outputDir + "nr-rl-rtt.csv", std::ios_base::app);
        rttFile << Simulator::Now().GetSeconds() << "," << rttMs << std::endl;
    }
}

void
BbrPacingGainChange(double oldValue, double newValue)
{
    std::ofstream pacingGainFile(g_outputDir + "nr-rl-pacing-gain.csv", std::ios_base::app);
    pacingGainFile << Simulator::Now().GetSeconds() << "," << newValue << std::endl;
}

void
BbrCwndGainChange(double oldValue, double newValue)
{
    std::ofstream cwndGainFile(g_outputDir + "nr-rl-cwnd-gain.csv", std::ios_base::app);
    cwndGainFile << Simulator::Now().GetSeconds() << "," << newValue << std::endl;
}

void
TcpRateSampleChange(const TcpRateOps::TcpRateSample& sample)
{
    std::ofstream rateFile(g_outputDir + "nr-rl-rate.csv", std::ios_base::app);
    rateFile << Simulator::Now().GetSeconds() << "," << sample.m_deliveryRate.GetBitRate()
             << std::endl;
}

// Track handovers
void
UavRrcStateChange(std::string context,
                  uint64_t imsi,
                  uint16_t cellId,
                  uint16_t rnti,
                  NrUeRrc::State oldState,
                  NrUeRrc::State newState)
{
    // Only log for UAV UE
    if (imsi != 1)
    {
        return;
    }
    std::cout << "RRC state change for UE " << imsi << ", RNTI " << rnti << " to cell "
              << cellId << " (state " << oldState << " -> " << newState << ") at time "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_currentRnti = rnti;
    g_currentCellId = cellId;
}

void
HandoverOk(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti)
{
    g_handoverInProgress = false;
    if (g_logging)
    {
        std::cout << "Handover OK for UE " << imsi << ", RNTI " << rnti << " to cell " << cellId
                  << " at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
        std::ofstream hoFile(g_outputDir + "nr-rl-handovers.csv", std::ios_base::app);
        hoFile << Simulator::Now().GetSeconds() << "," << cellId << std::endl;
    }

    // only increment when not using the rl app
    if (rlMode == false)
    {
        g_totalHandovers++;
    }
}

void
HandoverError(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti)
{
    std::cout << "Handover FAILED for UE " << imsi << ", RNTI " << rnti << " at cell " << cellId
              << " time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_handoverInProgress = false;
}

// Log ReportUeMeasurements (200ms, averaged, dBm/dB, all cells)
// This is the same trace source used by the obs app.
void
LogUeMeasReport(uint16_t rnti,
                uint16_t cellId,
                double rsrp,
                double rsrq,
                bool isServingCell,
                uint8_t componentCarrierId)
{
    std::ofstream ueMeasFile(g_outputDir + "ue_meas_report.csv", std::ios_base::app);
    ueMeasFile << Simulator::Now().GetSeconds() << "," << (int)cellId << "," << (int)rnti << ","
               << rsrp << "," << rsrq << "," << (int)isServingCell << std::endl;
}

// DL SINR (UE PHY DlDataSinr) logged as ul_sinr.csv for plot compatibility
void
ReportDlSinr(uint16_t cellId, uint16_t rnti, double sinrLinear, uint16_t bwpId)
{
    // DL SINR trace (UE PHY DlDataSinr) logged as ul_sinr.csv for plot compatibility.
    double sinrDb = (sinrLinear > 0.0) ? (10.0 * std::log10(sinrLinear)) : -40.0;
    std::ofstream ulSinrFile(g_outputDir + "ul_sinr.csv", std::ios_base::app);
    ulSinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti
               << "," << sinrDb << std::endl;
}

void
ReportUlSinr(uint64_t imsi, SpectrumValue& sinrSpectrum, SpectrumValue& /* interferenceSpectrum */)
{
    // Average SpectrumValue over RBs to get scalar SINR in dB
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
    if (numRb == 0) return;
    double sinrDb = sumSinr / static_cast<double>(numRb);
    std::ofstream ulSinrFile(g_outputDir + "ul_sinr.csv", std::ios_base::app);
    ulSinrFile << Simulator::Now().GetSeconds() << "," << g_currentCellId << "," << g_currentRnti
               << "," << sinrDb << std::endl;
}

// UL Interference at eNB
void
ReportInterference(uint16_t cellId, Ptr<SpectrumValue> spectrumValue)
{
    // if (cellId != g_currentCellId)
    // {
    //     return;
    // }
    std::cout << Simulator::Now().GetSeconds() << "," << cellId << "," << Sum(*spectrumValue)
              << std::endl;
}

void
ReportUlTbs(uint64_t imsi, uint64_t tbSize)
{
    std::ofstream mcsFile(g_outputDir + "mcs.csv", std::ios_base::app);
    mcsFile << Simulator::Now().GetSeconds() << "," << tbSize
            << std::endl;
}

void
ReportUeTxPower(uint16_t cellId, uint16_t rnti, double powerDbm)
{
    std::ofstream txPowerFile(g_outputDir + "ue_tx_power.csv", std::ios_base::app);
    txPowerFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << powerDbm
                << std::endl;
}

void
UavPeriodicPositionLog()
{
    if (g_uavContainer.GetN() > 0)
    {
        Ptr<MobilityModel> mob = g_uavContainer.Get(0)->GetObject<MobilityModel>();
        if (mob)
        {
            std::ofstream mobilityFile(g_outputDir + "mobility.csv", std::ios_base::app);
            mobilityFile << Simulator::Now().GetSeconds() << "," << mob->GetPosition()
                         << std::endl;
        }
    }
    // Re-schedule every 200ms (matches ReportUeMeasurements cadence)
    Simulator::Schedule(MilliSeconds(200), &UavPeriodicPositionLog);
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
        retransmissionFile << Simulator::Now().GetSeconds() << "," << packet->GetSize()
                           << std::endl;
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
scenarioSetup(std::string flowDirection = "ul",
              double ueSpeed = 20.0,            // m/s
              double simDuration = 80.0,        // seconds
              double intersiteDistance = 500.0, // m
              uint32_t numMacroCells = 7,       // number of macro sites
              double enbDowntilt = 10.0,        // degrees
              uint32_t seed = 0,                // Seed for RNG
              uint32_t runId = 0,
              std::string trialName = "1",
              std::string tcpVariant = "TcpBbr",
              std::string uavMobility = "ascend-random",
              std::string topology = "hexgrid",
              double startHeight = 80.0,
              double endHeight = 300.0,
              uint32_t bbrWindowLength = 10,
              uint32_t addStaticUes = 0,
              bool fullBufferInterference = false,
              double aerialUeRatio = 0.0,
              bool logging = true,
              bool rlMode = false,
              std::string handoverAlgorithm = "a3",
              uint32_t stepTime = 200,
              uint32_t delay = 0,
              double handoverPenalty = 0.1,
              double referenceRateBps = 5000000.0,
              double minAcceptableGoodputBps = 2500000.0,
              double delayMinRttMs = 40.0,
              double minAcceptableRttMs = 100.0,
              double tcpFailurePenalty = 5.0,
              double rlfPenalty = 10.0,
              double handoverMargin = 3.0)
{
    g_outputDir = pathToNs3 + "/contrib/defiance/examples/nr-rl-handover/output/";

    // Travel leg constraints for UAV waypoint generation
    const double travelLegMin = 30.0;
    const double travelLegMax = 100.0;

    // Seed RNG before any random operations
    RngSeedManager::SetSeed(seed == 0U ? time(nullptr) : seed);
    RngSeedManager::SetRun(runId == 0U ? 1 : runId);

    // Clear data files
    if (logging)
    {
        std::filesystem::create_directories(g_outputDir);
        for (auto f : {"nr-rl-cwnd.csv",
                       "nr-rl-handovers.csv",
                       "nr-rl-rate.csv",
                       "nr-rl-cwnd-gain.csv",
                       "nr-rl-pacing-gain.csv",
                       "nr-rl-rtt.csv",
                       "rsrp_sinr.csv",
                       "ue_meas_report.csv",
                       "mcs.csv",
                       "mobility.csv",
                       "sink-packets.csv",
                       "source-packets.csv",
                       "retransmissions.csv",
                       "ul_sinr.csv",
                       "ue_tx_power.csv",
                       "rl_obs.csv",
                       "rl_reward.csv",
                       "rl_action.csv"})
        {
            std::ofstream fout(g_outputDir + f);
            // truncate on open
        }
    }

    // ---- NR Helper Setup ---------------------------------------------------- //
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    g_nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    g_nrChannelHelper = CreateObject<NrChannelHelper>();
    g_nrHelper = CreateObject<NrHelper>();
    g_nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    g_nrHelper->SetEpcHelper(g_nrEpcHelper);

    // --- Spectrum: one band @ 3.5 GHz, 20 MHz, 1 CC, 1 BWP, numerology 0 --- //
    // Some common values from Switzerland //
    // Subcarrier spacing = 1 (numerology)
    // Bandwidth 100MHz matching ~n78 5G Band name
    // Center frequency 3.5GHz
    // However for faster iteration, use a smaller bandwidth and numerology
    const double centralFrequency = 3.5e9; // 3.5 GHz (FR1)
    const double bandwidth = 20e6;         // 20 MHz 
    const uint16_t numerology = 0;         // 15 kHz SCS
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // --- Channel: 3GPP TR 38.901 UMa (Urban Macro), default LOS condition --- //
    // NOTE: UpdatePeriod must be set on both ThreeGppChannelModel AND the channel condition model
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    g_nrChannelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");
    g_nrChannelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                         TimeValue(MilliSeconds(0)));
    g_nrChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    g_nrChannelHelper->AssignChannelsToBands({band});
    // UMa-AV (TR 38.901) is built into ThreeGppChannelModel but not exposed in
    // NrChannelHelper's enum. Override on the created channel objects.
    Config::Set("/ChannelList/*/$ns3::ThreeGppChannelModel/Scenario",
                StringValue("UMa-AV"));

    // --- Scheduler, error model, beamforming --- //
    g_nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));
    g_nrHelper->SetDlErrorModel("ns3::NrEesmIrT2");
    g_nrHelper->SetUlErrorModel("ns3::NrEesmIrT2");
    idealBeamformingHelper->SetAttribute(
        "BeamformingMethod",
        TypeIdValue(TypeId::LookupByName("ns3::DirectPathBeamforming")));

    // --- Handover algorithm ---
    if (handoverAlgorithm == "agent" || rlMode)
    {
        g_nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");
        Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(false));
    }
    else if (handoverAlgorithm == "a3")
    {
        g_nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
        // Use default hysteresis for A3 (handoverMargin is for the RL agent's action gate)
        double a3Hysteresis = 3.0;
        g_nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(a3Hysteresis));
        g_nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));
        Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(true));
    }
    else if (handoverAlgorithm == "noop")
    {
        g_nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");
        Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(false));
    }
    else
    {
        NS_FATAL_ERROR("Unknown handover algorithm: " << handoverAlgorithm
                                                      << ". Use a3, noop, or agent.");
    }

    // --- gNB antenna: UniformPlanarArray with 2x1 isotropic elements --- //
    // All NR examples use IsotropicAntennaModel as the antenna element (array provides beamforming).
    NrHelper::AntennaParams apGnb;
    apGnb.antennaElem = "ns3::IsotropicAntennaModel";
    apGnb.nAntCols = 4;
    apGnb.nAntRows = 2;
    apGnb.nHorizPorts = 1;
    apGnb.nVertPorts = 1;
    apGnb.isDualPolarized = false;
    apGnb.bearingAngle = 0.0;
    apGnb.polSlantAngle = 0.0;
    g_nrHelper->SetupGnbAntennas(apGnb);
    g_nrHelper->SetGnbAntennaAttribute("DowntiltAngle",
                                       DoubleValue(enbDowntilt * M_PI / 180.0));

    // --- UE antenna: single isotropic element --- //
    // Matches original LTE setup (no UE beamforming).
    NrHelper::AntennaParams apUe;
    apUe.antennaElem = "ns3::IsotropicAntennaModel";
    apUe.nAntCols = 4;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 1;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = false;
    apUe.bearingAngle = 0.0;
    apUe.polSlantAngle = 0.0;
    g_nrHelper->SetupUeAntennas(apUe);

    // --- UL Power Control (open-loop, fractional path loss compensation) --- //
    Config::SetDefault("ns3::NrUePowerControl::AccumulationEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrUePowerControl::ClosedLoop", BooleanValue(true));
    Config::SetDefault("ns3::NrUePowerControl::Alpha", DoubleValue(1.0));
    Config::SetDefault("ns3::NrUePowerControl::PoNominalPusch", IntegerValue(-80));
    Config::SetDefault("ns3::NrUePowerControl::PoUePusch", IntegerValue(0));

    // --- PHY configuration --- //
    g_nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    g_nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(49.0));
    g_nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
    g_nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(9.0));

    // --- RLC UM buffer size (matches legacy LTE setting from original prototype) --- //
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(180000));

    // --- BWP routing --- //
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
    uint32_t bwpId = 0;
    g_nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));
    g_nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));

    // --- Core network latency --- //
    g_nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(10)));

    // --- TCP variant configuration --- //
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
    // Set TCP ConnTimeout to be shorter, for quicker establishment (default 3s)
    Config::SetDefault("ns3::TcpSocket::ConnTimeout", TimeValue(Seconds(1)));

    Ptr<Node> pgw = g_nrEpcHelper->GetPgwNode();

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
        // Manual hexgrid: 1 ring = 7 sites, 3 sectors each = 21 gNBs
        // Site positions follow a hexagonal layout at intersiteDistance spacing.
        uint32_t nMacroEnbSitesX = numMacroCells == 7 ? 2 : 1;
        g_enbContainer.Create(3 * numMacroCells);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_enbContainer);

        // Site positions (hexagonal grid)
        // Site 0 at origin, sites 1-6 at 60-degree increments
        std::vector<Vector> sitePositions;
        sitePositions.push_back(Vector(0.0, 0.0, 25.0));
        for (uint32_t ring = 1; ring <= nMacroEnbSitesX; ++ring)
        {
            double radius = intersiteDistance * ring;
            for (uint32_t s = 0; s < 6; ++s)
            {
                double angle = s * M_PI / 3.0 + (ring % 2) * M_PI / 6.0;
                double x = radius * std::cos(angle);
                double y = radius * std::sin(angle);
                sitePositions.push_back(Vector(x, y, 25.0));
                if (sitePositions.size() >= numMacroCells)
                    break;
            }
            if (sitePositions.size() >= numMacroCells)
                break;
        }

        // Position gNBs and install devices (default bearing=0 for all sectors)
        uint32_t gNbIdx = 0;
        for (uint32_t site = 0; site < sitePositions.size() && gNbIdx < g_enbContainer.GetN(); ++site)
        {
            for (uint32_t sector = 0; sector < 3 && gNbIdx < g_enbContainer.GetN(); ++sector, ++gNbIdx)
            {
                Vector pos = sitePositions[site];
                g_enbContainer.Get(gNbIdx)->GetObject<MobilityModel>()->SetPosition(pos);
            }
        }
        g_gnbNrDevs = g_nrHelper->InstallGnbDevice(g_enbContainer, allBwps);

        // Add X2 interface for handover support
        g_nrHelper->AddX2Interface(g_enbContainer);

        // Update config after device installation
        // (NrGnbNetDevice::UpdateConfig is called inside InstallGnbDevice)

        // ---- UAV mobility (hexgrid) ------------------------------------ //
        BoundingBox bbox = ComputeEnbBoundingBox(g_enbContainer, intersiteDistance * 0.2);
        double uavCenterY = (bbox.minY + bbox.maxY) / 2.0;

        if (uavMobility == "ascend-random")
        {
            // (same as before — UAV waypoint logic unchanged)
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

            Vector startPos(rbx->GetValue(), rby->GetValue(), 1.5);
            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);
            wpMob->AddWaypoint(Waypoint(Seconds(0.0), startPos));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), startPos));
            double ascentTime = (startHeight - 1.5) / ueSpeed;
            Vector ascentEnd(startPos.x, startPos.y, startHeight);
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime + ascentTime), ascentEnd));
            Vector currentPos = ascentEnd;
            double currentTime = dwellTime + ascentTime;
            while (currentTime < simDuration * 2)
            {
                Vector nextPos;
                double dist;
                do
                {
                    nextPos = Vector(rbx->GetValue(), rby->GetValue(), rbz->GetValue());
                    dist = CalculateDistance(currentPos, nextPos);
                } while (dist < travelLegMin || dist > travelLegMax);
                double travelTime = dist / ueSpeed;
                currentTime += travelTime;
                wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                currentPos = nextPos;
            }
        }
        else if (uavMobility == "random-waypoint")
        {
            // (same UAV waypoint logic as before)
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

            Vector startPos(rbx->GetValue(), rby->GetValue(), startHeight);
            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);
            wpMob->AddWaypoint(Waypoint(Seconds(0.0), startPos));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), startPos));
            Vector currentPos = startPos;
            double currentTime = dwellTime;
            while (currentTime < simDuration * 2)
            {
                Vector nextPos;
                double dist;
                do
                {
                    nextPos = Vector(rbx->GetValue(), rby->GetValue(), rbz->GetValue());
                    dist = CalculateDistance(currentPos, nextPos);
                } while (dist < travelLegMin || dist > travelLegMax);
                double travelTime = dist / ueSpeed;
                currentTime += travelTime;
                wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                currentPos = nextPos;
            }
        }
        else // "constant"
        {
            MobilityHelper uavMob;
            uavMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            uavMob.Install(g_uavContainer);
            Ptr<ConstantPositionMobilityModel> uavPos =
                g_uavContainer.Get(0)->GetObject<ConstantPositionMobilityModel>();
            uavPos->SetPosition(Vector((bbox.minX + bbox.maxX) / 2.0, uavCenterY, startHeight));
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
        double uavY = 0.0;

        if (uavMobility == "ascend-random")
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

            Vector ground((bbox.minX + bbox.maxX) / 2.0, (bbox.minY + bbox.maxY) / 2.0, 1.5);

            Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
            g_uavContainer.Get(0)->AggregateObject(wpMob);

            wpMob->AddWaypoint(Waypoint(Seconds(0.0), ground));
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), ground));
            double ascentTime = (startHeight - 1.5) / ueSpeed;
            Vector ascentEnd(ground.x, ground.y, startHeight);
            wpMob->AddWaypoint(Waypoint(Seconds(dwellTime + ascentTime), ascentEnd));

            Vector currentPos = ascentEnd;
            double currentTime = dwellTime + ascentTime;
            while (currentTime < simDuration * 2)
            {
                Vector nextPos;
                double dist;
                do
                {
                    nextPos = Vector(rbx->GetValue(), rby->GetValue(), rbz->GetValue());
                    dist = CalculateDistance(currentPos, nextPos);
                } while (dist < travelLegMin || dist > travelLegMax);
                double travelTime = dist / ueSpeed;
                currentTime += travelTime;
                wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                currentPos = nextPos;
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
                } while (dist < travelLegMin || dist > travelLegMax);
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

    Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(46.0));
    Config::SetDefault("ns3::NrUePhy::TxPower", DoubleValue(23.0));

    if (topology == "simple")
    {
        // Simple topology: install gNBs with different bearing angles
        g_nrHelper->SetGnbAntennaAttribute("BearingAngle", DoubleValue(0.0)); // rad, pointing +x
        NetDeviceContainer gnb0 = g_nrHelper->InstallGnbDevice(NodeContainer(g_enbContainer.Get(0)), allBwps);
        g_gnbNrDevs.Add(gnb0.Get(0));
        g_nrHelper->SetGnbAntennaAttribute("BearingAngle", DoubleValue(M_PI)); // rad, pointing -x
        NetDeviceContainer gnb1 = g_nrHelper->InstallGnbDevice(NodeContainer(g_enbContainer.Get(1)), allBwps);
        g_gnbNrDevs.Add(gnb1.Get(0));
    }

    g_uavNrDevs = g_nrHelper->InstallUeDevice(g_uavContainer, allBwps);

    internet.Install(g_uavContainer);
    Ipv4InterfaceContainer ueIpIfaces =
        g_nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(g_uavNrDevs));
    if (topology == "hexgrid")
    {
        g_nrHelper->AttachToClosestGnb(g_uavNrDevs, g_gnbNrDevs);
    }
    else
    {
        g_nrHelper->AttachToGnb(g_uavNrDevs.Get(0), g_gnbNrDevs.Get(0));
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
            Ptr<NrGnbNetDevice> enbDev = g_gnbNrDevs.Get(i)->GetObject<NrGnbNetDevice>();
            Vector enbPos = enbDev->GetNode()->GetObject<MobilityModel>()->GetPosition();

            // Removed: LTE-specific antenna casting
        // Use NrHelper to set bearing angles
        auto gnbPhy = NrHelper::GetGnbPhy(
                g_gnbNrDevs.Get(i), 0);
        auto antenna = gnbPhy->GetSpectrumPhy()->GetAntenna();
            double orientation = 0.0; // bearing set via bearing angles

            double rad = orientation * M_PI / 180.0;
            Vector dir(std::cos(rad), std::sin(rad), 0.0);
            double ueHeight = (intUeHeightRng->GetValue() < aerialUeRatio)
                                  ? (50.0 + intUeHeightRng->GetValue() * 250.0)
                                  : 1.5;
            Vector uePos = enbPos + (100.0 * dir);
            uePos.z = ueHeight;
            g_interferingUeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(uePos);
        }

        g_interferingUeNrDevs = g_nrHelper->InstallUeDevice(g_interferingUeContainer, allBwps);
        internet.Install(g_interferingUeContainer);
        g_nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(g_interferingUeNrDevs));

        uint16_t intPort = 60000;
        for (uint32_t i = 0; i < numEnbs; ++i)
        {
            g_nrHelper->AttachToGnb(g_interferingUeNrDevs.Get(i), g_gnbNrDevs.Get(i));

            Ptr<Node> intUe = g_interferingUeContainer.Get(i);
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(intUe->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(g_nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

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

            // Interfering UE position retrieved from mobility model
            // (position already set above)
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
            // std::cout << "Static UE " << i << " position: (" << x << ", " << y << ", " <<
            // ueHeight
            //           << ")" << std::endl;
        }
        // std::cout << "Static UEs: " << numAerial << " aerial, " << (addStaticUes - numAerial)
        //           << " ground" << std::endl;

        g_staticUeNrDevs = g_nrHelper->InstallUeDevice(g_staticUeContainer, allBwps);
        internet.Install(g_staticUeContainer);
        Ipv4InterfaceContainer staticUeIfaces =
            g_nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(g_staticUeNrDevs));

        g_nrHelper->AttachToClosestGnb(g_staticUeNrDevs, g_gnbNrDevs);

        uint16_t staticPort = 50001;
        for (uint32_t i = 0; i < addStaticUes; ++i)
        {
            Ptr<Node> staticUe = g_staticUeContainer.Get(i);
            uint16_t port = staticPort + i;

            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(staticUe->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(g_nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

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
    // Removed: LTE measurement report config, not needed for NR
    // NR uses periodic RRC measurement reports (UE PHY ReportUeMeasurements)
                // (240ms cadence, matching MS240)
                                             // cadence)

    uint16_t dlPort = 50000;
    Ptr<Node> uav = g_uavContainer.Get(0);
    uint32_t uavNodeId = uav->GetId();

    // Determine sender and sink nodes based on TCP flow direction.
    // PHY-layer traces (RSRP, SINR, TBS, TxPower) always stay on the UAV.
    Ptr<Node> sourceNode;
    Ptr<Node> sinkNode;
    Address sinkAddress;
    if (flowDirection == "dl")
    {
        // DL: remoteHost sends data to UAV
        sourceNode = remoteHost;
        sinkNode = uav;
        sinkAddress = InetSocketAddress(ueIpIfaces.GetAddress(0), dlPort);
    }
    else
    {
        // UL (default): UAV sends data to remoteHost
        sourceNode = uav;
        sinkNode = remoteHost;
        sinkAddress = InetSocketAddress(interfaces.GetAddress(1), dlPort);
    }
    g_senderNodeId = sourceNode->GetId();
    g_receiverNodeId = sinkNode->GetId();

    // Ensure UAV has correct default route (needed in both directions for UL ACKs)
    {
        Ptr<Ipv4StaticRouting> uavStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(uav->GetObject<Ipv4>());
        uavStaticRouting->SetDefaultRoute(g_nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Install applications on the correct nodes per flow direction
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", sinkAddress);
    auto sinkApp = packetSinkHelper.Install(sinkNode);

    BulkSendHelper bulkSendHelper("ns3::TcpSocketFactory", sinkAddress);
    bulkSendHelper.SetAttribute("SendSize", UintegerValue(1024));
    auto sourceApp = bulkSendHelper.Install(sourceNode);
    sourceApp.Start(Seconds(1.0));
    sinkApp.Start(Seconds(1.0));

    std::string bulkSenderConfigPath =
        "/NodeList/" + std::to_string(g_senderNodeId) + "/ApplicationList/*/$ns3::BulkSendApplication/";
    std::string senderTcpBasePath =
        "/NodeList/" + std::to_string(g_senderNodeId) + "/$ns3::TcpL4Protocol/SocketList/*/";

    Config::ConnectWithoutContext(bulkSenderConfigPath + "ConnectionFailed",
                                  MakeCallback((&NotifyConnectionFailed)));
    Config::ConnectWithoutContext(bulkSenderConfigPath + "ConnectionSucceeded",
                                  MakeCallback((&NotifyConnectionSucceeded)));

    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/NrUeRrc/HandoverEndOk",
                                  MakeCallback(&HandoverOk));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/NrUeRrc/HandoverEndError",
                                  MakeCallback(&HandoverError));

    Simulator::Schedule(Seconds(1.1), []() {
        Ptr<Node> uav = g_uavContainer.Get(0);
        auto ueDev = uav->GetDevice(0)->GetObject<NrUeNetDevice>();
        auto rrc = ueDev->GetRrc();
        auto ipv4 = uav->GetObject<Ipv4>();
        std::cout << "DEBUG t=" << Simulator::Now().GetSeconds() << " rrcState=" << rrc->GetState()
                  << " cellId=" << rrc->GetCellId() << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal()
                  << std::endl;
    });
    // Schedule metric-accumulating traces (always connected, regardless of logging)
    Simulator::Schedule(Seconds(1.1), [senderTcpBasePath]() {
        Config::ConnectWithoutContext(senderTcpBasePath + "Retransmission",
                                      MakeCallback(&SourceRetransmissionPacket));

        std::string rxPath = "/NodeList/" + std::to_string(g_receiverNodeId) +
                             "/ApplicationList/*/$ns3::PacketSink/Rx";
        Config::ConnectWithoutContext(rxPath, MakeCallback(&SinkRxPacket));

        Config::ConnectWithoutContext(senderTcpBasePath + "RTT",
                                      MakeCallback(&TcpRttChange));

        Config::ConnectWithoutContext(senderTcpBasePath + "State",
                                      MakeCallback(&NotifyTcpStateChange));
    });

    if (logging)
    {
        Simulator::Schedule(Seconds(1.1), [bulkSenderConfigPath, senderTcpBasePath]() {
            Config::ConnectWithoutContext(senderTcpBasePath + "CongestionWindow",
                                          MakeCallback(&CwndTracer));

            Config::ConnectWithoutContext(bulkSenderConfigPath + "Tx",
                                          MakeCallback(&SourceTxPacket));
        });

        Config::Connect("/NodeList/" + std::to_string(uavNodeId) +
                            "/$ns3::MobilityModel/CourseChange",
                        MakeCallback(&MobilityCourseChange));

        // Periodic position logging (every 200ms) — fills gaps between CourseChange events
        Simulator::Schedule(Seconds(0.0), &UavPeriodicPositionLog);

        if (tcpVariant == "TcpBbr")
        {
            Simulator::Schedule(Seconds(1.1), [senderTcpBasePath]() {
                Config::ConnectWithoutContext(senderTcpBasePath + "CongestionOps/$ns3::TcpBbr/PacingGain",
                    MakeCallback(&BbrPacingGainChange));

                Config::ConnectWithoutContext(senderTcpBasePath + "CongestionOps/$ns3::TcpBbr/CwndGain",
                    MakeCallback(&BbrCwndGainChange));

                Config::ConnectWithoutContext(senderTcpBasePath + "RateOps/TcpRateSampleUpdated",
                    MakeCallback(&TcpRateSampleChange));
            });
        }

        // PHY-layer traces (always on UAV, independent of flow direction)
        Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                          "/DeviceList/*/$ns3::NrUeNetDevice/"
                                          "ComponentCarrierMapUe/*/NrUePhy/"
                                          "ReportUeMeasurements",
                                      MakeCallback(&LogUeMeasReport));

        // Use DL or UL TBS trace depending on flow direction
        std::string tbSizeTrace = (flowDirection == "dl")
            ? "ReportDownlinkTbSize"
            : "ReportUplinkTbSize";
        Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                          "/DeviceList/*/$ns3::NrUeNetDevice/"
                                          "ComponentCarrierMapUe/*/NrUePhy/" +
                                          tbSizeTrace,
                                      MakeCallback(&ReportUlTbs));

        // Use DL or UL SINR trace depending on flow direction
        if (flowDirection == "dl")
        {
            // DL data SINR measured at UE PHY (per-slot, linear scale -> dB)
            Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                              "/DeviceList/*/"
                                              "$ns3::NrUeNetDevice/"
                                              "ComponentCarrierMapUe/*/NrUePhy/"
                                              "DlDataSinr",
                                          MakeCallback(&ReportDlSinr));
        }
        else
        {
            // UL SINR measured at gNB PHY (based on SRS)
            Config::ConnectWithoutContext(
                "/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/UlSinrTrace",
                MakeCallback(&ReportUlSinr));
        }

        Simulator::Schedule(Seconds(1.1), [uavNodeId]() {
            Ptr<Node> uavNode = NodeList::GetNode(uavNodeId);
            Ptr<NrUeNetDevice> ueNetDev = uavNode->GetDevice(0)->GetObject<NrUeNetDevice>();
            if (ueNetDev)
            {
                Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueNetDev, 0);
                Ptr<NrUePowerControl> powerCtrl = uePhy->GetUplinkPowerControl();
                powerCtrl->TraceConnectWithoutContext("ReportPuschTxPower",
                                                      MakeCallback(&ReportUeTxPower));
            }
        });

        Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrNetDevice/$ns3::NrUeNetDevice/NrUeRrc"
                        "/StateTransition",
                        MakeCallback(&UavRrcStateChange));
    }

    // Add X2 Interface (already added inside hexgrid block)
    if (topology == "simple")
    {
        g_nrHelper->AddX2Interface(g_enbContainer);
    }

    // ---- RL Framework: Install handover RL apps on UAV node ---- //
    if (rlMode)
    {
        uint32_t numBs = g_enbContainer.GetN();
        uint32_t uavNodeId = g_uavContainer.Get(0)->GetId();
        g_lastRsrpValues.resize(numBs + 1, -140.0); // index by cellId (1-based), dBm
        g_lastSinrValues.resize(numBs + 1, -40.0);  // index by cellId (1-based), -40dB = unknown
        g_lastRsrqValues.resize(numBs + 1, -20.0);  // index by cellId (1-based), dB

        // std::cout << "Installing RL handover apps (NumBs=" << numBs << ", StepTime=" << stepTime
        //           << "ms"
        //           << ", delay=" << delay << "ms"
        //           << ", handoverPenalty=" << handoverPenalty << ", uavNodeId=" << uavNodeId <<
        //           ")"
        //           << std::endl;

        uint32_t remoteHostNodeId = g_remoteHostContainer.Get(0)->GetId();

        RlApplicationHelper rlAppHelper(NrRlHandoverRewardApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
        rlAppHelper.SetAttribute("StopTime", TimeValue(Seconds(simDuration)));
        rlAppHelper.SetAttribute("RemoteHostNodeId", UintegerValue(remoteHostNodeId));
        rlAppHelper.SetAttribute("HandoverPenalty", DoubleValue(handoverPenalty));
        rlAppHelper.SetAttribute("ReferenceRate", DoubleValue(referenceRateBps));
        rlAppHelper.SetAttribute("MinimumAcceptableGoodput", DoubleValue(minAcceptableGoodputBps));
        rlAppHelper.SetAttribute("DelayMinRttMs", DoubleValue(delayMinRttMs));
        rlAppHelper.SetAttribute("MaxAcceptableRttMs", DoubleValue(minAcceptableRttMs));
        rlAppHelper.SetAttribute("TcpFailurePenalty", DoubleValue(tcpFailurePenalty));
        rlAppHelper.SetAttribute("RlfPenalty", DoubleValue(rlfPenalty));
        rlAppHelper.SetAttribute(
            "CalculationInterval",
            TimeValue(MilliSeconds(stepTime))); // Match UE PHY measurement period
        auto rewardApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverAgentApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        auto agentApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverObservationApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("UavNodeId", UintegerValue(uavNodeId));
        rlAppHelper.SetAttribute("StepTimeMs", UintegerValue(stepTime));
        rlAppHelper.SetAttribute("HandoverMargin", DoubleValue(handoverMargin));
        auto obsApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverActionApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
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
