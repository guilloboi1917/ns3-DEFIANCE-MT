/**
 * Setup file for the HARL NR Scenario
 * Supports multiple topologies:
 *   "simple"  — 2 gNodeBs on a line, UAV shuttles between them
 *   "triangle" — 3 sites, 3 sectors each
 *   "hexgrid" — hexagonal grid of 3-sector macro sites, UAV follows ascend-random or
 * random-waypoint 1 aerial UE, 1 remote server. The UE runs a TCP/UDP OnOffApplication to the
 * remote server (BulkSend was removed — it floods the NR MAC at line rate).
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
#include "ns3/point-to-point-module.h"
#include "ns3/rl-application-helper.h"
#include "ns3/spectrum-value.h"
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
    const char* oldName = TcpSocket::TcpStateName[oldState];
    const char* newName = TcpSocket::TcpStateName[newState];

    std::cout << "TCP state: " << oldName << " -> " << newName
              << " at t=" << Simulator::Now().GetSeconds() << "s" << std::endl;

    if (newState == TcpSocket::ESTABLISHED)
    {
        g_tcpConnected = true;
        g_tcpAlive = true;
    }

    if (newState == TcpSocket::CLOSED || newState == TcpSocket::LAST_ACK)
    {
        g_tcpAlive = false;
    }
}

// TCP congestion state logger — logs CA_OPEN, CA_DISORDER, CA_RECOVERY, CA_LOSS, CA_CWR
void
CongestionStateLogger(TcpSocketState::TcpCongState_t oldState,
                      TcpSocketState::TcpCongState_t newState)
{
    const char* oldName = TcpSocketState::TcpCongStateName[oldState];
    const char* newName = TcpSocketState::TcpCongStateName[newState];

    std::cout << "TCP congestion: " << oldName << " -> " << newName
              << " at t=" << Simulator::Now().GetSeconds() << "s" << std::endl;

    std::ofstream congFile(g_outputDir + "nr-rl-congestion.csv", std::ios_base::app);
    congFile << Simulator::Now().GetSeconds() << "," << oldName << "," << newName << std::endl;
}

// HARQ feedback logger — fires when UE sends ACK/NACK on PUCCH/PUSCH
void
HarqFeedbackLogger(SfnSf sfn,
                   uint16_t cellId,
                   uint16_t rnti,
                   uint8_t bwpId,
                   uint8_t harqId,
                   uint32_t k1Delay)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream harqFile(g_outputDir + "nr-rl-harq.csv", std::ios_base::app);
    harqFile << Simulator::Now().GetSeconds() << "," << cellId << "," << (uint32_t)rnti << ","
             << (uint32_t)bwpId << "," << (uint32_t)harqId << "," << k1Delay << std::endl;
}

// SlotDataStats logger — per-slot RB utilization (serving cell only)
void
SlotDataStatsLogger(const SfnSf& sfn,
                    uint32_t scheduledUe,
                    uint32_t usedReg,
                    uint32_t usedSym,
                    uint32_t availableRb,
                    uint32_t availableSym,
                    uint16_t bwpId,
                    uint16_t cellId)
{
    if (!g_logging)
    {
        return;
    }
    double utilPct = (availableRb > 0) ? (100.0 * usedReg / (availableRb * availableSym)) : 0.0;

    // Only log to serving-cell file when this gNB is the UAV's current serving cell
    if (cellId != static_cast<uint16_t>(g_currentCellId))
    {
        return;
    }
    std::ofstream slotFile(g_outputDir + "nr-rl-slot-stats.csv", std::ios_base::app);
    slotFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << scheduledUe
             << "," << usedReg << "," << usedSym << "," << availableRb << "," << availableSym << ","
             << (int)utilPct << std::endl;
}

// CWND tracing callback — fires on every CWND change (every ACK)
void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream cwndFile(g_outputDir + "nr-rl-cwnd.csv", std::ios_base::app);
    cwndFile << Simulator::Now().GetSeconds() << "," << newCwnd << std::endl;
}

// SrReq logger — fires when a UE sends a Scheduling Request
void
SrReqLogger(uint16_t cellId, uint8_t bwpId, uint16_t rnti)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream srFile(g_outputDir + "nr-rl-sr.csv", std::ios_base::app);
    srFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << (uint32_t)bwpId
           << "," << (uint32_t)rnti << std::endl;
}

// DlScheduling logger — fires for every DL scheduling decision
void
DlSchedulingLogger(uint16_t cellId, NrSchedulingCallbackInfo traceInfo)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream dlSchedFile(g_outputDir + "nr-rl-dl-sched.csv", std::ios_base::app);
    dlSchedFile << Simulator::Now().GetSeconds() << "," << cellId << ","
                << (uint32_t)traceInfo.m_rnti << "," << (uint32_t)traceInfo.m_mcs << ","
                << traceInfo.m_tbSize << "," << (uint32_t)traceInfo.m_symStart << ","
                << (uint32_t)traceInfo.m_numSym << std::endl;
}

// UlScheduling logger — fires for every UL scheduling decision
void
UlSchedulingLogger(uint16_t cellId, NrSchedulingCallbackInfo traceInfo)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream ulSchedFile(g_outputDir + "nr-rl-ul-sched.csv", std::ios_base::app);
    ulSchedFile << Simulator::Now().GetSeconds() << "," << cellId << ","
                << (uint32_t)traceInfo.m_rnti << "," << (uint32_t)traceInfo.m_mcs << ","
                << traceInfo.m_tbSize << "," << (uint32_t)traceInfo.m_symStart << ","
                << (uint32_t)traceInfo.m_numSym << std::endl;
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
    std::cout << "RRC state change for UE " << imsi << ", RNTI " << rnti << " to cell " << cellId
              << " (state " << oldState << " -> " << newState << ") at time "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
    g_currentRnti = rnti;
    g_currentCellId = cellId;

    // Detect RLF: drops from CONNECTED_NORMALLY to a non-handover state while TCP was connected
    if (oldState == NrUeRrc::CONNECTED_NORMALLY && newState != NrUeRrc::CONNECTED_HANDOVER &&
        g_tcpConnected)
    {
        g_rlfTriggered = true;
        g_tcpAlive = false;
        g_rlfCount++;
        if (g_logging)
        {
            std::ofstream rlfFile(g_outputDir + "nr-rl-rlf.csv", std::ios_base::app);
            rlfFile << Simulator::Now().GetSeconds() << "," << cellId << std::endl;
        }
        std::cout << "RLF detected at time " << Simulator::Now().GetSeconds() << "s" << std::endl;
    }
}

void
HandoverOk(const uint64_t imsi, const uint16_t cellId, const uint16_t rnti)
{
    g_handoverInProgress = false;
    g_currentCellId = cellId;
    g_currentRnti = rnti;
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
    if (!g_logging)
    {
        return;
    }
    std::ofstream ueMeasFile(g_outputDir + "ue_meas_report.csv", std::ios_base::app);
    ueMeasFile << Simulator::Now().GetSeconds() << "," << (int)cellId << "," << (int)rnti << ","
               << rsrp << "," << rsrq << "," << (int)isServingCell << std::endl;
}

// DL data SINR (UE PHY DlDataSinr) logged to dl_sinr.csv (always present)
void
DlDataSinrLogger(uint16_t cellId, uint16_t rnti, double sinrLinear, uint16_t bwpId)
{
    if (!g_logging)
    {
        return;
    }
    double sinrDb = (sinrLinear > 0.0) ? (10.0 * std::log10(sinrLinear)) : -40.0;
    std::ofstream dlSinrFile(g_outputDir + "dl_sinr.csv", std::ios_base::app);
    dlSinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << sinrDb
               << std::endl;
}

// UL HARQ feedback logger (gNB PHY)
void
UlHarqFeedbackLogger(uint16_t rnti, bool isReceivedOk)
{
    if (g_logging)
    {
        std::ofstream harqFile(g_outputDir + "nr-rl-ul-harq.csv", std::ios_base::app);
        harqFile << Simulator::Now().GetSeconds() << "," << rnti << "," << (int)isReceivedOk
                 << std::endl;
    }
}

// UL RX packet logger (gNB spectrum PHY, post-beamforming MIMO SINR)
void
UlRxPacketLogger(uint16_t cellId, RxPacketTraceParams params)
{
    if (g_logging)
    {
        std::ofstream sinrFile(g_outputDir + "nr-rl-ul-rx-sinr.csv", std::ios_base::app);
        double sinrDb = (params.m_sinr > 0.0) ? (10.0 * std::log10(params.m_sinr)) : -40.0;
        sinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << params.m_rnti << ","
                 << sinrDb << "," << (int)params.m_mcs << "," << params.m_tbSize << ","
                 << (int)params.m_corrupt << "," << params.m_tbler << std::endl;
    }
}

// UL SINR logged to ul_sinr_srs.csv
//
// Fires from GenerateDataCqiReport in NrGnbPhy, which is connected to the
// pData chunk processor (UL DATA SINR).  The SINR is time-averaged by the
// NrChunkProcessor.  This is the same SINR that the scheduler uses for UL
// CQI / MCS selection.
//
// NOTE: When gNBs share the exact same position (e.g., co-located sectors
// without antenna offset), the channel model produces NaN Doppler values,
// which causes all SINR entries to be NaN and get clamped to -40 dB.
// Always apply a small (>= 1 m) antenna offset between co-located gNBs.
void
UlSrsSinrLogger(uint16_t cellId,
                uint64_t imsi,
                SpectrumValue& sinrSpectrum,
                SpectrumValue& /* interferenceSpectrum */)
{
    if (!g_logging)
    {
        return;
    }
    if (cellId != g_currentCellId)
    {
        return;
    }
    double sumSinr = 0.0;
    uint32_t numRb = 0;
    for (auto it = sinrSpectrum.ConstValuesBegin(); it != sinrSpectrum.ConstValuesEnd(); ++it)
    {
        double sinrDb = (*it > 1e-12) ? (10.0 * std::log10(*it)) : -40.0;
        sumSinr += sinrDb;
        numRb++;
    }
    if (numRb == 0)
    {
        return;
    }
    double sinrDb = sumSinr / static_cast<double>(numRb);
    std::ofstream ulSrsFile(g_outputDir + "ul_sinr_srs.csv", std::ios_base::app);
    ulSrsFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << sinrDb
              << std::endl;
}

// UE TX power logger
void
ReportUeTxPower(uint16_t cellId, uint16_t rnti, double powerDbm)
{
    if (!g_logging)
    {
        return;
    }
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
            mobilityFile << Simulator::Now().GetSeconds() << "," << mob->GetPosition() << "," << 0
                         << std::endl;
        }
    }
    // Re-schedule every 500ms
    Simulator::Schedule(MilliSeconds(500), &UavPeriodicPositionLog);
}

void
SimHeartbeatLog()
{
    static int beatCount = 0;
    beatCount++;
    std::cout << "[HEARTBEAT #" << beatCount << "] t=" << Simulator::Now().GetSeconds()
              << "s cellId=" << g_currentCellId << " hoCount=" << g_totalHandovers
              << " nEvents=" << Simulator::GetEventCount()
              << " hoInProgress=" << g_handoverInProgress << std::endl;
    Simulator::Schedule(Seconds(1.0), &SimHeartbeatLog);
}

void
WatchdogLog()
{
    static int wdCount = 0;
    wdCount++;
    if (wdCount <= 30 || wdCount % 10 == 0) // first 30 every 100ms, then every 1s
    {
        std::cout << "[WATCHDOG " << wdCount << "] t=" << Simulator::Now().GetSeconds()
                  << "s nEvents=" << Simulator::GetEventCount() << std::endl;
    }
    if (wdCount < 100) // stop after 10s sim time
    {
        Simulator::Schedule(MilliSeconds(100), &WatchdogLog);
    }
}

void
MobilityCourseChange(std::string context, Ptr<const MobilityModel> model)
{
    std::ofstream mobilityFile(g_outputDir + "mobility.csv", std::ios_base::app);
    mobilityFile << Simulator::Now().GetSeconds() << "," << model->GetPosition() << "," << 1
                 << std::endl;
}

/**
 * Dump gNB antenna positions and orientations to gnb-antennas.csv.
 *
 * Position comes from the gNB node mobility model (includes the per-sector
 * 1 m antenna offset); orientation is read back from the antenna model:
 * bearing angle (GetAlpha, radians in [-pi, pi], 0 = +x axis, counter-
 * clockwise) and downtilt (GetBeta, radians), both converted to degrees.
 * Written once at setup; consumed by plot-uav-path.py to draw translucent
 * sector cones over the UAV path.
 */
void
LogGnbAntennas()
{
    std::ofstream antFile(g_outputDir + "gnb-antennas.csv");
    antFile << "cellId,x,y,z,bearingDeg,downtiltDeg" << std::endl;
    for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(g_gnbNrDevs.Get(i), 0);
        Ptr<UniformPlanarArray> antenna =
            DynamicCast<UniformPlanarArray>(phy->GetSpectrumPhy()->GetAntenna());
        if (!antenna)
        {
            continue;
        }
        Vector pos = g_gnbContainer.Get(i)->GetObject<MobilityModel>()->GetPosition();
        uint32_t cellId = g_gnbNrDevs.Get(i)->GetObject<NrGnbNetDevice>()->GetCellId();
        antFile << cellId << "," << pos.x << "," << pos.y << "," << pos.z << ","
                << antenna->GetAlpha() * 180.0 / M_PI << ","
                << antenna->GetBeta() * 180.0 / M_PI << std::endl;
    }
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
SourceTxPacket(Ptr<const Packet> packet, const Address& local, const Address& remote)
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
// Helper: bounding box over gNB positions (used by hexgrid topology)
// ------------------------------------------------------------------------- //
struct BoundingBox
{
    double minX;
    double maxX;
    double minY;
    double maxY;
};

BoundingBox
ComputeGnbBoundingBox(NodeContainer gnbNodes, double padding)
{
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Vector pos = gnbNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
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
              std::string transportProtocol = "tcp",
              double ueSpeed = 20.0,            // m/s
              double simDuration = 80.0,        // seconds
              double intersiteDistance = 500.0, // m
              uint32_t numMacroCells = 7,       // number of macro sites
              double gnbDowntilt = 10.0,        // degrees
              uint32_t seed = 0,                // Seed for RNG
              uint32_t runId = 0,
              std::string trialName = "1",
              std::string tcpVariant = "TcpBbr",
              std::string uavMobility = "random-waypoint",
              std::string topology = "triangle",
              double startHeight = 80.0,
              double endHeight = 300.0,
              uint32_t bbrWindowLength = 10,
              uint32_t addInterferingUes = 0,
              double aerialUeRatio = 0.0,
              bool logging = true,
              bool rlMode = false,
              std::string handoverAlgorithm = "a3",
              uint32_t stepTime = 200,
              uint32_t delay = 0,
              double handoverPenalty = 0.1,
              double handoverMargin = 3.0,
              double rlAlphaGoodput = 0.8,
              double rlBetaGoodput = 5.0,
              double rlBetaHandover = 60.0,
              const std::string& rlRewardComposition = "additive",
              bool useTbsObservation = true,
              double rlPingPongMultiplier = 5.0,
              bool rlHandoverRatePenalty = false,
              uint32_t rlHandoverRateWindowMs = 10000,
              uint32_t rlHandoverRateBudget = 2,
              double rlHandoverRateLambda = 0.2,
              uint32_t rlTopN = 3,
              uint32_t rlHandoverDebounceMs = 0,
              double rlRewardRefMbps = 40.0,
              std::string rlRewardGoodputShape = "deng",
              double rlRewardGoodputAlpha = 3.0,
              double rlRewardGoodputP = 0.4,
              std::string outputDir = "")
{
    if (outputDir.empty())
    {
        g_outputDir = pathToNs3 + "/contrib/defiance/examples/nr-rl-handover/output/";
    }
    else
    {
        g_outputDir = outputDir;
        if (g_outputDir.back() != '/')
        {
            g_outputDir += '/';
        }
    }

    // Travel leg constraints for UAV waypoint generation
    const double travelLegMin = 80.0;
    const double travelLegMax = 200.0;

    // Seed RNG before any random operations
    RngSeedManager::SetSeed(seed == 0U ? time(nullptr) : seed);
    RngSeedManager::SetRun(runId == 0U ? 1 : runId);

    // Clear data files
    if (logging)
    {
        std::cout << "Logging to: " << g_outputDir << std::endl;
        std::filesystem::create_directories(g_outputDir);
        for (auto f : {"nr-rl-cwnd.csv",      "nr-rl-handovers.csv",   "nr-rl-rate.csv",
                       "nr-rl-cwnd-gain.csv", "nr-rl-pacing-gain.csv", "nr-rl-rtt.csv",
                       "dl_sinr.csv",         "ul_sinr_srs.csv",       "rsrp_sinr.csv",
                       "ue_meas_report.csv",  "mobility.csv",          "sink-packets.csv",
                       "source-packets.csv",  "retransmissions.csv",   "ue_tx_power.csv",
                       "rl_obs.csv",          "rl_reward.csv",         "rl_action.csv",
                       "rl_actions_full.csv", "nr-rl-harq.csv",        "nr-rl-congestion.csv",  "nr-rl-slot-stats.csv",
                       "nr-rl-rlf.csv",       "nr-rl-sr.csv",          "nr-rl-dl-sched.csv",
                       "nr-rl-ul-sched.csv",  "nr-rl-ul-harq.csv",     "nr-rl-ul-rx-sinr.csv",
                       "gnb-antennas.csv"})
        {
            std::ofstream fout(g_outputDir + f);
            // truncate on open
        }

        // Write meta.yaml with input parameters for reproducibility
        std::ofstream metaOut(g_outputDir + "meta.yaml");
        metaOut << "# Simulation metadata — auto-generated by scenarioSetup" << std::endl;
        metaOut << "flowDirection: " << flowDirection << std::endl;
        metaOut << "transportProtocol: " << transportProtocol << std::endl;
        metaOut << "ueSpeed: " << ueSpeed << std::endl;
        metaOut << "simDuration: " << simDuration << std::endl;
        metaOut << "intersiteDistance: " << intersiteDistance << std::endl;
        metaOut << "numMacroCells: " << numMacroCells << std::endl;
        metaOut << "gnbDowntilt: " << gnbDowntilt << std::endl;
        metaOut << "seed: " << seed << std::endl;
        metaOut << "runId: " << runId << std::endl;
        metaOut << "trialName: " << trialName << std::endl;
        metaOut << "tcpVariant: " << tcpVariant << std::endl;
        metaOut << "uavMobility: " << uavMobility << std::endl;
        metaOut << "topology: " << topology << std::endl;
        metaOut << "startHeight: " << startHeight << std::endl;
        metaOut << "endHeight: " << endHeight << std::endl;
        metaOut << "bbrWindowLength: " << bbrWindowLength << std::endl;
        metaOut << "addInterferingUes: " << addInterferingUes << std::endl;
        metaOut << "aerialUeRatio: " << aerialUeRatio << std::endl;
        metaOut << "rlMode: " << (rlMode ? "true" : "false") << std::endl;
        metaOut << "handoverAlgorithm: " << handoverAlgorithm << std::endl;
        metaOut << "stepTime: " << stepTime << std::endl;
        metaOut << "delay: " << delay << std::endl;
        metaOut << "handoverMargin: " << handoverMargin << std::endl;
        metaOut << "handoverPenalty: " << handoverPenalty << std::endl;
        metaOut << "rlAlphaGoodput: " << rlAlphaGoodput << std::endl;
        metaOut << "rlBetaGoodput: " << rlBetaGoodput << std::endl;
        metaOut << "rlBetaHandover: " << rlBetaHandover << std::endl;
        metaOut << "rlRewardComposition: " << rlRewardComposition << std::endl;
        metaOut << "useTbsObservation: " << (useTbsObservation ? "true" : "false") << std::endl;
        metaOut << "rlPingPongMultiplier: " << rlPingPongMultiplier << std::endl;
        metaOut << "rlHandoverRatePenalty: " << (rlHandoverRatePenalty ? "true" : "false") << std::endl;
        metaOut << "rlHandoverRateWindowMs: " << rlHandoverRateWindowMs << std::endl;
        metaOut << "rlHandoverRateBudget: " << rlHandoverRateBudget << std::endl;
        metaOut << "rlHandoverRateLambda: " << rlHandoverRateLambda << std::endl;
        metaOut << "rlTopN: " << rlTopN << std::endl;
        metaOut << "rlHandoverDebounceMs: " << rlHandoverDebounceMs << std::endl;
        metaOut << "rlRewardRefMbps: " << rlRewardRefMbps << std::endl;
        metaOut << "rlRewardGoodputShape: " << rlRewardGoodputShape << std::endl;
        metaOut << "rlRewardGoodputAlpha: " << rlRewardGoodputAlpha << std::endl;
        metaOut << "rlRewardGoodputP: " << rlRewardGoodputP << std::endl;
        metaOut << "outputDir: " << g_outputDir << std::endl;
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
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    g_nrChannelHelper->ConfigureFactories("UMa-AV", "Default", "ThreeGpp");
    g_nrChannelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                         TimeValue(MilliSeconds(100)));
    g_nrChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    g_nrChannelHelper->AssignChannelsToBands({band});

    // --- Scheduler, error model, beamforming --- //
    g_nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaPF"));
    g_nrHelper->SetDlErrorModel("ns3::NrEesmIrT2");
    g_nrHelper->SetUlErrorModel("ns3::NrEesmIrT2");
    idealBeamformingHelper->SetAttribute(
        "BeamformingMethod",
        TypeIdValue(TypeId::LookupByName("ns3::DirectPathQuasiOmniBeamforming")));

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

    // --- gNB antenna: UniformPlanarArray with 4x2 isotropic elements --- //
    // All NR examples use IsotropicAntennaModel as the antenna element (array provides
    // beamforming).
    NrHelper::AntennaParams apGnb;
    apGnb.antennaElem = "ns3::ThreeGppAntennaModel";
    apGnb.nAntCols = 4;
    apGnb.nAntRows = 2;
    apGnb.nHorizPorts = 1;
    apGnb.nVertPorts = 1;
    apGnb.isDualPolarized = false;
    apGnb.bearingAngle = 0.0;
    apGnb.polSlantAngle = 0.0;
    g_nrHelper->SetupGnbAntennas(apGnb);
    g_nrHelper->SetGnbAntennaAttribute("DowntiltAngle", DoubleValue(gnbDowntilt * M_PI / 180.0));

    // --- UE antenna --- //
    NrHelper::AntennaParams apUe;
    apUe.antennaElem = "ns3::IsotropicAntennaModel";
    apUe.nAntCols = 1;
    apUe.nAntRows = 1;
    apUe.nHorizPorts = 1;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = false;
    apUe.bearingAngle = 0.0;
    apUe.polSlantAngle = 0.0;
    g_nrHelper->SetupUeAntennas(apUe);

    // --- UL Power Control (open-loop, fractional path loss compensation) --- //
    Config::SetDefault("ns3::NrUePowerControl::AccumulationEnabled", BooleanValue(false));
    Config::SetDefault("ns3::NrUePowerControl::ClosedLoop", BooleanValue(true));
    Config::SetDefault("ns3::NrUePowerControl::Alpha", DoubleValue(0.7));
    Config::SetDefault("ns3::NrUePowerControl::PoNominalPusch", IntegerValue(-80));
    Config::SetDefault("ns3::NrUePowerControl::PoUePusch", IntegerValue(0));
    // Use fixed TDD pattern instead of all-Flexible (default). The all-Flexible
    // pattern causes "Cannot TX while RX" crashes when the scheduler independently
    // assigns UL grants and DL data to the same UE in Flexible slots.
    Config::SetDefault("ns3::NrGnbPhy::Pattern", StringValue("DL|S|UL|UL|DL|DL|S|UL|UL|DL|"));

    // --- PHY configuration --- //
    g_nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    g_nrHelper->SetGnbPhyAttribute("TxPower",
                                   DoubleValue(23.0)); // This was too high before at 46? db
    g_nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
    g_nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(9.0));

    // --- RLC UM buffer size (matches legacy LTE setting from original prototype) --- //
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(180000));

    // Reduce TCP MinRTO from RFC 6298 default (1s) to Linux standard (200ms)
    // to recover faster from handover-induced packet loss.
    Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue(MilliSeconds(200)));

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

    // ---- gNB setup: simple vs hexgrid ---------------------------------- //
    if (topology == "hexgrid")
    {
        // Manual hexgrid: 1 ring = 7 sites, 3 sectors each = 21 gNBs
        // Site positions follow a hexagonal layout at intersiteDistance spacing.
        uint32_t nMacroGnbSitesX = numMacroCells == 7 ? 2 : 1;
        g_gnbContainer.Create(3 * numMacroCells);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_gnbContainer);

        // Site positions (hexagonal grid)
        // Site 0 at origin, sites 1-6 at 60-degree increments
        std::vector<Vector> sitePositions;
        sitePositions.push_back(Vector(0.0, 0.0, 25.0));
        for (uint32_t ring = 1; ring <= nMacroGnbSitesX; ++ring)
        {
            double radius = intersiteDistance * ring;
            for (uint32_t s = 0; s < 6; ++s)
            {
                double angle = s * M_PI / 3.0 + (ring % 2) * M_PI / 6.0;
                double x = radius * std::cos(angle);
                double y = radius * std::sin(angle);
                sitePositions.push_back(Vector(x, y, 25.0));
                if (sitePositions.size() >= numMacroCells)
                {
                    break;
                }
            }
            if (sitePositions.size() >= numMacroCells)
            {
                break;
            }
        }

        // Position gNBs with 1m antenna offset from site center per sector
        // (matching HexagonalGridScenarioHelper::GetAntennaPosition).
        // Without the offset, gNBs at the same site share identical positions,
        // which violates the constraint that gNBs cannot have the same location.
        uint32_t gNbIdx = 0;
        for (uint32_t site = 0; site < sitePositions.size() && gNbIdx < g_gnbContainer.GetN();
             ++site)
        {
            for (uint32_t sector = 0; sector < 3 && gNbIdx < g_gnbContainer.GetN();
                 ++sector, ++gNbIdx)
            {
                Vector pos = sitePositions[site];
                // Antenna offset: 1m in the direction of the sector bore sight
                // Sector 0: 30°  (cos=0.866, sin=0.5)
                // Sector 1: 150° (cos=-0.866, sin=0.5)
                // Sector 2: 270° (cos=0, sin=-1)
                if (sector == 0)
                {
                    pos.x += 0.866025;
                    pos.y += 0.5;
                }
                else if (sector == 1)
                {
                    pos.x += -0.866025;
                    pos.y += 0.5;
                }
                else
                {
                    pos.x += 0.0;
                    pos.y += -1.0;
                }
                g_gnbContainer.Get(gNbIdx)->GetObject<MobilityModel>()->SetPosition(pos);
            }
        }
        g_gnbNrDevs = g_nrHelper->InstallGnbDevice(g_gnbContainer, allBwps);

        // Set per-sector bearing angles (NR module convention: 30°, 150°, 270°)
        for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); ++i)
        {
            uint32_t sector = i % 3;
            double bearingRads;
            if (sector == 0)
            {
                bearingRads = M_PI / 6.0; // 30°
            }
            else if (sector == 1)
            {
                bearingRads = 5.0 * M_PI / 6.0; // 150°
            }
            else
            {
                bearingRads = -M_PI / 2.0; // 270° mapped to [-\u03c0, \u03c0] = -90°
            }
            Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(g_gnbNrDevs.Get(i), 0);
            Ptr<UniformPlanarArray> antenna =
                DynamicCast<UniformPlanarArray>(phy->GetSpectrumPhy()->GetAntenna());
            antenna->SetAttribute("BearingAngle", DoubleValue(bearingRads));
        }

        // Add X2 interface for handover support
        g_nrHelper->AddX2Interface(g_gnbContainer);

        // Update config after device installation
        // (NrGnbNetDevice::UpdateConfig is called inside InstallGnbDevice)

        // ---- UAV mobility (hexgrid) ------------------------------------ //
        BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.2);
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
    else if (topology == "triangle")
    {
        // 3 sites forming an equilateral triangle, 3 sectors each = 9 gNBs
        uint32_t numSites = 3;
        uint32_t sectorsPerSite = 3;
        g_gnbContainer.Create(numSites * sectorsPerSite);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_gnbContainer);

        // Triangle sites (equilateral, side = intersiteDistance):
        // Site 0: (0, 0)
        // Site 1: (isd, 0)
        // Site 2: (isd/2, isd * sqrt(3)/2)
        double isd = intersiteDistance;
        Vector sitePositions[3] = {Vector(0.0, 0.0, 25.0),
                                   Vector(isd, 0.0, 25.0),
                                   Vector(isd * 0.5, isd * std::sqrt(3.0) / 2.0, 25.0)};

        // Position gNBs with 1m antenna offset from site center per sector
        // (matching HexagonalGridScenarioHelper::GetAntennaPosition).
        uint32_t gNbIdx = 0;
        for (uint32_t site = 0; site < numSites; ++site)
        {
            for (uint32_t sector = 0; sector < sectorsPerSite; ++sector, ++gNbIdx)
            {
                Vector pos = sitePositions[site];
                // Antenna offset: 1m in the direction of the sector bore sight
                // Sector 0: 30°  (cos=0.866, sin=0.5)
                // Sector 1: 150° (cos=-0.866, sin=0.5)
                // Sector 2: 270° (cos=0, sin=-1)
                if (sector == 0)
                {
                    pos.x += 0.866025;
                    pos.y += 0.5;
                }
                else if (sector == 1)
                {
                    pos.x += -0.866025;
                    pos.y += 0.5;
                }
                else
                {
                    pos.x += 0.0;
                    pos.y += -1.0;
                }
                g_gnbContainer.Get(gNbIdx)->GetObject<MobilityModel>()->SetPosition(pos);
            }
        }

        g_gnbNrDevs = g_nrHelper->InstallGnbDevice(g_gnbContainer, allBwps);

        // Set per-sector bearing angles (NR module convention: 30°, 150°, 270°)
        for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); ++i)
        {
            uint32_t sector = i % 3;
            double bearingRads;
            if (sector == 0)
            {
                bearingRads = M_PI / 6.0; // 30°
            }
            else if (sector == 1)
            {
                bearingRads = 5.0 * M_PI / 6.0; // 150°
            }
            else
            {
                bearingRads = -M_PI / 2.0; // 270° mapped to [-\u03c0, \u03c0] = -90°
            }
            Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(g_gnbNrDevs.Get(i), 0);
            Ptr<UniformPlanarArray> antenna =
                DynamicCast<UniformPlanarArray>(phy->GetSpectrumPhy()->GetAntenna());
            antenna->SetAttribute("BearingAngle", DoubleValue(bearingRads));
        }

        g_nrHelper->AddX2Interface(g_gnbContainer);

        // ---- UAV mobility (triangle) ------------------------------------ //
        BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.2);
        double uavCenterY = (bbox.minY + bbox.maxY) / 2.0;

        if (uavMobility == "ascend-random")
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
        g_gnbContainer.Create(2);

        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(g_gnbContainer);
        Ptr<ConstantPositionMobilityModel> gnbMobility1 =
            g_gnbContainer.Get(0)->GetObject<ConstantPositionMobilityModel>();
        gnbMobility1->SetPosition(Vector(0.0, 0.0, 25.0));
        Ptr<ConstantPositionMobilityModel> gnbMobility2 =
            g_gnbContainer.Get(1)->GetObject<ConstantPositionMobilityModel>();
        gnbMobility2->SetPosition(Vector(500.0, 0.0, 25.0));

        double startX = intersiteDistance * 0.3;
        double uavY = 0.0;

        if (uavMobility == "ascend-random")
        {
            BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.2);
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
            BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.2);
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
        NetDeviceContainer gnb0 =
            g_nrHelper->InstallGnbDevice(NodeContainer(g_gnbContainer.Get(0)), allBwps);
        g_gnbNrDevs.Add(gnb0.Get(0));
        g_nrHelper->SetGnbAntennaAttribute("BearingAngle", DoubleValue(M_PI)); // rad, pointing -x
        NetDeviceContainer gnb1 =
            g_nrHelper->InstallGnbDevice(NodeContainer(g_gnbContainer.Get(1)), allBwps);
        g_gnbNrDevs.Add(gnb1.Get(0));
    }

    // --- Configure RLC mode per transport protocol ---
    // TCP: RLC AM (link-layer recovery reduces handover-induced TCP retransmissions)
    // UDP: RLC UM (default, no link-layer retransmission needed)
    {
        NrGnbRrc::NrQosFlowToRlcMapping_t rlcMode =
            (transportProtocol == "tcp") ? NrGnbRrc::RLC_UM_ALWAYS : NrGnbRrc::RLC_UM_ALWAYS;
        for (uint32_t i = 0; i < g_gnbContainer.GetN(); ++i)
        {
            Ptr<NrGnbNetDevice> gnbDev =
                g_gnbContainer.Get(i)->GetDevice(0)->GetObject<NrGnbNetDevice>();
            if (gnbDev)
            {
                Ptr<NrGnbRrc> rrc = gnbDev->GetRrc();
                if (rrc)
                {
                    rrc->SetAttribute("QosFlowToRlcMapping", EnumValue(rlcMode));
                }
            }
        }
    }

    g_uavNrDevs = g_nrHelper->InstallUeDevice(g_uavContainer, allBwps);

    internet.Install(g_uavContainer);
    Ipv4InterfaceContainer ueIpIfaces =
        g_nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(g_uavNrDevs));
    if (topology == "hexgrid" || topology == "triangle")
    {
        g_nrHelper->AttachToMaxRsrpGnb(g_uavNrDevs, g_gnbNrDevs);
    }
    else
    {
        g_nrHelper->AttachToGnb(g_uavNrDevs.Get(0), g_gnbNrDevs.Get(0));
    }

    // Dump gNB antenna positions/orientations once (topology fully set up)
    if (logging)
    {
        LogGnbAntennas();
    }

    // ---- Interfering UEs (background UL traffic) ----------------------- //
    if (addInterferingUes > 0)
    {
        // Compute bounding box of gNB positions with padding
        BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.15);

        g_interferingRemoteHostContainer.Create(1);
        Ptr<Node> interferingRemoteHost = g_interferingRemoteHostContainer.Get(0);
        InternetStackHelper interferingInternet;
        interferingInternet.Install(interferingRemoteHost);

        PointToPointHelper p2phInterfering;
        p2phInterfering.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        p2phInterfering.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2phInterfering.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
        NetDeviceContainer interferingInternetDevices =
            p2phInterfering.Install(pgw, interferingRemoteHost);
        Ipv4AddressHelper ipv4hInterfering;
        ipv4hInterfering.SetBase("2.0.0.0", "255.0.0.0");
        Ipv4InterfaceContainer interferingRemoteInterfaces =
            ipv4hInterfering.Assign(interferingInternetDevices);

        Ptr<Ipv4StaticRouting> interferingRemoteHostRouting =
            ipv4RoutingHelper.GetStaticRouting(interferingRemoteHost->GetObject<Ipv4>());
        interferingRemoteHostRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                        Ipv4Mask("255.0.0.0"),
                                                        1);

        g_interferingUeContainer.Create(addInterferingUes);

        // Seed-deterministic: each interfering UE gets its own RNG stream offset
        // (global seed already set at top of scenarioSetup)
        Ptr<UniformRandomVariable> interferingUeRng = CreateObject<UniformRandomVariable>();
        interferingUeRng->SetAttribute("Min", DoubleValue(0.0));
        interferingUeRng->SetAttribute("Max", DoubleValue(1.0));

        for (uint32_t i = 0; i < addInterferingUes; ++i)
        {
            double x = bbox.minX + interferingUeRng->GetValue() * (bbox.maxX - bbox.minX);
            double y = bbox.minY + interferingUeRng->GetValue() * (bbox.maxY - bbox.minY);
            double ueHeight = (interferingUeRng->GetValue() < aerialUeRatio)
                                  ? (50.0 + interferingUeRng->GetValue() * 250.0)
                                  : 1.5;

            if (ueHeight > 1.5)
            {
                // Aerial interfering UE: RandomWaypoint via WaypointMobilityModel
                Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
                g_interferingUeContainer.Get(i)->AggregateObject(wpMob);
                double dwellTime = 2.0;
                Vector startPos(x, y, ueHeight);
                wpMob->AddWaypoint(Waypoint(Seconds(0.0), startPos));
                wpMob->AddWaypoint(Waypoint(Seconds(dwellTime), startPos));
                Vector currentPos = startPos;
                double currentTime = dwellTime;
                Ptr<UniformRandomVariable> wpRng = CreateObject<UniformRandomVariable>();
                wpRng->SetAttribute("Min", DoubleValue(bbox.minX));
                wpRng->SetAttribute("Max", DoubleValue(bbox.maxX));
                Ptr<UniformRandomVariable> wpRngY = CreateObject<UniformRandomVariable>();
                wpRngY->SetAttribute("Min", DoubleValue(bbox.minY));
                wpRngY->SetAttribute("Max", DoubleValue(bbox.maxY));
                while (currentTime < simDuration * 2)
                {
                    Vector nextPos;
                    double dist;
                    do
                    {
                        nextPos = Vector(wpRng->GetValue(), wpRngY->GetValue(), ueHeight);
                        dist = CalculateDistance(currentPos, nextPos);
                    } while (dist < travelLegMin || dist > travelLegMax);
                    double travelTime = dist / 10.0;
                    currentTime += travelTime;
                    wpMob->AddWaypoint(Waypoint(Seconds(currentTime), nextPos));
                    currentPos = nextPos;
                }
            }
            else
            {
                // Ground interfering UE: ConstantPosition (static at starting location)
                MobilityHelper groundMob;
                groundMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
                groundMob.Install(g_interferingUeContainer.Get(i));
                g_interferingUeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(
                    Vector(x, y, 1.5));
            }
        }

        g_interferingUeNrDevs = g_nrHelper->InstallUeDevice(g_interferingUeContainer, allBwps);
        internet.Install(g_interferingUeContainer);
        Ipv4InterfaceContainer interferingUeIfaces =
            g_nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(g_interferingUeNrDevs));

        g_nrHelper->AttachToClosestGnb(g_interferingUeNrDevs, g_gnbNrDevs);

        uint16_t interferingPort = 50001;
        for (uint32_t i = 0; i < addInterferingUes; ++i)
        {
            Ptr<Node> interferingUe = g_interferingUeContainer.Get(i);
            uint16_t port = interferingPort + i;

            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(interferingUe->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(g_nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

            OnOffHelper ulUdp("ns3::UdpSocketFactory",
                              InetSocketAddress(interferingRemoteInterfaces.GetAddress(1), port));
            ulUdp.SetAttribute("DataRate", DataRateValue(DataRate("100Kbps")));
            ulUdp.SetAttribute("PacketSize", UintegerValue(512));
            ulUdp.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            ulUdp.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            auto ulApp = ulUdp.Install(interferingUe);
            ulApp.Start(Seconds(1.0 + i * 0.1));

            PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
            auto ulSinkApp = ulSink.Install(interferingRemoteHost);
            ulSinkApp.Start(Seconds(1.0 + i * 0.1));

            // Downlink: remote host sends UDP back to interfering UE (bidirectional)
            uint16_t dlPort = interferingPort + 100 + i;
            OnOffHelper dlUdp("ns3::UdpSocketFactory",
                              InetSocketAddress(interferingUeIfaces.GetAddress(i), dlPort));
            dlUdp.SetAttribute("DataRate", DataRateValue(DataRate("100Kbps")));
            dlUdp.SetAttribute("PacketSize", UintegerValue(512));
            dlUdp.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            dlUdp.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            auto dlApp = dlUdp.Install(interferingRemoteHost);
            dlApp.Start(Seconds(1.0 + i * 0.1));

            PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            auto dlSinkApp = dlSink.Install(interferingUe);
            dlSinkApp.Start(Seconds(1.0 + i * 0.1));
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
    bool isTcp = (transportProtocol == "tcp");
    std::string socketFactory = isTcp ? "ns3::TcpSocketFactory" : "ns3::UdpSocketFactory";
    PacketSinkHelper packetSinkHelper(socketFactory, sinkAddress);
    auto sinkApp = packetSinkHelper.Install(sinkNode);

    // OnOff data rate (Mbps) — the traffic cap. Single source of truth for the
    // OnOff applications (both TCP and UDP branches).
    const double onOffDataRateMbps = 100.0;
    const double onOffDataRateBps = onOffDataRateMbps * 1e6;

    // Reward/obs goodput reference (Mbps) — SEPARATE from the traffic rate.
    // normGoodput = goodput / ref. With actual goodput ~5-34 Mbps, a 100Mbps
    // reference pins normG in the flat low region of the Deng inverse
    // (R_G = 1/(1+beta*(1-normG))), destroying reward differentiation across
    // cells. Setting the reference near the achievable throughput (~40 Mbps)
    // restores the gradient (RL-AUDIT.md §6, normG analysis 2026-08-04).
    const double rewardRefMbps = rlRewardRefMbps;
    const double rewardRefBps = rewardRefMbps * 1e6;

    if (isTcp)
    {
        // Use OnOff over TCP too: BulkSend floods the NR MAC (no flow control
        // pacing), and OnOff at a fixed rate behaves identically for TCP
        // evaluation while keeping the MAC scheduler healthy.
        OnOffHelper tcpSource(socketFactory, sinkAddress);
        tcpSource.SetAttribute("DataRate", DataRateValue(DataRate(onOffDataRateBps)));
        tcpSource.SetAttribute("PacketSize", UintegerValue(1400));
        tcpSource.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        tcpSource.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        auto sourceApp = tcpSource.Install(sourceNode);
        sourceApp.Start(Seconds(1.0));
        sinkApp.Start(Seconds(1.0));

        // OnOffApplication has no ConnectionSucceeded/ConnectionFailed traces
        // (BulkSend-only); TCP connection state is tracked via the socket
        // State trace in NotifyTcpStateChange (sets g_tcpConnected on ESTABLISHED).
        std::string senderAppConfigPath = "/NodeList/" + std::to_string(g_senderNodeId) +
                                          "/ApplicationList/*/$ns3::OnOffApplication/";
        std::string senderTcpBasePath =
            "/NodeList/" + std::to_string(g_senderNodeId) + "/$ns3::TcpL4Protocol/SocketList/*/";

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
            std::cout << "DEBUG t=" << Simulator::Now().GetSeconds()
                      << " rrcState=" << rrc->GetState() << " cellId=" << rrc->GetCellId()
                      << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal() << std::endl;
        });

        // Schedule metric-accumulating traces (always connected, regardless of logging)
        Simulator::Schedule(Seconds(1.1), [senderTcpBasePath]() {
            Config::ConnectWithoutContext(senderTcpBasePath + "Retransmission",
                                          MakeCallback(&SourceRetransmissionPacket));

            std::string rxPath = "/NodeList/" + std::to_string(g_receiverNodeId) +
                                 "/ApplicationList/*/$ns3::PacketSink/Rx";
            Config::ConnectWithoutContext(rxPath, MakeCallback(&SinkRxPacket));

            Config::ConnectWithoutContext(senderTcpBasePath + "RTT", MakeCallback(&TcpRttChange));

            Config::ConnectWithoutContext(senderTcpBasePath + "State",
                                          MakeCallback(&NotifyTcpStateChange));

            Config::ConnectWithoutContext(senderTcpBasePath + "CongState",
                                          MakeCallback(&CongestionStateLogger));
        });

        if (logging)
        {
            Simulator::Schedule(Seconds(1.1), [senderAppConfigPath, senderTcpBasePath]() {
                Config::ConnectWithoutContext(senderTcpBasePath + "CongestionWindow",
                                              MakeCallback(&CwndTracer));

                Config::ConnectWithoutContext(senderAppConfigPath + "TxWithAddresses",
                                              MakeCallback(&SourceTxPacket));
            });
        }

        // BBR-specific traces
        if (tcpVariant == "TcpBbr")
        {
            Simulator::Schedule(Seconds(1.1), [senderTcpBasePath]() {
                Config::ConnectWithoutContext(senderTcpBasePath +
                                                  "CongestionOps/$ns3::TcpBbr/PacingGain",
                                              MakeCallback(&BbrPacingGainChange));

                Config::ConnectWithoutContext(senderTcpBasePath +
                                                  "CongestionOps/$ns3::TcpBbr/CwndGain",
                                              MakeCallback(&BbrCwndGainChange));

                Config::ConnectWithoutContext(senderTcpBasePath + "RateOps/TcpRateSampleUpdated",
                                              MakeCallback(&TcpRateSampleChange));
            });
        }
    }
    else
    {
        // UDP: use OnOff for both standalone and RL mode (BulkSend over UDP
        // has no flow control and overwhelms the NR MAC at line rate)
        OnOffHelper udpSource(socketFactory, sinkAddress);
        udpSource.SetAttribute("DataRate", DataRateValue(DataRate(onOffDataRateBps)));
        udpSource.SetAttribute("PacketSize", UintegerValue(1400));
        udpSource.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        udpSource.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        auto sourceApp = udpSource.Install(sourceNode);
        sourceApp.Start(Seconds(1.0));
        sinkApp.Start(Seconds(1.0));

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
            std::cout << "DEBUG t=" << Simulator::Now().GetSeconds()
                      << " rrcState=" << rrc->GetState() << " cellId=" << rrc->GetCellId()
                      << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal() << std::endl;
        });

        // UDP: only SinkRx trace for throughput
        Simulator::Schedule(Seconds(1.1), []() {
            std::string rxPath = "/NodeList/" + std::to_string(g_receiverNodeId) +
                                 "/ApplicationList/*/$ns3::PacketSink/Rx";
            Config::ConnectWithoutContext(rxPath, MakeCallback(&SinkRxPacket));
        });
    }

    // Non-TCP-specific traces (always connected, independent of transport protocol)
    Config::Connect("/NodeList/" + std::to_string(uavNodeId) + "/$ns3::MobilityModel/CourseChange",
                    MakeCallback(&MobilityCourseChange));

    // Periodic position logging (every 500ms) — fills gaps between CourseChange events
    Simulator::Schedule(Seconds(0.0), &UavPeriodicPositionLog);

    // PHY-layer traces (always on UAV, independent of flow direction)
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/$ns3::NrUeNetDevice/"
                                      "ComponentCarrierMapUe/*/NrUePhy/"
                                      "ReportUeMeasurements",
                                  MakeCallback(&LogUeMeasReport));

    // DL data SINR (always on UE PHY, independent of flow direction)
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/"
                                      "$ns3::NrUeNetDevice/"
                                      "ComponentCarrierMapUe/*/NrUePhy/"
                                      "DlDataSinr",
                                  MakeCallback(&DlDataSinrLogger));

    // UL SINR trace (gNB PHY UlSinrTrace, always)
    // Use direct pointer iteration over g_gnbNrDevs for reliable connection
    // across all topologies (Config path with wildcards fails in some setups).
    for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> phy = NrHelper::GetGnbPhy(g_gnbNrDevs.Get(i), 0);
        if (phy)
        {
            uint16_t gnbCellId = g_gnbNrDevs.Get(i)->GetObject<NrGnbNetDevice>()->GetCellId();
            phy->TraceConnectWithoutContext("UlSinrTrace",
                                            MakeBoundCallback(&UlSrsSinrLogger, gnbCellId));
            phy->TraceConnectWithoutContext("SlotDataStats", MakeCallback(&SlotDataStatsLogger));
            phy->TraceConnectWithoutContext("UlHarqFeedbackTrace",
                                            MakeCallback(&UlHarqFeedbackLogger));
            // Connect gNB RX packet trace for proper UL SINR (post-beamforming MIMO SINR)
            Ptr<NrSpectrumPhy> gnbSpectrumPhy = phy->GetSpectrumPhy();
            if (gnbSpectrumPhy)
            {
                gnbSpectrumPhy->TraceConnectWithoutContext(
                    "RxPacketTraceGnb",
                    MakeBoundCallback(&UlRxPacketLogger, gnbCellId));
            }
            // Connect MAC traces (SrReq, DlScheduling, UlScheduling) with cellId bound
            Ptr<NrGnbMac> mac = NrHelper::GetGnbMac(g_gnbNrDevs.Get(i), 0);
            if (mac)
            {
                mac->TraceConnectWithoutContext("SrReq",
                                                MakeBoundCallback(&SrReqLogger, gnbCellId));
                mac->TraceConnectWithoutContext("DlScheduling",
                                                MakeBoundCallback(&DlSchedulingLogger, gnbCellId));
                mac->TraceConnectWithoutContext("UlScheduling",
                                                MakeBoundCallback(&UlSchedulingLogger, gnbCellId));
            }
        }
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
            uePhy->TraceConnectWithoutContext("UePhyTxedHarqFeedbackTrace",
                                              MakeCallback(&HarqFeedbackLogger));
        }
    });

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrNetDevice/$ns3::NrUeNetDevice/NrUeRrc"
                    "/StateTransition",
                    MakeCallback(&UavRrcStateChange));

    // Add X2 Interface (already added inside hexgrid block)
    if (topology == "simple")
    {
        g_nrHelper->AddX2Interface(g_gnbContainer);
    }

    // ---- RL Framework: Install handover RL apps on UAV node ---- //
    if (rlMode)
    {
        uint32_t numBs = g_gnbContainer.GetN();
        uint32_t uavNodeId = g_uavContainer.Get(0)->GetId();
        g_lastRsrpValues.resize(numBs + 1, -140.0); // index by cellId (1-based), dBm
        g_lastSinrValues.resize(numBs + 1, -40.0);  // index by cellId (1-based), -40dB = unknown
        g_lastRsrqValues.resize(numBs + 1, -20.0);  // index by cellId (1-based), dB

        RlApplicationHelper rlAppHelper(NrRlHandoverRewardApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("StopTime", TimeValue(Seconds(simDuration)));
        rlAppHelper.SetAttribute("AlphaGoodput", DoubleValue(rlAlphaGoodput));
        rlAppHelper.SetAttribute("BetaGoodput", DoubleValue(rlBetaGoodput));
        rlAppHelper.SetAttribute("BetaHandover", DoubleValue(rlBetaHandover));
        rlAppHelper.SetAttribute("PingPongBetaMultiplier", DoubleValue(rlPingPongMultiplier));
        rlAppHelper.SetAttribute("HandoverRatePenaltyEnabled", BooleanValue(rlHandoverRatePenalty));
        rlAppHelper.SetAttribute("HandoverRateWindowMs", UintegerValue(rlHandoverRateWindowMs));
        rlAppHelper.SetAttribute("HandoverRateBudget", UintegerValue(rlHandoverRateBudget));
        rlAppHelper.SetAttribute("HandoverRatePenaltyLambda", DoubleValue(rlHandoverRateLambda));
        rlAppHelper.SetAttribute("RewardComposition", StringValue(rlRewardComposition));
        rlAppHelper.SetAttribute("RewardGoodputShape", StringValue(rlRewardGoodputShape));
        rlAppHelper.SetAttribute("RewardGoodputAlpha", DoubleValue(rlRewardGoodputAlpha));
        rlAppHelper.SetAttribute("RewardGoodputP", DoubleValue(rlRewardGoodputP));
        rlAppHelper.SetAttribute("GoodputRefBps", DoubleValue(rewardRefBps));
        rlAppHelper.SetAttribute("CalculationInterval", TimeValue(MilliSeconds(stepTime)));
        auto rewardApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverAgentApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("TopN", UintegerValue(rlTopN)); // action space: Discrete(rlTopN+1)
        auto agentApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverObservationApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        // Obs ranks a fixed 5 slots (more context than the action space).
        rlAppHelper.SetAttribute("TopN", UintegerValue(5));
        rlAppHelper.SetAttribute("UavNodeId", UintegerValue(uavNodeId));
        rlAppHelper.SetAttribute("StepTimeMs", UintegerValue(stepTime));
        rlAppHelper.SetAttribute("SinrEwmaAlpha", DoubleValue(0.1));
        rlAppHelper.SetAttribute("GoodputRefBps", DoubleValue(rewardRefBps));
        rlAppHelper.SetAttribute("UseTbsObservation", BooleanValue(useTbsObservation));
        rlAppHelper.SetAttribute("HandoverRateWindowMs", UintegerValue(rlHandoverRateWindowMs));
        auto obsApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverActionApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("TopN", UintegerValue(rlTopN));
        rlAppHelper.SetAttribute("HandoverAlgorithm", StringValue(handoverAlgorithm));
        rlAppHelper.SetAttribute("HandoverDebounceMs", UintegerValue(rlHandoverDebounceMs));
        auto actApps = rlAppHelper.Install(g_uavContainer.Get(0));

        CommunicationHelper commHelper;
        commHelper.SetAgentApps(agentApps);
        commHelper.SetActionApps(actApps);
        commHelper.SetObservationApps(obsApps);
        commHelper.SetRewardApps(rewardApps);
        commHelper.SetIds();

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
    }
}
