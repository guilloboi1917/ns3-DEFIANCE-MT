/**
 * Setup file for the HARL NR Scenario
 * Supports multiple topologies:
 *   "simple"  — 2 gNodeBs on a line, UAV shuttles between them
 *   "triangle" — 3 sites, 3 sectors each
 *   "hexgrid" — hexagonal grid of 3-sector macro sites
 *
 * 1 aerial UE, 1 remote server. The UE runs a TCP/UDP OnOffApplication to the
 * remote server.
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
#include <unordered_map>
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
              << "\n";
}

void
NotifyConnectionFailed(Ptr<Socket> socket, const Address& local, const Address& remote)
{
    g_tcpAlive = false;
    std::cout << "TCP connection failed at time " << Simulator::Now().GetSeconds() << "s"
              << "\n";
}

void
NotifyTcpStateChange(const TcpSocket::TcpStates_t oldState, const TcpSocket::TcpStates_t newState)
{
    const char* oldName = TcpSocket::TcpStateName[oldState];
    const char* newName = TcpSocket::TcpStateName[newState];

    // std::cout << "TCP state: " << oldName << " -> " << newName
    //           << " at t=" << Simulator::Now().GetSeconds() << "s" << "\n";

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

// ------------------------------------------------------------------------- //
// Persistent CSV streams: each file opens once on first use and appends for
// the rest of the run (per-event open/close dominated the logging overhead).
// ------------------------------------------------------------------------- //
namespace
{
// Persistent per-file output streams (see LogStream below).
std::unordered_map<std::string, std::ofstream> g_csvStreams;
} // namespace

std::ofstream&
LogStream(const std::string& fileName)
{
    auto it = g_csvStreams.find(fileName);
    if (it == g_csvStreams.end())
    {
        it = g_csvStreams
                 .emplace(std::piecewise_construct,
                          std::forward_as_tuple(fileName),
                          std::forward_as_tuple(g_outputDir + fileName,
                                                std::ios_base::app))
                 .first;
    }
    return it->second;
}

// Flush all persistent CSV streams once. The loggers write with "\n" (no
// per-line flush — see LogStream); the caller invokes this right after
// Simulator::Run() so the log files are complete when the run summary prints.
void
FlushLogStreams()
{
    for (auto& [fileName, stream] : g_csvStreams)
    {
        stream.flush();
    }
}

// TCP congestion state logger — logs CA_OPEN, CA_DISORDER, CA_RECOVERY, CA_LOSS, CA_CWR
void
CongestionStateLogger(TcpSocketState::TcpCongState_t oldState,
                      TcpSocketState::TcpCongState_t newState)
{
    const char* oldName = TcpSocketState::TcpCongStateName[oldState];
    const char* newName = TcpSocketState::TcpCongStateName[newState];

    // std::cout << "TCP congestion: " << oldName << " -> " << newName
    //           << " at t=" << Simulator::Now().GetSeconds() << "s" << "\n";

    std::ofstream& congFile = LogStream("nr-rl-congestion.csv");
    congFile << Simulator::Now().GetSeconds() << "," << oldName << "," << newName << "\n";
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
    std::ofstream& slotFile = LogStream("nr-rl-slot-stats.csv");
    slotFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << scheduledUe
             << "," << usedReg << "," << usedSym << "," << availableRb << "," << availableSym << ","
             << (int)utilPct << "\n";
}

// CWND tracing callback — fires on every CWND change (every ACK)
void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream& cwndFile = LogStream("nr-rl-cwnd.csv");
    cwndFile << Simulator::Now().GetSeconds() << "," << newCwnd << "\n";
}

// SrReq logger — fires when a UE sends a Scheduling Request
void
SrReqLogger(uint16_t cellId, uint8_t bwpId, uint16_t rnti)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream& srFile = LogStream("nr-rl-sr.csv");
    srFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << (uint32_t)bwpId
           << "," << (uint32_t)rnti << "\n";
}

// DlScheduling logger — fires for every DL scheduling decision
void
DlSchedulingLogger(uint16_t cellId, NrSchedulingCallbackInfo traceInfo)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream& dlSchedFile = LogStream("nr-rl-dl-sched.csv");
    dlSchedFile << Simulator::Now().GetSeconds() << "," << cellId << ","
                << (uint32_t)traceInfo.m_rnti << "," << (uint32_t)traceInfo.m_mcs << ","
                << traceInfo.m_tbSize << "," << (uint32_t)traceInfo.m_symStart << ","
                << (uint32_t)traceInfo.m_numSym << "\n";
}

// UlScheduling logger — fires for every UL scheduling decision
void
UlSchedulingLogger(uint16_t cellId, NrSchedulingCallbackInfo traceInfo)
{
    if (!g_logging)
    {
        return;
    }
    std::ofstream& ulSchedFile = LogStream("nr-rl-ul-sched.csv");
    ulSchedFile << Simulator::Now().GetSeconds() << "," << cellId << ","
                << (uint32_t)traceInfo.m_rnti << "," << (uint32_t)traceInfo.m_mcs << ","
                << traceInfo.m_tbSize << "," << (uint32_t)traceInfo.m_symStart << ","
                << (uint32_t)traceInfo.m_numSym << "\n";
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
        std::ofstream& rttFile = LogStream("nr-rl-rtt.csv");
        rttFile << Simulator::Now().GetSeconds() << "," << rttMs << "\n";
    }
}

void
BbrPacingGainChange(double oldValue, double newValue)
{
    std::ofstream& pacingGainFile = LogStream("nr-rl-pacing-gain.csv");
    pacingGainFile << Simulator::Now().GetSeconds() << "," << newValue << "\n";
}

void
BbrCwndGainChange(double oldValue, double newValue)
{
    std::ofstream& cwndGainFile = LogStream("nr-rl-cwnd-gain.csv");
    cwndGainFile << Simulator::Now().GetSeconds() << "," << newValue << "\n";
}

void
TcpRateSampleChange(const TcpRateOps::TcpRateSample& sample)
{
    std::ofstream& rateFile = LogStream("nr-rl-rate.csv");
    rateFile << Simulator::Now().GetSeconds() << "," << sample.m_deliveryRate.GetBitRate()
             << "\n";
}

// Track handovers. Connected to the UAV's RRC only (UAV-specific Config
// path), so no imsi filter is needed.
void
UavRrcStateChange(uint64_t imsi,
                  uint16_t cellId,
                  uint16_t rnti,
                  NrUeRrc::State oldState,
                  NrUeRrc::State newState)
{
    // std::cout << "RRC state change for UE " << imsi << ", RNTI " << rnti << " to cell " << cellId
    //           << " (state " << oldState << " -> " << newState << ") at time "
    //           << Simulator::Now().GetSeconds() << "s" << "\n";
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
            std::ofstream& rlfFile = LogStream("nr-rl-rlf.csv");
            rlfFile << Simulator::Now().GetSeconds() << "," << cellId << "\n";
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
                  << " at time " << Simulator::Now().GetSeconds() << "s" << "\n";
        std::ofstream& hoFile = LogStream("nr-rl-handovers.csv");
        hoFile << Simulator::Now().GetSeconds() << "," << cellId << "\n";
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
              << " time " << Simulator::Now().GetSeconds() << "s" << "\n";
    g_handoverInProgress = false;
}

// Log ReportUeMeasurements (averaged, dBm/dB, all cells) — the same trace
// source used by the obs app.
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
    std::ofstream& ueMeasFile = LogStream("ue_meas_report.csv");
    ueMeasFile << Simulator::Now().GetSeconds() << "," << (int)cellId << "," << (int)rnti << ","
               << rsrp << "," << rsrq << "," << (int)isServingCell << "\n";
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
    std::ofstream& dlSinrFile = LogStream("dl_sinr.csv");
    dlSinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << rnti << "," << sinrDb
               << "\n";
}

// UL HARQ feedback logger (gNB PHY)
void
UlHarqFeedbackLogger(uint16_t rnti, bool isReceivedOk)
{
    if (g_logging)
    {
        std::ofstream& harqFile = LogStream("nr-rl-ul-harq.csv");
        harqFile << Simulator::Now().GetSeconds() << "," << rnti << "," << (int)isReceivedOk
                 << "\n";
    }
}

// UL RX packet logger (gNB spectrum PHY, post-beamforming MIMO SINR)
void
UlRxPacketLogger(uint16_t cellId, RxPacketTraceParams params)
{
    if (g_logging)
    {
        std::ofstream& sinrFile = LogStream("nr-rl-ul-rx-sinr.csv");
        double sinrDb = (params.m_sinr > 0.0) ? (10.0 * std::log10(params.m_sinr)) : -40.0;
        sinrFile << Simulator::Now().GetSeconds() << "," << cellId << "," << params.m_rnti << ","
                 << sinrDb << "," << (int)params.m_mcs << "," << params.m_tbSize << ","
                 << (int)params.m_corrupt << "," << params.m_tbler << "\n";
    }
}

// DL RX packet logger (UE spectrum PHY, post-beamforming MIMO SINR) - the DL
// mirror of nr-rl-ul-rx-sinr.csv. Same schema: t, cellId, rnti, sinrDb, mcs,
// tbSize, corrupt, tbler; cellId is the serving cell at TB time
// (g_currentCellId - changes on handover). Source: RxPacketTraceUe on the UE's
// NrSpectrumPhy.
void
DlRxPacketLogger(RxPacketTraceParams params)
{
    if (g_logging)
    {
        std::ofstream& sinrFile = LogStream("nr-rl-dl-rx-sinr.csv");
        double sinrDb = (params.m_sinr > 0.0) ? (10.0 * std::log10(params.m_sinr)) : -40.0;
        sinrFile << Simulator::Now().GetSeconds() << "," << (uint32_t)g_currentCellId << ","
                 << params.m_rnti << "," << sinrDb << "," << (int)params.m_mcs << ","
                 << params.m_tbSize << "," << (int)params.m_corrupt << "," << params.m_tbler
                 << "\n";
    }
}

// UL SINR logged to ul_sinr_srs.csv. The trace fires from the UL DATA CQI
// report (GenerateDataCqiReport -> pData chunk) — despite the "srs" name it
// carries the full-band per-RB SINR spectrum, time-averaged by the
// NrChunkProcessor (the same SINR the scheduler uses for UL CQI/MCS
// selection). RBs not used by the UL transmission sit at the -40 dB sentinel,
// so the average covers only the RBs with a real SINR (> 1e-12).
//
// NOTE: when gNBs share the exact same position (e.g., co-located sectors
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
        if (*it > 1e-12)
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
    std::ofstream& ulSrsFile = LogStream("ul_sinr_srs.csv");
    ulSrsFile << Simulator::Now().GetSeconds() << "," << (uint32_t)cellId << "," << sinrDb
              << "\n";
}

// UL SINR capture (always on): RB-averaged UL SINR per slot ->
// g_lastSinrValues[cellId] (latest) and the per-cell step accumulators
// g_ulSinrSum/g_ulSinrCount (mean over the obs step, reset by the obs app).
// Same full-band fix as UlSrsSinrLogger: average only RBs with a real SINR.
void
CaptureUlSrsSinr(uint16_t cellId, uint64_t /* imsi */, SpectrumValue& sinrSpectrum, SpectrumValue& /* interference */)
{
    double sumSinr = 0.0;
    uint32_t numRb = 0;
    for (auto it = sinrSpectrum.ConstValuesBegin(); it != sinrSpectrum.ConstValuesEnd(); ++it)
    {
        if (*it > 1e-12)
        {
            sumSinr += 10.0 * std::log10(*it);
            numRb++;
        }
    }
    if (numRb > 0 && cellId < g_lastSinrValues.size())
    {
        const double rbMean = sumSinr / static_cast<double>(numRb);
        g_lastSinrValues[cellId] = rbMean;
        if (cellId < g_ulSinrSum.size())
        {
            g_ulSinrSum[cellId] += rbMean;
            g_ulSinrCount[cellId]++;
        }
    }
}

// UL slot-stat capture (always on): serving-cell UL RB utilization + scheduled
// UE count -> globals for the obs UL-load features.
void
CaptureSlotDataStats(const SfnSf& /* sfn */,
                     uint32_t scheduledUe,
                     uint32_t usedReg,
                     uint32_t /* dataSym */,
                     uint32_t availableRb,
                     uint32_t availableSym,
                     uint16_t /* bwpId */,
                     uint16_t cellId)
{
    if (cellId != static_cast<uint16_t>(g_currentCellId))
    {
        return;
    }
    g_ulServingSchedUe = scheduledUe;
    g_ulServingRbUtil = (availableRb > 0 && availableSym > 0)
                            ? std::min(1.0, static_cast<double>(usedReg) / (availableRb * availableSym))
                            : 0.0;
}

void
UavPeriodicPositionLog()
{
    if (g_uavContainer.GetN() > 0)
    {
        Ptr<MobilityModel> mob = g_uavContainer.Get(0)->GetObject<MobilityModel>();
        if (mob)
        {
            std::ofstream& mobilityFile = LogStream("mobility.csv");
            mobilityFile << Simulator::Now().GetSeconds() << "," << mob->GetPosition() << "," << 0
                         << "\n";
        }
    }
    // Re-schedule every 500ms
    Simulator::Schedule(MilliSeconds(500), &UavPeriodicPositionLog);
}

void
MobilityCourseChange(std::string context, Ptr<const MobilityModel> model)
{
    std::ofstream& mobilityFile = LogStream("mobility.csv");
    mobilityFile << Simulator::Now().GetSeconds() << "," << model->GetPosition() << "," << 1
                 << "\n";
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
    antFile << "cellId,x,y,z,bearingDeg,downtiltDeg" << "\n";
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
                << antenna->GetBeta() * 180.0 / M_PI << "\n";
    }
}

void
SinkRxPacket(Ptr<const Packet> packet, const Address& address)
{
    uint32_t size = packet->GetSize();
    g_totalRxBytes += size;
    if (g_logging)
    {
        std::ofstream& sinkFile = LogStream("sink-packets.csv");
        sinkFile << Simulator::Now().GetSeconds() << "," << size << "\n";
    }
}

void
SourceTxPacket(Ptr<const Packet> packet, const Address& local, const Address& remote)
{
    std::ofstream& sourceBulkSenderFile = LogStream("source-packets.csv");
    sourceBulkSenderFile << Simulator::Now().GetSeconds() << "," << packet->GetSize() << "\n";
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
        std::ofstream& retransmissionFile = LogStream("retransmissions.csv");
        retransmissionFile << Simulator::Now().GetSeconds() << "," << packet->GetSize()
                           << "\n";
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

/**
 * @brief Record a waypoint in the plan (mobility model + trajectory-obs table).
 */
static void
AddUavWaypoint(Ptr<WaypointMobilityModel> mob, Time t, const Vector& p)
{
    Waypoint w(t, p);
    mob->AddWaypoint(w);
    g_uavWaypoints.push_back(w);
}

/**
 * @brief Install the UAV mobility model and generate its waypoint plan.
 *
 * The plan depends ONLY on (seed, runId): the waypoint RNGs are pinned to
 * fixed streams (100/101/102) so rlMode must not shift the waypoints
 * (trajectory isolation for the matched A3-vs-RL comparison). Draw ORDER is
 * preserved per topology: hexgrid/triangle draw the start position from
 * rbx/rby (randomStart=true); simple uses a centered start (no start draws).
 * Do NOT reorder the draws — that would change every trajectory and
 * invalidate all baselines.
 *
 * @param uav UAV node
 * @param bbox movement bounding box
 * @param startHeight initial altitude (m)
 * @param endHeight max altitude (m)
 * @param ueSpeed speed (m/s)
 * @param simDuration sim duration (s) — plan generated up to 2x
 * @param travelLegMin minimum leg length (m)
 * @param travelLegMax maximum leg length (m)
 * @param randomStart true = draw start (x,y) from rbx/rby; false = centered
 * @param constantPos position for uavMobility == "constant" (per topology)
 * @param mobility "constant" | "ascend-random" | "random-waypoint"
 */
static void
InstallUavMobility(Ptr<Node> uav,
                   const BoundingBox& bbox,
                   double startHeight,
                   double endHeight,
                   double ueSpeed,
                   double simDuration,
                   double travelLegMin,
                   double travelLegMax,
                   bool randomStart,
                   const Vector& constantPos,
                   const std::string& mobility)
{
    if (mobility == "constant")
    {
        MobilityHelper uavMob;
        uavMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        uavMob.Install(uav);
        uav->GetObject<ConstantPositionMobilityModel>()->SetPosition(constantPos);
        return;
    }

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
    // Pin the UAV waypoint RNGs to fixed streams so the plan depends only on
    // (seed, runId) — rlMode must not shift the waypoints (trajectory isolation).
    rbx->SetStream(100);
    rby->SetStream(101);
    rbz->SetStream(102);

    Ptr<WaypointMobilityModel> wpMob = CreateObject<WaypointMobilityModel>();
    uav->AggregateObject(wpMob);

    double currentTime;
    Vector currentPos;
    if (mobility == "ascend-random")
    {
        Vector startPos = randomStart
                              ? Vector(rbx->GetValue(), rby->GetValue(), 1.5)
                              : Vector((bbox.minX + bbox.maxX) / 2.0,
                                       (bbox.minY + bbox.maxY) / 2.0,
                                       1.5);
        AddUavWaypoint(wpMob, Seconds(0.0), startPos);
        AddUavWaypoint(wpMob, Seconds(dwellTime), startPos);
        double ascentTime = (startHeight - 1.5) / ueSpeed;
        Vector ascentEnd(startPos.x, startPos.y, startHeight);
        AddUavWaypoint(wpMob, Seconds(dwellTime + ascentTime), ascentEnd);
        currentPos = ascentEnd;
        currentTime = dwellTime + ascentTime;
    }
    else // "random-waypoint"
    {
        Vector startPos = randomStart
                              ? Vector(rbx->GetValue(), rby->GetValue(), startHeight)
                              : Vector((bbox.minX + bbox.maxX) / 2.0,
                                       (bbox.minY + bbox.maxY) / 2.0,
                                       startHeight);
        AddUavWaypoint(wpMob, Seconds(0.0), startPos);
        AddUavWaypoint(wpMob, Seconds(dwellTime), startPos);
        currentPos = startPos;
        currentTime = dwellTime;
    }

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
        AddUavWaypoint(wpMob, Seconds(currentTime), nextPos);
        currentPos = nextPos;
    }
}


// ------------------------------------------------------------------------- //
// scenarioSetup
// ------------------------------------------------------------------------- //
inline void
scenarioSetup(std::string flowDirection = "ul",
              std::string transportProtocol = "udp",
              double ueSpeed = 20.0,            // m/s
              double simDuration = 50.0,        // seconds
              uint32_t bandwidthMhz = 10,       // MHz
              double trafficRateMbps = 50.0,   // UAV OnOff data rate (Mbps)
              double intersiteDistance = 500.0, // m
              uint32_t numMacroCells = 7,       // number of macro sites
              double gnbDowntilt = 10.0,        // degrees
              uint32_t seed = 0,                // Seed for RNG
              uint32_t runId = 0,
              std::string trialName = "1",
              std::string tcpVariant = "TcpBbr",
              std::string uavMobility = "random-waypoint",
              std::string topology = "triangle",
              double startHeight = 50.0,
              double endHeight = 200.0,
              uint32_t bbrWindowLength = 10,
              uint32_t addInterferingUes = 0,
              double aerialUeRatio = 0.0,
              std::string interfererMobility = "static",
              bool logging = false,
              bool rlMode = false,
              std::string handoverAlgorithm = "a3",
              uint32_t stepTime = 400,
              uint32_t delay = 0,
              double rlAlphaGoodput = 0.8,
              double rlBetaGoodput = 5.0,
              double rlBetaHandover = 5.0,
              const std::string& rlRewardComposition = "multiplicative",
              double rlPingPongMultiplier = 5.0,
              uint32_t rlHandoverHangoverLength = 1,
              uint32_t rlHandoverRateWindowMs = 10000,
              double rlRewardRefMbps = 15.0,
              std::string rlRewardGoodputShape = "compl_pwr",
              double rlRewardGoodputAlpha = 3.0,
              double rlRewardGoodputP = 0.4,
              uint32_t rlcTxBufferBytes = 180000, // RLC TX buffer cap (0 = unlimited)
              std::string errorModel = "eesm-ir-t1", // eesm-ir-t1 | eesm-ir-t2 | eesm-cc-t2 | eesm-cc-t1 | lte-mi
              uint32_t channelUpdateMs = 20,       // channel UpdatePeriod ms (0 = disabled/module default)
              uint32_t numerology = 0,             // 0 = 15 kHz SCS, 1 = 30 kHz SCS (n78 typical)
              std::string channelModel = "umav",   // channel: umav (3GPP UMa-AV, default) | tworay (TwoRaySpectrumPropagationLossModel)
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

    // Episode duration for the obs-app lookahead clamp (pos2s).
    g_simDuration = simDuration;

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
        // Clear data files and write one-time CSV headers (the loggers below
        // only ever append). Every log file is self-describing: header rows
        // name the columns, so the Python readers (plot-nr-rl-stats.py,
        // run-evaluation.py, feature-importance.py, ...) parse by name.
        // gnb-antennas.csv is excluded: LogGnbAntennas() writes its own header
        // (truncate mode). Keep this map in sync with the loggers and readers.
        const std::vector<std::pair<std::string, std::string>> csvHeaders = {
            {"nr-rl-cwnd.csv",       "time,cwnd"},
            {"nr-rl-handovers.csv",  "time,cellId"},
            {"nr-rl-rate.csv",       "time,deliveryRateBps"},
            {"nr-rl-cwnd-gain.csv",  "time,cwndGain"},
            {"nr-rl-pacing-gain.csv","time,pacingGain"},
            {"nr-rl-rtt.csv",        "time,rttMs"},
            {"dl_sinr.csv",          "time,cellId,rnti,sinrDb"},
            {"ul_sinr_srs.csv",      "time,cellId,sinrDb"},
            {"ue_meas_report.csv",   "time,cellId,rnti,rsrpDbm,rsrqDb,isServingCell"},
            {"mobility.csv",         "time,position,isWaypoint"},
            {"sink-packets.csv",     "time,packetSizeBytes"},
            {"source-packets.csv",   "time,packetSizeBytes"},
            {"retransmissions.csv",  "time,packetSizeBytes"},
            {"rl_obs.csv",           "time,serving_rsrp,serving_rsrq,slot_rsrp_0,slot_rsrp_1,slot_rsrp_2,rsrp_delta_0,rsrp_delta_1,rsrp_delta_2,dl_sinr,time_since_ho,norm_goodput,ho_count_10s,ul_sinr,ul_rb_util,ul_sched_ue,d_serving_rsrp,d_serving_sinr,d_serving_rsrq,d_norm_goodput,d_margin_best,d_slot_rsrp_0,d_slot_rsrp_1,d_slot_rsrp_2,heading_x,heading_y,heading_z,pos_x,pos_y,pos_z,pos2s_x,pos2s_y,pos2s_z"},
            {"rl_reward.csv",        "time,goodputMbps,dynRefMbps,dynMinMbps,normGoodputRaw,normGoodput,R_G,I_ho,R_H,pingPong,reward"},
            {"rl_action.csv",        "time,currentCellId,targetCellId,srcRsrpDbm,targetRsrpDbm"},
            {"rl_actions_full.csv",  "time,actionIndex,targetCellId,currentCellId,outcome"},
            {"nr-rl-congestion.csv", "time,oldState,newState"},
            {"nr-rl-slot-stats.csv", "time,cellId,scheduledUe,usedReg,usedSym,availableRb,availableSym,utilPct"},
            {"nr-rl-rlf.csv",        "time,cellId"},
            {"nr-rl-sr.csv",         "time,cellId,bwpId,rnti"},
            {"nr-rl-dl-sched.csv",   "time,cellId,rnti,mcs,tbSize,symStart,numSym"},
            {"nr-rl-ul-sched.csv",   "time,cellId,rnti,mcs,tbSize,symStart,numSym"},
            {"nr-rl-ul-harq.csv",    "time,rnti,isReceivedOk"},
            {"nr-rl-ul-rx-sinr.csv", "time,cellId,rnti,sinrDb,mcs,tbSize,corrupt,tbler"},
            {"nr-rl-dl-rx-sinr.csv", "time,cellId,rnti,sinrDb,mcs,tbSize,corrupt,tbler"},
        };
        for (const auto& [fileName, header] : csvHeaders)
        {
            std::ofstream fout(g_outputDir + fileName);
            fout << header << "\n";
        }

        // Write meta.yaml with input parameters for reproducibility
        std::ofstream metaOut(g_outputDir + "meta.yaml");
        metaOut << "# Simulation metadata — auto-generated by scenarioSetup" << "\n";
        metaOut << "flowDirection: " << flowDirection << "\n";
        metaOut << "transportProtocol: " << transportProtocol << "\n";
        metaOut << "ueSpeed: " << ueSpeed << "\n";
        metaOut << "simDuration: " << simDuration << "\n";
        metaOut << "bandwidthMhz: " << bandwidthMhz << "\n";
        metaOut << "trafficRateMbps: " << trafficRateMbps << "\n";
        metaOut << "intersiteDistance: " << intersiteDistance << "\n";
        metaOut << "numMacroCells: " << numMacroCells << "\n";
        metaOut << "gnbDowntilt: " << gnbDowntilt << "\n";
        metaOut << "seed: " << seed << "\n";
        metaOut << "runId: " << runId << "\n";
        metaOut << "trialName: " << trialName << "\n";
        metaOut << "tcpVariant: " << tcpVariant << "\n";
        metaOut << "uavMobility: " << uavMobility << "\n";
        metaOut << "topology: " << topology << "\n";
        metaOut << "startHeight: " << startHeight << "\n";
        metaOut << "endHeight: " << endHeight << "\n";
        metaOut << "bbrWindowLength: " << bbrWindowLength << "\n";
        metaOut << "addInterferingUes: " << addInterferingUes << "\n";
        metaOut << "aerialUeRatio: " << aerialUeRatio << "\n";
        metaOut << "rlMode: " << (rlMode ? "true" : "false") << "\n";
        metaOut << "handoverAlgorithm: " << handoverAlgorithm << "\n";
        metaOut << "stepTime: " << stepTime << "\n";
        metaOut << "delay: " << delay << "\n";
        metaOut << "rlAlphaGoodput: " << rlAlphaGoodput << "\n";
        metaOut << "rlBetaGoodput: " << rlBetaGoodput << "\n";
        metaOut << "rlBetaHandover: " << rlBetaHandover << "\n";
        metaOut << "rlRewardComposition: " << rlRewardComposition << "\n";
        metaOut << "rlPingPongMultiplier: " << rlPingPongMultiplier << "\n";
        metaOut << "rlHandoverHangoverLength: " << rlHandoverHangoverLength << "\n";
        metaOut << "rlHandoverRateWindowMs: " << rlHandoverRateWindowMs << "\n";
        metaOut << "rlRewardRefMbps: " << rlRewardRefMbps << "\n";
        metaOut << "rlRewardGoodputShape: " << rlRewardGoodputShape << "\n";
        metaOut << "rlRewardGoodputAlpha: " << rlRewardGoodputAlpha << "\n";
        metaOut << "rlRewardGoodputP: " << rlRewardGoodputP << "\n";
        metaOut << "outputDir: " << g_outputDir << "\n";
    }

    // ---- NR Helper Setup ---------------------------------------------------- //
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    g_nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    g_nrChannelHelper = CreateObject<NrChannelHelper>();
    g_nrHelper = CreateObject<NrHelper>();
    g_nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    g_nrHelper->SetEpcHelper(g_nrEpcHelper);

    // --- Spectrum: one band @ 3.5 GHz, bandwidthMhz MHz (default 10), 1 CC, 1 BWP --- //
    // Subcarrier spacing from CLI --numerology (0 = 15 kHz, 1 = 30 kHz).
    // 3.5 GHz matches n78 (FR1); 100 MHz is the full n78 deployment, smaller
    // bandwidths iterate faster. Numerology affects the PRB count per MHz
    // (15 kHz: 52 PRB @ 10 MHz; 30 kHz: 24 PRB @ 10 MHz) and slot duration.
    const double centralFrequency = 3.5e9; // 3.5 GHz (FR1)
    const double bandwidth = bandwidthMhz * 1e6; // e.g. 20 MHz (106 RBs) or 10 MHz (52 RBs)
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // --- Channel: 3GPP TR 38.901 UMa (Urban Macro), default LOS condition --- //
    // CLI --channelUpdateMs: 0 disables the spatial-consistency / LOS-NLOS
    // evolution (module default = frozen realization). At 20 ms the per-update
    // displacement is 0.4 m at 20 m/s (~5 lambda at 3.5 GHz); at 50 ms it hits
    // 1 m — the model's consistency limit — and the frozen staircase aliases
    // the ~2 m-scale multipath fading into a two-level square-wave SINR.
    // CLI --channelModel=tworay swaps in
    // TwoRaySpectrumPropagationLossModel: it drops the 3GPP CHANNEL MATRIX
    // machinery (GenSpectrumChannelMatrix + UpdatePeriod spatial consistency +
    // MIMO spatial correlation — the dominant radio cost) but KEEPS the
    // phased-array beamforming gain (array response x BF vector, NLOS penalty
    // x1/19) and FTR small-scale fading + LOS corrections — what goes away is
    // the matrix, not beamforming/fading. The physics differ (UMa condition
    // model, FTR fading) — benchmark-only unless deliberately chosen.
    if (channelModel == "tworay")
    {
        // The two-ray model accepts the UMa-AV scenario (FTR fading aliases
        // the UMa calibration; condition + propagation stay UMa-AV via the helper).
        g_nrChannelHelper->ConfigureFactories("UMa-AV", "Default", "TwoRay");
    }
    else
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdateMs)));
        // Moving-scatterer Doppler (TR 37.885 Sec. 6.2.3): random per-cluster
        // speed in [-vScatt, vScatt] on the reflected paths. 20 m/s ~ urban
        // ground-traffic speeds; doubles the UT Doppler at 20 m/s.
        Config::SetDefault("ns3::ThreeGppChannelModel::vScatt", DoubleValue(20.0));
        g_nrChannelHelper->ConfigureFactories("UMa-AV", "Default", "ThreeGpp");
        g_nrChannelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                             TimeValue(MilliSeconds(channelUpdateMs)));
        g_nrChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    }
    g_nrChannelHelper->AssignChannelsToBands({band});

    // --- Scheduler, error model, beamforming --- //
    // 5G NR DL is OFDMA by design; a TDMA scheduler (one UE per slot, whole
    // band) under-utilizes the band and inflates full-band interference from
    // neighbor cells. OFDMA lets multiple UEs share each slot on disjoint RBGs.
    g_nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerOfdmaPF"));
    // PHY error model (CLI --errorModel): NR EESM HARQ-CC/IR x MCS Table 1/2
    // (Table 2 = 256-QAM up to MCS 27, aggressive; Table 1 = 64-QAM, robust)
    // or LTE-MI (NrLteMiErrorModel, the module default). Default eesm-ir-t1
    // (MCS Table 1 / 64-QAM), robust under the volatile UAV channel.
    std::string errorModelType = "ns3::NrEesmIrT2";
    if (errorModel == "eesm-ir-t1")
        errorModelType = "ns3::NrEesmIrT1";
    else if (errorModel == "eesm-cc-t2")
        errorModelType = "ns3::NrEesmCcT2";
    else if (errorModel == "eesm-cc-t1")
        errorModelType = "ns3::NrEesmCcT1";
    else if (errorModel == "lte-mi")
        errorModelType = "ns3::NrLteMiErrorModel";
    g_nrHelper->SetDlErrorModel(errorModelType);
    g_nrHelper->SetUlErrorModel(errorModelType);
    idealBeamformingHelper->SetAttribute(
        "BeamformingMethod",
        TypeIdValue(TypeId::LookupByName("ns3::DirectPathQuasiOmniBeamforming")));

    // --- Handover algorithm ---
    // The RL observation cadence is driven by the UE PHY's ReportUeMeasurements
    // trace, so the L1 filter period is aligned with stepTime for stepTime to
    // genuinely control the RL decision cadence. A3 handover decisions also use
    // these reports, so a larger stepTime coarsens the A3 baseline cadence too.
    Config::SetDefault("ns3::NrUePhy::UeMeasurementsFilterPeriod",
                       TimeValue(MilliSeconds(stepTime)));
    if (handoverAlgorithm == "agent" || rlMode)
    {
        g_nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");
        Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(false));
    }
    else if (handoverAlgorithm == "a3")
    {
        g_nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
        double a3Hysteresis = 3.0;  // hardcoded A3 hysteresis
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
    // Fixed TDD pattern instead of the all-Flexible default, which crashes
    // ("Cannot TX while RX") when UL grants and DL data are scheduled for the
    // same UE in Flexible slots. msg="gNB transmission overlaps in time with UE transmission. CellId:5", +16.401071429s 9 file=contrib/nr/model/nr-spectrum-phy.cc, line=1239
    Config::SetDefault("ns3::NrGnbPhy::Pattern", StringValue("DL|S|UL|UL|DL|DL|S|UL|UL|DL|"));

    // --- PHY configuration --- //
    g_nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    g_nrHelper->SetGnbPhyAttribute("TxPower",
                                   DoubleValue(23.0));
    g_nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));


    // --- RLC buffer size --- //
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(rlcTxBufferBytes));
    Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(rlcTxBufferBytes)); // parity for TCP (AM)

    // Reduce TCP MinRTO from RFC 6298 default (1s) to Linux standard (200ms)
    // to recover faster from handover-induced packet loss.
    Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue(MilliSeconds(200)));

    // TCP socket buffers sized above the BDP (~0.4 MB at the ~80 ms
    // saturation RTT and the 50 Mbps link); the 128 KB ns-3 default would
    // cap the transport at ~13 Mbps, making it buffer-limited instead of
    // radio-limited.
    if (transportProtocol == "tcp")
    {
        Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(1 << 20));
        Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(1 << 20));
    }

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
        // Trajectory obs support: capture the pre-generated UAV waypoint plan
        // so the obs-app can lerp exact positions from it (g_uavWaypoints).
        // WaypointMobilityModel moves at constant velocity between waypoints,
        // so the plan IS the trajectory.
        g_uavWaypoints.clear();

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

        InstallUavMobility(g_uavContainer.Get(0),
                           bbox,
                           startHeight,
                           endHeight,
                           ueSpeed,
                           simDuration,
                           travelLegMin,
                           travelLegMax,
                           /*randomStart=*/true,
                           Vector((bbox.minX + bbox.maxX) / 2.0, uavCenterY, startHeight),
                           uavMobility);
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

        InstallUavMobility(g_uavContainer.Get(0),
                           bbox,
                           startHeight,
                           endHeight,
                           ueSpeed,
                           simDuration,
                           travelLegMin,
                           travelLegMax,
                           /*randomStart=*/true,
                           Vector((bbox.minX + bbox.maxX) / 2.0, uavCenterY, startHeight),
                           uavMobility);
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

        BoundingBox bbox = ComputeGnbBoundingBox(g_gnbContainer, intersiteDistance * 0.2);

        InstallUavMobility(g_uavContainer.Get(0),
                           bbox,
                           startHeight,
                           endHeight,
                           ueSpeed,
                           simDuration,
                           travelLegMin,
                           travelLegMax,
                           /*randomStart=*/false,
                           Vector(startX, uavY, startHeight),
                           uavMobility);
    }


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
            (transportProtocol == "tcp") ? NrGnbRrc::RLC_AM_ALWAYS : NrGnbRrc::RLC_UM_ALWAYS;
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
        if (channelModel == "tworay")
        {
            // AttachToMaxRsrpGnb requires the ThreeGpp phased-array channel
            // (NrInitialAssociation::ExtractUeParameters derefs it); the
            // two-ray model has none — attach by distance instead.
            g_nrHelper->AttachToClosestGnb(g_uavNrDevs, g_gnbNrDevs);
        }
        else
        {
            g_nrHelper->AttachToMaxRsrpGnb(g_uavNrDevs, g_gnbNrDevs);
        }
    }
    else
    {
        g_nrHelper->AttachToGnb(g_uavNrDevs.Get(0), g_gnbNrDevs.Get(0));
    }

    // The attach helper runs at t=0 (scheduled, not synchronous), so the
    // UAV's serving cell is only known once the simulation starts. Seed the
    // tracked cell at 1.1 s (before UL traffic at ~1.2 s) so the
    // serving-cell filters (ul_sinr_srs.csv, nr-rl-slot-stats.csv) log from
    // the first PUSCH instead of waiting for the first handover.
    Simulator::Schedule(Seconds(1.1), []() {
        g_currentCellId = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>()->GetCellId();
    });

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

        // Pin the interferer RNGs so their trajectories depend only on
        // (seed, runId, UE index). Unpinned RNGs draw from the shared
        // auto-assigned stream, so upstream consumption-order shifts
        // (e.g. rlMode on/off between A3 baselines and RL evals) would silently
        // move the interferers. Stream 110 = start positions (prefix-stable:
        // adding interferers extends the draw sequence without moving UE 0..N-1);
        // 111+2i / 112+2i = per-UE waypoint RNGs (each UE's plan independent of
        // the total interferer count).
        Ptr<UniformRandomVariable> interferingUeRng = CreateObject<UniformRandomVariable>();
        interferingUeRng->SetAttribute("Min", DoubleValue(0.0));
        interferingUeRng->SetAttribute("Max", DoubleValue(1.0));
        interferingUeRng->SetStream(110);

        for (uint32_t i = 0; i < addInterferingUes; ++i)
        {
            double x = bbox.minX + interferingUeRng->GetValue() * (bbox.maxX - bbox.minX);
            double y = bbox.minY + interferingUeRng->GetValue() * (bbox.maxY - bbox.minY);
            double ueHeight = (interferingUeRng->GetValue() < aerialUeRatio)
                                  ? (50.0 + interferingUeRng->GetValue() * 250.0)
                                  : 1.5;

            // Aerial interfering UE: random waypoint flight only in
            // interfererMobility="waypoint"; the default "static" hovers the
            // interferers at their start positions (deterministic spatial
            // interference field — a function of the UAV's position, which is
            // in the obs). Waypoint streams 111+2i/112+2i are drawn only in
            // waypoint mode; start positions always use stream 110.
            if (ueHeight > 1.5 && interfererMobility == "waypoint")
            {
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
                wpRng->SetStream(111 + 2 * i);
                wpRngY->SetStream(112 + 2 * i);
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
                // Ground UE or static aerial UE: ConstantPosition at the
                // starting location (aerial height for static aerial).
                MobilityHelper groundMob;
                groundMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
                groundMob.Install(g_interferingUeContainer.Get(i));
                g_interferingUeContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(
                    Vector(x, y, ueHeight));
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

    // NR uses periodic RRC measurement reports (UE PHY ReportUeMeasurements).

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

    // OnOff data rate (Mbps) — the traffic cap (CLI --trafficRateMbps). Single
    // source of truth for the OnOff applications (both TCP and UDP branches).
    // Above the link capacity this acts as full-buffer (goodput = link); below
    // it, the rate becomes the ceiling and idle slots appear (cheaper runtime).
    const double onOffDataRateMbps = trafficRateMbps;
    const double onOffDataRateBps = onOffDataRateMbps * 1e6;

    // Reward/obs goodput reference (Mbps) — SEPARATE from the traffic rate.
    // normGoodput = goodput / ref. A reference far above the achieved goodput
    // pins normG in the flat low region of the Deng inverse
    // (R_G = 1/(1+beta*(1-normG))), destroying reward differentiation across cells.
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
            // std::cout << "DEBUG t=" << Simulator::Now().GetSeconds()
            //           << " rrcState=" << rrc->GetState() << " cellId=" << rrc->GetCellId()
            //           << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal() << "\n";
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

            // The app starts at 1.0 s and the handshake usually completes before
            // the State trace attaches here, so ESTABLISHED is never observed and
            // the RTT guard would discard every sample. Mark the connection
            // established explicitly; the State trace stays as fallback.
            g_tcpConnected = true;
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
                      << " ueIp=" << ipv4->GetAddress(1, 0).GetLocal() << "\n";
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
                                            MakeBoundCallback(&CaptureUlSrsSinr, gnbCellId));
            phy->TraceConnectWithoutContext("UlSinrTrace",
                                            MakeBoundCallback(&UlSrsSinrLogger, gnbCellId));
            phy->TraceConnectWithoutContext("SlotDataStats", MakeCallback(&CaptureSlotDataStats));
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
            // DL per-TB RX trace (RxPacketTraceUe) — DL mirror of
            // nr-rl-ul-rx-sinr.csv
            Ptr<NrSpectrumPhy> ueSpectrumPhy = uePhy->GetSpectrumPhy();
            if (ueSpectrumPhy)
            {
                ueSpectrumPhy->TraceConnectWithoutContext("RxPacketTraceUe",
                                                          MakeCallback(&DlRxPacketLogger));
            }
        }
    });

    Config::ConnectWithoutContext("/NodeList/" + std::to_string(uavNodeId) +
                                      "/DeviceList/*/NrUeRrc/StateTransition",
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
        g_ulSinrSum.assign(numBs + 1, 0.0);
        g_ulSinrCount.assign(numBs + 1, 0);
        g_lastRsrqValues.resize(numBs + 1, -20.0);  // index by cellId (1-based), dB

        RlApplicationHelper rlAppHelper(NrRlHandoverRewardApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("StopTime", TimeValue(Seconds(simDuration)));
        rlAppHelper.SetAttribute("AlphaGoodput", DoubleValue(rlAlphaGoodput));
        rlAppHelper.SetAttribute("BetaGoodput", DoubleValue(rlBetaGoodput));
        rlAppHelper.SetAttribute("BetaHandover", DoubleValue(rlBetaHandover));
        rlAppHelper.SetAttribute("PingPongBetaMultiplier", DoubleValue(rlPingPongMultiplier));
        rlAppHelper.SetAttribute("HandoverHangoverLength", UintegerValue(rlHandoverHangoverLength));
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
        auto agentApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverObservationApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("UavNodeId", UintegerValue(uavNodeId));
        rlAppHelper.SetAttribute("StepTimeMs", UintegerValue(stepTime));
        rlAppHelper.SetAttribute("GoodputRefBps", DoubleValue(rewardRefBps));
        rlAppHelper.SetAttribute("HandoverRateWindowMs", UintegerValue(rlHandoverRateWindowMs));
        auto obsApps = rlAppHelper.Install(g_uavContainer.Get(0));

        rlAppHelper.SetTypeId(NrRlHandoverActionApp::GetTypeId());
        rlAppHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
        rlAppHelper.SetAttribute("NumBs", UintegerValue(numBs));
        rlAppHelper.SetAttribute("HandoverAlgorithm", StringValue(handoverAlgorithm));
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
