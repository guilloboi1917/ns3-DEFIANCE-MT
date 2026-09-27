#include "nr-rl-handover-obs-app.h"

#include "ns3/base-test.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/node-list.h"
#include "ns3/nr-module.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

using namespace ns3;

// External globals from the scenario
extern std::string g_flowDirection;
extern NetDeviceContainer g_uavNrDevs;
extern NetDeviceContainer g_gnbNrDevs;
extern std::vector<double> g_lastRsrpValues;
extern std::vector<double> g_lastSinrValues;
extern std::vector<double> g_ulSinrSum;    ///< Per-cell step accumulators (UL SINR)
extern std::vector<uint32_t> g_ulSinrCount; ///< Per-cell step sample counts (UL SINR)
extern std::vector<double> g_lastRsrqValues;
extern uint32_t g_topNCells[4];
extern uint32_t g_receiverNodeId;
extern bool g_logging;
extern std::string g_outputDir;
extern std::vector<Waypoint> g_uavWaypoints; ///< Pre-generated UAV waypoint plan (time, pos)
extern double g_simDuration;                ///< Simulation duration (s) — lookahead clamp
extern double intersiteDistance;             ///< gNB site spacing (m) — position normalization reference
extern double g_ulServingRbUtil;             ///< Serving-cell UL RB utilization fraction [0,1]
extern uint32_t g_ulServingSchedUe;          ///< Serving-cell scheduled-UE count (last slot)

namespace ns3
{

// Trajectory lookahead horizon for the pos2s observation block.
static constexpr double kLookaheadS = 2.0;

NS_LOG_COMPONENT_DEFINE("NrRlHandoverObservationApp");

// ---------------------------------------------------------------------------
// Clamp helper
// ---------------------------------------------------------------------------
namespace
{
double
Clamp(double val, double lo, double hi)
{
    return val < lo ? lo : (val > hi ? hi : val);
}
} // anonymous namespace

// ---------------------------------------------------------------------------
// Trajectory helpers: exact position from the pre-generated waypoint plan.
// WaypointMobilityModel moves at constant velocity between waypoints
// (waypoint-mobility-model.cc:158-168), so piecewise-linear interpolation is
// bit-identical to the sim's own position.
// ---------------------------------------------------------------------------

/**
 * Interpolated UAV position at time t (seconds) from the waypoint plan.
 *
 * @param t Simulation time (s).
 * @return Position at t; falls back to the nearest endpoint outside the plan.
 */
Vector
GetUavPositionAt(double t)
{
    if (g_uavWaypoints.empty())
    {
        return Vector(0.0, 0.0, 0.0);
    }
    const auto& wps = g_uavWaypoints;
    // Find the first waypoint with time > t; the segment is [i-1, i).
    auto it = std::upper_bound(wps.begin(),
                               wps.end(),
                               t,
                               [](double val, const Waypoint& w)
                               { return val < w.time.GetSeconds(); });
    if (it == wps.begin())
    {
        return wps.front().position;
    }
    if (it == wps.end())
    {
        return wps.back().position;
    }
    const Waypoint& a = *(it - 1);
    const Waypoint& b = *it;
    double ta = a.time.GetSeconds();
    double tb = b.time.GetSeconds();
    // Linear interpolation
    double frac = (tb > ta) ? (t - ta) / (tb - ta) : 0.0;
    return Vector(a.position.x + frac * (b.position.x - a.position.x),
                  a.position.y + frac * (b.position.y - a.position.y),
                  a.position.z + frac * (b.position.z - a.position.z));
}

/**
 * Next distinct-position waypoint and the time-to-arrival at it.
 *
 * Dwell waypoints (same position, later time) are skipped so the "next waypoint"
 * is always a leg target; TTA includes the remaining dwell.
 *
 * @param t Simulation time (s).
 * @param[out] nextPos Position of the next leg target.
 * @return Seconds until arrival; 0 if the plan is exhausted.
 */
double
GetNextWaypointAt(double t, Vector& nextPos)
{
    Vector cur = GetUavPositionAt(t);
    nextPos = cur;
    for (const auto& w : g_uavWaypoints)
    {
        double wt = w.time.GetSeconds();
        if (wt > t + 1e-9 &&
            (std::abs(w.position.x - cur.x) > 1e-6 || std::abs(w.position.y - cur.y) > 1e-6 ||
             std::abs(w.position.z - cur.z) > 1e-6))
        {
            nextPos = w.position;
            return wt - t;
        }
    }
    return 0.0;
}

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------
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
                          "Informational: expected ReportUeMeasurements cadence (ms).",
                          UintegerValue(200),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_stepTimeMs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("GoodputRefBps",
                          "Fixed upper-bound reference (bps) for the normalized "
                          "goodput observation dimension. Set to the OnOff data rate.",
                          DoubleValue(40e6),
                          MakeDoubleAccessor(&NrRlHandoverObservationApp::m_goodputRefBps),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("HandoverRateWindowMs",
                          "Sliding window (ms) for the ho_count_10s observation "
                          "dimension (obs index 29): handovers in the window as "
                          "a handover-activity signal.",
                          UintegerValue(10000),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_hoRateWindowMs),
                          MakeUintegerChecker<uint32_t>(100, 600000));
    return tid;
}

void
NrRlHandoverObservationApp::DoInitialize()
{
    ObservationApplication::DoInitialize();

    m_rsrpValues = std::vector<double>(m_numBs, -140.0);
    m_rsrqValues = std::vector<double>(m_numBs, -20.0);
    m_currentCellId = 0;
    m_servingRsrp = -140.0;
    m_servingRsrq = -20.0;
    m_sinrSum = 0.0;
    m_sinrCount = 0;
    m_lastHandoverTime = Seconds(0);
    m_lastReportTime = Seconds(0);
    m_velocityX = 0.0;
    m_velocityY = 0.0;
    m_velocityZ = 0.0;
    m_sinkBytesReceived = 0;

    // Initialise global Top-N array
    for (uint32_t i = 0; i < 4; i++)
    {
        g_topNCells[i] = 0;
    }
}

// ---------------------------------------------------------------------------
// RegisterCallbacks
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::RegisterCallbacks()
{
    uint32_t nodeId = GetNode()->GetId();

    // --- Per-cell RSRP/RSRQ (always on UAV PHY) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "ReportUeMeasurements",
        MakeCallback(&NrRlHandoverObservationApp::ObserveUeRsrpRsrq, this));

    // --- DL data SINR (always on UE PHY, post-beamforming) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "DlDataSinr",
        MakeCallback(&NrRlHandoverObservationApp::ObserveDlSinr, this));

    // --- HandoverEndOk (for time_since_ho) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/NrUeRrc/HandoverEndOk",
        MakeCallback(&NrRlHandoverObservationApp::ObserveHandover, this));

    // --- Mobility CourseChange (for heading/speed) ---
    Config::Connect("/NodeList/" + std::to_string(m_uavNodeId) + "/$ns3::MobilityModel/CourseChange",
                    MakeCallback(&NrRlHandoverObservationApp::ObserveCourseChange, this));

    // --- Sink Rx on the receiving node (for normalized goodput) ---
    // DL: UAV is the receiver; UL: remoteHost is the receiver. The wildcard
    // matches PacketSink (UDP/TCP) or QuicServer (QUIC), the only app with an
    // Rx trace on the receiver node.
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(g_receiverNodeId) + "/ApplicationList/*/Rx",
        MakeCallback(&NrRlHandoverObservationApp::ObserveSinkRx, this));

    NS_LOG_INFO("NrRlHandoverObservationApp callbacks registered on node "
                << nodeId << " flowDirection=" << g_flowDirection << " topN=" << kTopN);
}

// ---------------------------------------------------------------------------
// Callback: RSRP/RSRQ from ReportUeMeasurements
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveUeRsrpRsrq(uint16_t rnti,
                                              uint16_t cellId,
                                              double rsrp,
                                              double rsrq,
                                              bool isServingCell,
                                              uint8_t componentCarrierId)
{
    if (m_rsrpValues.empty())
    {
        return;
    }

    // Filter for our primary UAV UE
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
    if (rnti != ueRrc->GetRnti())
    {
        return;
    }

    // Store RSRP/RSRQ for this cell
    if (cellId > 0 && cellId <= m_numBs)
    {
        m_rsrpValues[cellId - 1] = rsrp;
        m_rsrqValues[cellId - 1] = rsrq;

        if (isServingCell)
        {
            m_currentCellId = cellId;
            m_servingRsrp = rsrp;
            m_servingRsrq = rsrq;
        }

        // Update global arrays (for logging and ActApp compatibility)
        if (cellId < g_lastRsrpValues.size())
        {
            g_lastRsrpValues[cellId] = rsrp;
        }
        if (cellId < g_lastRsrqValues.size())
        {
            g_lastRsrqValues[cellId] = rsrq;
        }
    }

    // Detect new ReportUeMeasurements cycle: all callbacks in one cycle fire
    // at the same sim time. When the sim time advances, the previous cycle's
    // data is complete — send the observation.
    Time now = Simulator::Now();
    if (now > m_lastReportTime && m_lastReportTime > Seconds(0))
    {
        SendObservation();
    }
    m_lastReportTime = now;
}

// ---------------------------------------------------------------------------
// Callback: DL SINR from UE PHY
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveDlSinr(uint16_t cellId,
                                          uint16_t rnti,
                                          double sinrLinear,
                                          uint16_t bwpId)
{
    if (m_rsrpValues.empty())
    {
        return;
    }

    // Only track serving cell SINR
    if (cellId != m_currentCellId)
    {
        return;
    }

    double sinrDb = (sinrLinear > 0.0) ? (10.0 * std::log10(sinrLinear)) : -40.0;

    // Step accumulation: the obs emits the MEAN of the per-slot DL data SINR
    // over the step (a step mean is stable and aligns with the step-windowed
    // goodput reward).
    m_sinrSum += sinrDb;
    m_sinrCount++;

    // Update global array for logging compatibility
    if (cellId < g_lastSinrValues.size())
    {
        g_lastSinrValues[cellId] = sinrDb;
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Callback: HandoverEndOk
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveHandover(const uint64_t imsi,
                                            const uint16_t cellId,
                                            const uint16_t rnti)
{
    m_lastHandoverTime = Simulator::Now();

    // Record the handover in the sliding window for the ho_count_10s feature.
    double now = Simulator::Now().GetSeconds();
    double windowSec = m_hoRateWindowMs / 1000.0;
    m_hoTimes.push_back(now);
    while (!m_hoTimes.empty() && m_hoTimes.front() < now - windowSec)
    {
        m_hoTimes.pop_front();
    }
}

// ---------------------------------------------------------------------------
// Callback: CourseChange (mobility)
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveCourseChange(std::string context,
                                                  Ptr<const MobilityModel> model)
{
    Vector velocity = model->GetVelocity();
    m_velocityX = velocity.x;
    m_velocityY = velocity.y;
    m_velocityZ = velocity.z;
}

// ---------------------------------------------------------------------------
// Callback: PacketSink Rx (for normalized goodput)
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveSinkRx(Ptr<const Packet> packet, const Address& from)
{
    m_sinkBytesReceived += packet->GetSize();
}

// ---------------------------------------------------------------------------
// ComputeTopNCells — rank cells by RSRP, populate g_topNCells[1..3]
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ComputeTopNCells()
{
    // Rank all NON-serving cells above the noise floor: every action 1..N is a
    // genuine handover candidate, and the margin features
    // (serving_rsrp - slot_rsrp[k]) become the A3-style margin.
    struct Candidate
    {
        uint32_t cellId;
        double rsrp;
    };

    // Re-sync the serving cell with the live RRC state: the observation comes
    // from the previous measurement cycle, so m_currentCellId may still name the
    // cell the UE just left, which would make it requestable again.
    if (g_uavNrDevs.GetN() > 0)
    {
        auto ueDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
        if (ueDev && ueDev->GetRrc())
        {
            uint32_t liveCellId = ueDev->GetRrc()->GetCellId();
            if (liveCellId > 0 && liveCellId <= m_numBs && liveCellId != m_currentCellId)
            {
                m_currentCellId = liveCellId;
                if (!std::isnan(m_rsrpValues[liveCellId - 1]))
                {
                    m_servingRsrp = m_rsrpValues[liveCellId - 1];
                }
                if (!std::isnan(m_rsrqValues[liveCellId - 1]))
                {
                    m_servingRsrq = m_rsrqValues[liveCellId - 1];
                }
            }
        }
    }

    std::vector<Candidate> candidates;
    candidates.reserve(m_numBs);

    constexpr double noiseFloorRsrp = -135.0; // dBm, might be too low

    for (uint32_t i = 0; i < m_numBs; i++)
    {
        uint32_t cellId = i + 1;
        if (m_currentCellId > 0 && cellId == m_currentCellId)
        {
            continue; // serving cell is not a handover candidate
        }
        double rsrp = m_rsrpValues[i];
        if (std::isnan(rsrp) || rsrp <= noiseFloorRsrp)
        {
            continue;
        }
        candidates.push_back({cellId, rsrp});
    }

    // Sort descending by RSRP
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.rsrp > b.rsrp;
    });

    // Populate g_topNCells[0..kTopN] (Top-N fixed at 3: the obs ranks exactly
    // the cells the action space can hand over to).
    g_topNCells[0] = 0; // no-op sentinel
    for (uint32_t k = 0; k < kTopN; k++)
    {
        if (k < candidates.size())
        {
            g_topNCells[k + 1] = candidates[k].cellId;
        }
        else
        {
            g_topNCells[k + 1] = 0; // padding: fewer than N valid candidates
        }
    }
}

// ---------------------------------------------------------------------------
// BuildObservation — flat Box vector (32 dims)
// ---------------------------------------------------------------------------
Ptr<OpenGymDictContainer>
NrRlHandoverObservationApp::BuildObservation()
{
    // Compute Top-N ranking before building observation
    ComputeTopNCells();

    // Flat 32-dim Box; the declared shape must match the AddValue count below.
    //
    // Layout:
    //   [0-1]   serving_rsrp / serving_rsrq
    //   [2-4]   slot_rsrp[0..2]     (top-3 ranked NON-serving cells)
    //   [5-7]   rsrp_delta[0..2]    (serving_rsrp - slot_rsrp)
    //   [8]     dl_sinr            (mean over the 200 ms step of the DL data SINR)
    //   [9]     time_since_ho
    //   [10]    norm_goodput
    //   [11]    ho_count_10s
    //   [12-14] ul_sinr / ul_rb_util / ul_sched_ue
    //   [15-19] time-delta block: d_serving_rsrp, d_serving_sinr,
    //            d_serving_rsrq, d_norm_goodput, d_margin
    //   [20-22] d_slot_rsrp[0..2]
    //   [23-25] heading_x / heading_y / heading_z
    //   [26-28] pos_x / pos_y / pos_z       (current position / ISD)
    //   [29-31] pos2s_x / pos2s_y / pos2s_z (position 2 s ahead / ISD)
    auto box = MakeBoxContainer<double>(32);

    // [0] serving_rsrp
    double servingRsrp = std::isnan(m_servingRsrp) ? -140.0 : m_servingRsrp;
    box->AddValue(Clamp(servingRsrp, -160.0, -40.0));

    // [1] serving_rsrq
    double servingRsrq = std::isnan(m_servingRsrq) ? -20.0 : m_servingRsrq;
    box->AddValue(Clamp(servingRsrq, -100.0, -3.0));

    // RSRQ was removed from the ranked slots: RSRQ = RSRP + cell-independent
    // RSSI offset (nr-ue-phy.cc), so slot_rsrq and rsrq_delta were fully
    // determined by the RSRP fields — 10 redundant dims.
    // [8] dl_sinr — mean over the 200 ms step of the per-slot DL data SINR
    // (UE PHY); -40 when no DL data flowed during the step.
    double sinrVal = (m_sinrCount > 0) ? (m_sinrSum / m_sinrCount) : -40.0;
    m_sinrSum = 0.0;
    m_sinrCount = 0;

    // Windowed normalized goodput (emitted at [10]) — computed early so the
    // time-delta block can use it.
    double elapsed = (Simulator::Now() - m_lastObservationTime).GetSeconds();
    double normGoodput = 0.0;
    if (elapsed > 0.0 && m_goodputRefBps > 0.0)
    {
        double goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / elapsed;
        normGoodput = goodputBps / m_goodputRefBps;
    }
    m_sinkBytesReceived = 0;

    // Ranked-slot RSRPs (top-3 cells per g_topNCells[1..3]; same Top-N as the
    // action space — the obs ranks exactly the cells the agent can hand over to).
    std::vector<double> slotRsrp(kTopN, -140.0);
    for (uint32_t k = 0; k < kTopN; k++)
    {
        uint32_t cellId = g_topNCells[k + 1];
        slotRsrp[k] = (cellId > 0 && cellId <= m_numBs)
                          ? (std::isnan(m_rsrpValues[cellId - 1]) ? -140.0 : m_rsrpValues[cellId - 1])
                          : -140.0;
    }

    // [2..4] slot_rsrp[0..2] — RSRP of the top-3 ranked cells
    for (uint32_t k = 0; k < kTopN; k++)
    {
        box->AddValue(Clamp(slotRsrp[k], -160.0, -40.0));
    }
    // [5..7] rsrp_delta[0..2] — serving_rsrp - slot_rsrp (positive = serving better)
    for (uint32_t k = 0; k < kTopN; k++)
    {
        box->AddValue(Clamp(servingRsrp - slotRsrp[k], -60.0, 60.0));
    }

    // [8] dl_sinr — step mean of the per-slot DL data SINR
    box->AddValue(Clamp(sinrVal, -40.0, 50.0));

    // [9] time_since_ho — seconds since last handover, clamped to 10s
    double timeSinceHo = (m_lastHandoverTime > Seconds(0))
                             ? (Simulator::Now() - m_lastHandoverTime).GetSeconds()
                             : 10.0;
    box->AddValue(Clamp(timeSinceHo, 0.0, 10.0));

    // [10] norm_goodput — windowed normalized goodput (0 = no data flowing).
    box->AddValue(Clamp(normGoodput, 0.0, 2.0));

    // [11] ho_count_10s — handovers within the sliding window (clamped to 10).
    double nowS = Simulator::Now().GetSeconds();
    double windowSec = m_hoRateWindowMs / 1000.0;
    while (!m_hoTimes.empty() && m_hoTimes.front() < nowS - windowSec)
    {
        m_hoTimes.pop_front();
    }
    box->AddValue(Clamp(static_cast<double>(m_hoTimes.size()), 0.0, 10.0));

    // [12..14] UL block (always present; sentinels when no UL activity — DL-only
    // flows schedule nothing UL): serving-cell UL SINR (gNB, RB-averaged per
    // slot, step mean), UL RB utilization, scheduled-UE count (contention).
    // Enables UL-aware policies and the interference regime (RB contention at
    // good SINR).
    double ulSinrVal = -40.0;
    if (m_currentCellId > 0 && m_currentCellId < g_ulSinrCount.size() &&
        g_ulSinrCount[m_currentCellId] > 0)
    {
        ulSinrVal = g_ulSinrSum[m_currentCellId] / g_ulSinrCount[m_currentCellId];
    }
    for (uint32_t c = 1; c < g_ulSinrCount.size(); c++)
    {
        g_ulSinrSum[c] = 0.0;
        g_ulSinrCount[c] = 0;
    }
    box->AddValue(Clamp(ulSinrVal, -40.0, 50.0)); // [12] ul_sinr (dB)
    box->AddValue(Clamp(g_ulServingRbUtil, 0.0, 1.0));        // [13] ul_rb_util
    box->AddValue(Clamp(g_ulServingSchedUe / 16.0, 0.0, 1.0)); // [14] ul_sched_ue

    // Time-delta block [15..22]: 1 s trends, computed per PHYSICAL cell
    // (cellId-keyed ring buffer) so rank re-orderings do not alias the trend —
    // each ranked slot's delta compares that cell against its own value 1 s
    // ago. Deltas are 0 until the window fills.
    double margin = servingRsrp - slotRsrp[0];
    double dServRsrp = 0.0;
    double dServSinr = 0.0;
    double dServRsrq = 0.0;
    double dNormG = 0.0;
    double dMargin = 0.0;
    std::vector<double> dSlot(kTopN, 0.0);
    if (m_obsHistoryCount == OBS_WINDOW_STEPS)
    {
        const ObsSnapshot& old = m_obsHistory[m_obsHistoryIndex];
        // Serving cell: true trend of the current serving cell (handover-safe —
        // a new serving cell is compared against its own 1 s-old RSRP).
        if (m_currentCellId > 0 && m_currentCellId <= m_numBs &&
            old.cellRsrp.size() == m_numBs)
        {
            dServRsrp = servingRsrp - old.cellRsrp[m_currentCellId - 1];
        }
        dServSinr = sinrVal - old.servingSinr;
        dServRsrq = servingRsrq - old.servingRsrq;
        dNormG = normGoodput - old.normGoodput;
        dMargin = margin - old.margin;
        for (uint32_t k = 0; k < kTopN; k++)
        {
            uint32_t cellId = g_topNCells[k + 1];
            if (cellId > 0 && cellId <= m_numBs && old.cellRsrp.size() == m_numBs)
            {
                dSlot[k] = slotRsrp[k] - old.cellRsrp[cellId - 1];
            }
        }
    }
    box->AddValue(Clamp(dServRsrp, -20.0, 20.0)); // [15]
    box->AddValue(Clamp(dServSinr, -20.0, 20.0)); // [16]
    box->AddValue(Clamp(dServRsrq, -20.0, 20.0)); // [17]
    box->AddValue(Clamp(dNormG, -2.0, 2.0));      // [18]
    box->AddValue(Clamp(dMargin, -20.0, 20.0));   // [19]
    for (uint32_t k = 0; k < kTopN; k++)
    {
        box->AddValue(Clamp(dSlot[k], -20.0, 20.0)); // [20..22]
    }

    // Store this observation's per-cell RSRP snapshot in the ring buffer.
    ObsSnapshot& cur = m_obsHistory[m_obsHistoryIndex];
    cur.servingSinr = sinrVal;
    cur.servingRsrq = servingRsrq;
    cur.normGoodput = normGoodput;
    cur.margin = margin;
    cur.cellRsrp.assign(m_numBs, -140.0);
    for (uint32_t cellId = 1; cellId <= m_numBs; cellId++)
    {
        cur.cellRsrp[cellId - 1] = std::isnan(m_rsrpValues[cellId - 1])
                                       ? -140.0
                                       : m_rsrpValues[cellId - 1];
    }
    m_obsHistoryIndex = (m_obsHistoryIndex + 1) % OBS_WINDOW_STEPS;
    m_obsHistoryCount = std::min(m_obsHistoryCount + 1, OBS_WINDOW_STEPS);

    // [23..25] heading_x, heading_y, heading_z — 3D unit heading (v / speed).
    // The UAV moves on 3D waypoints (altitude alternates), so the vertical
    // heading is meaningful for LoS-based cell selection. speed itself is
    // constant (ueSpeed) and was dropped as a dead feature.
    double speed = std::sqrt(m_velocityX * m_velocityX +
                             m_velocityY * m_velocityY +
                             m_velocityZ * m_velocityZ);
    if (speed > 0.01)
    {
        box->AddValue(Clamp(m_velocityX / speed, -1.0, 1.0));
        box->AddValue(Clamp(m_velocityY / speed, -1.0, 1.0));
        box->AddValue(Clamp(m_velocityZ / speed, -1.0, 1.0));
    }
    else
    {
        box->AddValue(0.0);
        box->AddValue(0.0);
        box->AddValue(0.0);
    }

    // [26..31] Trajectory block (always present): exact UAV position and the
    // position 2 s ahead, normalized by intersiteDistance, lerped from the
    // pre-generated waypoint plan (Markovian state — the future channel is
    // near-deterministic given position; this is the V-credit-assignment fix
    // for the collapse regime). The lookahead is clamped to the episode end.
    if (intersiteDistance > 0.0)
    {
        double nowT = Simulator::Now().GetSeconds();
        double tAhead = std::min(nowT + kLookaheadS, g_simDuration);
        Vector pos = GetUavPositionAt(nowT);
        Vector posAhead = GetUavPositionAt(tAhead);
        double invIsd = 1.0 / intersiteDistance;
        box->AddValue(Clamp(pos.x * invIsd, -3.0, 3.0));      // [26] pos_x / ISD
        box->AddValue(Clamp(pos.y * invIsd, -3.0, 3.0));      // [27] pos_y / ISD
        box->AddValue(Clamp(pos.z * invIsd, -3.0, 3.0));      // [28] pos_z / ISD
        box->AddValue(Clamp(posAhead.x * invIsd, -3.0, 3.0)); // [29] pos2s_x / ISD
        box->AddValue(Clamp(posAhead.y * invIsd, -3.0, 3.0)); // [30] pos2s_y / ISD
        box->AddValue(Clamp(posAhead.z * invIsd, -3.0, 3.0)); // [31] pos2s_z / ISD
    }
    else
    {
        for (uint32_t i = 0; i < 6; i++)
        {
            box->AddValue(0.0);
        }
    }

    // Wrap in Dict for Send() transport compatibility
    auto obs = CreateObject<OpenGymDictContainer>();
    obs->Add("obs", box);
    return obs;
}

// ---------------------------------------------------------------------------
// SendObservation
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::SendObservation()
{
    auto obs = BuildObservation();

    // Log observation to CSV if logging is enabled
    if (g_logging)
    {
        auto box = DynamicCast<OpenGymBoxContainer<double>>(obs->Get("obs"));
        std::ofstream obsFile(g_outputDir + "rl_obs.csv", std::ios_base::app);
        if (box)
        {
            auto shape = box->GetShape();
            uint32_t totalDims = 1;
            for (auto d : shape)
            {
                totalDims *= d;
            }
            obsFile << Simulator::Now().GetSeconds();
            for (uint32_t i = 0; i < totalDims; i++)
            {
                obsFile << "," << box->GetValue(i);
            }
        }
        obsFile << std::endl;
    }

    Send(obs);

    // Mark the observation window boundary (for the next normGoodput window)
    m_lastObservationTime = Simulator::Now();
}

} // namespace ns3
