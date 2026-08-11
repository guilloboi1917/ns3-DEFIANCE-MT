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
extern std::vector<double> g_lastRsrqValues;
extern uint32_t g_topNCells[6];
extern uint32_t g_receiverNodeId;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

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
            .AddAttribute("TopN",
                          "Number of ranked cells for Top-N action space.",
                          UintegerValue(5),
                          MakeUintegerAccessor(&NrRlHandoverObservationApp::m_topN),
                          MakeUintegerChecker<uint32_t>(1, 10))
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
            .AddAttribute("SinrEwmaAlpha",
                          "EWMA smoothing factor for serving cell SINR.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&NrRlHandoverObservationApp::m_sinrEwmaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("GoodputRefBps",
                          "Fixed upper-bound reference (bps) for the normalized "
                          "goodput observation dimension. Set to the OnOff data rate.",
                          DoubleValue(40e6),
                          MakeDoubleAccessor(&NrRlHandoverObservationApp::m_goodputRefBps),
                          MakeDoubleChecker<double>(1.0))
            .AddAttribute("UseTbsObservation",
                          "Emit the real avg TBS at obs index 26. When false, emit a "
                          "constant (0.0) instead — used to test removing the "
                          "goodput-reward-proxy feature without changing the obs layout.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&NrRlHandoverObservationApp::m_useTbsObservation),
                          MakeBooleanChecker())
            .AddAttribute("HandoverRateWindowMs",
                          "Sliding window (ms) for the ho_count_10s observation "
                          "dimension (obs index 29). Should match the reward "
                          "app's HandoverRateWindowMs so the agent can observe "
                          "how close it is to the rate-penalty threshold.",
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
    m_servingSinr = -40.0;
    m_tbsSum = 0;
    m_tbsCount = 0;
    m_lastHandoverTime = Seconds(0);
    m_lastReportTime = Seconds(0);
    m_velocityX = 0.0;
    m_velocityY = 0.0;
    m_velocityZ = 0.0;
    m_sinkBytesReceived = 0;

    // Initialise global Top-N array
    for (uint32_t i = 0; i < 6; i++)
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

    // --- DL TBS (always available on UE PHY) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "ReportDownlinkTbSize",
        MakeCallback(&NrRlHandoverObservationApp::ObserveTbs, this));

    // --- HandoverEndOk (for time_since_ho) ---
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(m_uavNodeId) +
            "/DeviceList/*/NrUeRrc/HandoverEndOk",
        MakeCallback(&NrRlHandoverObservationApp::ObserveHandover, this));

    // --- Mobility CourseChange (for heading/speed) ---
    Config::Connect("/NodeList/" + std::to_string(m_uavNodeId) + "/$ns3::MobilityModel/CourseChange",
                    MakeCallback(&NrRlHandoverObservationApp::ObserveCourseChange, this));

    // --- PacketSink Rx on the receiving node (for normalized goodput) ---
    // DL: UAV is the receiver; UL: remoteHost is the receiver.
    Config::ConnectWithoutContext(
        "/NodeList/" + std::to_string(g_receiverNodeId) +
            "/ApplicationList/*/$ns3::PacketSink/Rx",
        MakeCallback(&NrRlHandoverObservationApp::ObserveSinkRx, this));

    NS_LOG_INFO("NrRlHandoverObservationApp callbacks registered on node "
                << nodeId << " flowDirection=" << g_flowDirection << " topN=" << m_topN);
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

    if (m_servingSinr <= -39.0)
    {
        m_servingSinr = sinrDb;
    }
    else
    {
        m_servingSinr = m_sinrEwmaAlpha * sinrDb + (1.0 - m_sinrEwmaAlpha) * m_servingSinr;
    }

    // Update global array for logging compatibility
    if (cellId < g_lastSinrValues.size())
    {
        g_lastSinrValues[cellId] = m_servingSinr;
    }
}

// ---------------------------------------------------------------------------
// Callback: TBS (DL)
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ObserveTbs(uint64_t imsi, uint64_t tbSize)
{
    m_tbsSum += static_cast<int64_t>(tbSize);
    m_tbsCount++;
}

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
// ComputeTopNCells — rank cells by RSRP, populate g_topNCells[6]
// ---------------------------------------------------------------------------
void
NrRlHandoverObservationApp::ComputeTopNCells()
{
    // Collect (cellId, rsrp) pairs for all NON-serving cells, filtering out
    // cells below the noise floor. The serving cell is EXCLUDED from the
    // ranking (2026-08-07): every action 1..N is then a genuine handover
    // candidate — no blocked-same-cell actions, no Q-inflation from blocked
    // handovers being credited with calm rewards — and the margin features
    // (serving_rsrp - slot_rsrp[k]) become the unambiguous A3-style margin
    // (positive = stay, negative = a better other cell exists).
    struct Candidate
    {
        uint32_t cellId;
        double rsrp;
    };

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

    // Populate g_topNCells[0..m_topN]
    g_topNCells[0] = 0; // no-op sentinel
    for (uint32_t k = 0; k < m_topN; k++)
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
// BuildObservation — flat Box vector (30 dims)
// ---------------------------------------------------------------------------
Ptr<OpenGymDictContainer>
NrRlHandoverObservationApp::BuildObservation()
{
    // Compute Top-N ranking before building observation
    ComputeTopNCells();

    // Build flat Box with 30 doubles (29 + ho_count_10s). The shape MUST match
    // the number of AddValue calls below: the CSV logger and the protobuf both
    // read the declared shape, while the Python env reads the data length — a
    // mismatch silently drops/desyncs the last column (2026-08-10 bug fix).
    auto box = MakeBoxContainer<double>(30);

    // [0] serving_rsrp
    double servingRsrp = std::isnan(m_servingRsrp) ? -140.0 : m_servingRsrp;
    box->AddValue(Clamp(servingRsrp, -160.0, -40.0));

    // [1] serving_rsrq
    double servingRsrq = std::isnan(m_servingRsrq) ? -20.0 : m_servingRsrq;
    box->AddValue(Clamp(servingRsrq, -100.0, -3.0));

    // [2..6] slot_rsrp[0..4] — RSRP of top-5 ranked cells
    // [7..11] rsrp_delta[0..4] — serving_rsrp - slot_rsrp (positive = serving better)
    // [12..21] time-delta block — 1 s trend (5 steps at 0.2 s):
    //   [12] d_serving_rsrp, [13] d_serving_sinr, [14] d_serving_rsrq,
    //   [15] d_norm_goodput, [16] d_margin (d(serving_rsrp - best_slot_rsrp)),
    //   [17..21] d_slot_rsrp[0..4]
    // NOTE (2026-08-05): RSRQ fields were removed. RSRQ = RSRP + UE-RSSI offset
    // (cell-independent RSSI, nr-ue-phy.cc), so slot_rsrq and rsrq_delta were
    // exactly determined by the RSRP fields + serving pair — 10 redundant dims.
    double sinrVal = std::isnan(m_servingSinr) ? -40.0 : m_servingSinr;

    // Windowed normalized goodput (also emitted at [28]) — computed early so
    // the time-delta block can use it.
    double elapsed = (Simulator::Now() - m_lastObservationTime).GetSeconds();
    double normGoodput = 0.0;
    if (elapsed > 0.0 && m_goodputRefBps > 0.0)
    {
        double goodputBps = static_cast<double>(m_sinkBytesReceived) * 8.0 / elapsed;
        normGoodput = goodputBps / m_goodputRefBps;
    }
    m_sinkBytesReceived = 0;

    // Ranked-slot RSRPs (top-5 cells per g_topNCells[1..5]).
    std::vector<double> slotRsrp(m_topN, -140.0);
    for (uint32_t k = 0; k < m_topN; k++)
    {
        uint32_t cellId = g_topNCells[k + 1]; // g_topNCells[1..5]
        slotRsrp[k] = (cellId > 0 && cellId <= m_numBs)
                          ? (std::isnan(m_rsrpValues[cellId - 1]) ? -140.0 : m_rsrpValues[cellId - 1])
                          : -140.0;
    }

    // [2..6] slot_rsrp[0..4] — RSRP of the top-5 ranked cells
    for (uint32_t k = 0; k < m_topN; k++)
    {
        box->AddValue(Clamp(slotRsrp[k], -160.0, -40.0));
    }
    // [7..11] rsrp_delta[0..4] — serving_rsrp - slot_rsrp (positive = serving better)
    for (uint32_t k = 0; k < m_topN; k++)
    {
        box->AddValue(Clamp(servingRsrp - slotRsrp[k], -60.0, 60.0));
    }

    // Time-delta block: 1 s trends, computed per PHYSICAL cell (cellId-keyed
    // ring buffer) so rank re-orderings do not alias the trend — each ranked
    // slot's delta compares that cell against its own value 1 s ago. Deltas are
    // 0 until the window fills.
    double margin = servingRsrp - slotRsrp[0];
    double dServRsrp = 0.0;
    double dServSinr = 0.0;
    double dServRsrq = 0.0;
    double dNormG = 0.0;
    double dMargin = 0.0;
    std::vector<double> dSlot(m_topN, 0.0);
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
        for (uint32_t k = 0; k < m_topN; k++)
        {
            uint32_t cellId = g_topNCells[k + 1];
            if (cellId > 0 && cellId <= m_numBs && old.cellRsrp.size() == m_numBs)
            {
                dSlot[k] = slotRsrp[k] - old.cellRsrp[cellId - 1];
            }
        }
    }
    box->AddValue(Clamp(dServRsrp, -20.0, 20.0)); // [12]
    box->AddValue(Clamp(dServSinr, -20.0, 20.0)); // [13]
    box->AddValue(Clamp(dServRsrq, -20.0, 20.0)); // [14]
    box->AddValue(Clamp(dNormG, -2.0, 2.0));      // [15]
    box->AddValue(Clamp(dMargin, -20.0, 20.0));   // [16]
    for (uint32_t k = 0; k < m_topN; k++)
    {
        box->AddValue(Clamp(dSlot[k], -20.0, 20.0)); // [17..21]
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

    // [22] sinr
    box->AddValue(Clamp(sinrVal, -40.0, 50.0));

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

    // [26] tbs — average TBS over step. UseTbsObservation=false emits a constant
    // (0.0) so the policy cannot key on the goodput-reward proxy; the obs layout
    // and bounds are unchanged (easy revert by setting the attribute back).
    double avgTbs = (m_useTbsObservation && m_tbsCount > 0)
                         ? static_cast<double>(m_tbsSum) / static_cast<double>(m_tbsCount)
                         : 0.0;
    box->AddValue(Clamp(avgTbs, 0.0, 100000.0));

    // Reset TBS accumulators for next step
    m_tbsSum = 0;
    m_tbsCount = 0;

    // [27] time_since_ho — seconds since last handover, clamped to 10s
    double timeSinceHo = (m_lastHandoverTime > Seconds(0))
                             ? (Simulator::Now() - m_lastHandoverTime).GetSeconds()
                             : 10.0;
    box->AddValue(Clamp(timeSinceHo, 0.0, 10.0));

    // [28] norm_goodput — windowed normalized goodput (computed above for the
    // time-delta block; 0 = no data flowing).
    box->AddValue(Clamp(normGoodput, 0.0, 2.0));

    // [29] ho_count_10s — handovers within the sliding window (clamped to 10).
    // The windowed count makes the reward's handover-rate penalty observable:
    // sustained churn (count > budget) is what triggers the signaling cost.
    double nowS = Simulator::Now().GetSeconds();
    double windowSec = m_hoRateWindowMs / 1000.0;
    while (!m_hoTimes.empty() && m_hoTimes.front() < nowS - windowSec)
    {
        m_hoTimes.pop_front();
    }
    box->AddValue(Clamp(static_cast<double>(m_hoTimes.size()), 0.0, 10.0));

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
