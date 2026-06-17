#include "harl-tcp-handover-act-app.h"

#include "ns3/base-test.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-enb-rrc.h"
#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/network-module.h"

#include <cstdint>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in
// harl-tcp-scenario.cc)
extern NetDeviceContainer g_uavLteDevs;
extern NetDeviceContainer g_enbLteDevs;
extern Ptr<LteHelper> g_lteHelper;
extern uint32_t g_totalHandovers;
extern bool g_tcpConnected;
extern bool g_handoverInProgress;
extern std::vector<int32_t> g_lastRsrpValues;
extern std::vector<double> g_lastSinrValues;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("HarlTcpHandoverActionApp");

HarlTcpHandoverActionApp::HarlTcpHandoverActionApp()
    : ActionApplication()
{
}

HarlTcpHandoverActionApp::~HarlTcpHandoverActionApp()
{
}

TypeId
HarlTcpHandoverActionApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::HarlTcpHandoverActionApp")
            .SetParent<ActionApplication>()
            .SetGroupName("defiance")
            .AddConstructor<HarlTcpHandoverActionApp>()
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&HarlTcpHandoverActionApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverAlgorithm",
                          "Handover algorithm: agent, a3, or noop.",
                          StringValue("agent"),
                          MakeStringAccessor(&HarlTcpHandoverActionApp::m_handoverAlgorithm),
                          MakeStringChecker())
            .AddAttribute("HandoverMargin",
                          "RSRP margin (3GPP range, ~1 dB per step). "
                          "Target must have RSRP > serving + margin. "
                          "Set to -999 to disable gating.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&HarlTcpHandoverActionApp::m_handoverMargin),
                          MakeDoubleChecker<double>());
    return tid;
}

void
HarlTcpHandoverActionApp::ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action)
{
    NS_LOG_FUNCTION(this << remoteAppId << action);

    if (m_handoverAlgorithm != "agent")
    {
        NS_LOG_INFO("Handover algorithm is '" << m_handoverAlgorithm
                                              << "', not executing RL action.");
        return;
    }

    // --- Null check: action container ---
    if (!action)
    {
        NS_LOG_WARN("Action container is null, skipping.");
        return;
    }

    // --- Precondition 1: Is a handover already in progress? ---
    // Prevents dispatching a second HandoverRequest while the first is still
    // being prepared (avoids NS_FATAL "method unexpected in state HANDOVER_PREPARATION").
    if (g_handoverInProgress)
    {
        NS_LOG_DEBUG("Handover already in progress, deferring.");
        return;
    }

    // --- Precondition 2: Does the UAV LTE device exist? ---
    if (g_uavLteDevs.GetN() == 0)
    {
        NS_LOG_WARN("No UAV LTE device, skipping handover.");
        return;
    }

    auto ueLteDev = g_uavLteDevs.Get(0)->GetObject<LteUeNetDevice>();
    if (!ueLteDev)
    {
        NS_LOG_WARN("UAV LTE device is null, skipping handover.");
        return;
    }

    // --- Precondition 2: Is the UE in CONNECTED_NORMALLY state? ---
    auto ueRrc = ueLteDev->GetRrc();
    if (!ueRrc)
    {
        NS_LOG_WARN("UAV RRC is null, skipping handover.");
        return;
    }
    if (ueRrc->GetState() != LteUeRrc::CONNECTED_NORMALLY)
    {
        NS_LOG_DEBUG("UE not in CONNECTED_NORMALLY state (state=" << ueRrc->GetState()
                                                                  << "), skipping handover.");
        return;
    }

    // --- Get current cell ID ---
    uint32_t currentCellId = ueRrc->GetCellId();

    // --- Null check: action dict content ---
    auto cellIdContainer = DynamicCast<OpenGymDiscreteContainer>(action->Get("newCellId"));
    if (!cellIdContainer)
    {
        NS_LOG_WARN("Action dict missing 'newCellId', skipping.");
        return;
    }
    uint32_t newCellId = cellIdContainer->GetValue();

    NS_LOG_DEBUG("Handover attempt: cell " << currentCellId << " -> " << newCellId);

    // --- Precondition 3: No-op? ---
    if (newCellId == 0)
    {
        NS_LOG_DEBUG("No-op (action=0), skipping handover.");
        return;
    }

    // --- Precondition 4: Same cell? ---
    if (newCellId == currentCellId)
    {
        NS_LOG_DEBUG("Target cell is same as current cell, skipping handover.");
        return;
    }

    // --- Precondition 5: Is target cell valid? ---
    if (newCellId > m_numBs)
    {
        NS_LOG_WARN("Invalid target cell ID " << newCellId << " (max=" << m_numBs
                                              << "), skipping.");
        return;
    }

    // --- Find the source eNB device by cell ID ---
    Ptr<NetDevice> sourceEnbDev;
    for (uint32_t i = 0; i < g_enbLteDevs.GetN(); i++)
    {
        auto enbDev = g_enbLteDevs.Get(i)->GetObject<LteEnbNetDevice>();
        if (enbDev && enbDev->GetCellId() == currentCellId)
        {
            sourceEnbDev = g_enbLteDevs.Get(i);
            break;
        }
    }

    if (!sourceEnbDev)
    {
        NS_LOG_WARN("Could not find source eNB for cell " << currentCellId);
        return;
    }

    // --- Precondition 6: Get source eNB net device & RRC ---
    uint16_t rnti = ueRrc->GetRnti();
    auto sourceEnbNetDev = sourceEnbDev->GetObject<LteEnbNetDevice>();
    if (!sourceEnbNetDev)
    {
        NS_LOG_WARN("Source eNB net device is null.");
        return;
    }
    auto sourceEnbRrc = sourceEnbNetDev->GetRrc();
    if (!sourceEnbRrc)
    {
        NS_LOG_WARN("Source eNB RRC is null.");
        return;
    }

    // --- Precondition 7: Does the source eNB have the UE's UeManager? ---
    if (!sourceEnbRrc->HasUeManager(rnti))
    {
        NS_LOG_DEBUG("Source eNB does not have UeManager for RNTI " << rnti);
        return;
    }

    // --- Precondition 8: Is the UE connected to this eNB? ---
    auto ueImsi = ueLteDev->GetImsi();
    auto ueMgr = sourceEnbRrc->GetUeManager(rnti);
    if (!ueMgr)
    {
        NS_LOG_DEBUG("UeManager is null for RNTI " << rnti);
        return;
    }
    if (ueImsi != ueMgr->GetImsi())
    {
        NS_LOG_DEBUG("UE IMSI mismatch at source eNB");
        return;
    }

    // --- Precondition 9: Is UE amidst handover? ---
    if (ueMgr->GetState() != UeManager::CONNECTED_NORMALLY)
    {
        NS_LOG_DEBUG("UE is amidst handover at source eNB, skipping.");
        return;
    }

    // --- Execute the handover ---
    if (!g_lteHelper)
    {
        NS_LOG_WARN("g_lteHelper is null, cannot execute handover.");
        return;
    }

    // --- Precondition: Apply handover margin ---
    // Only handover if target cell RSRP > serving cell RSRP + margin.
    // RSRP values are in 3GPP range (0-97, -44 to -140 dBm mapping).
    if (m_handoverMargin > -999.0 && newCellId < g_lastRsrpValues.size() &&
        currentCellId < g_lastRsrpValues.size())
    {
        double servingRsrp = g_lastRsrpValues[currentCellId];
        double targetRsrp = g_lastRsrpValues[newCellId];

        // Block if either RSRP is unknown (-1) — no measurement available
        if (servingRsrp < 0 || targetRsrp < 0)
        {
            NS_LOG_DEBUG("Handover blocked: RSRP unknown (serving=" << servingRsrp
                          << " target=" << targetRsrp << ")");
            return;
        }

        if (targetRsrp < servingRsrp + m_handoverMargin)
        {
            NS_LOG_DEBUG("Handover blocked: target RSRP " << targetRsrp << " < serving "
                                                          << servingRsrp << " + margin "
                                                          << m_handoverMargin);
            return;
        }
    }

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Handover UE RNTI=" << rnti << " cell "
                                              << currentCellId << " -> " << newCellId);

    std::cout << "Time: " << Simulator::Now().GetSeconds() << "s: Handover UE RNTI=" << rnti
              << " cell " << currentCellId
              << " rsrp_curr: " << g_lastRsrpValues[currentCellId]
              << " sinr_curr: " << (currentCellId < g_lastSinrValues.size()
                                         ? g_lastSinrValues[currentCellId] : -40.0)
              << " -> " << newCellId
              << " rsrp_target: " << g_lastRsrpValues[newCellId]
              << " sinr_target: " << (newCellId < g_lastSinrValues.size()
                                          ? g_lastSinrValues[newCellId] : -40.0)
              << std::endl;

    g_handoverInProgress = true;
    g_lteHelper->HandoverRequest(Seconds(0), g_uavLteDevs.Get(0), sourceEnbDev, newCellId);
    g_totalHandovers++;
}

} // namespace ns3
