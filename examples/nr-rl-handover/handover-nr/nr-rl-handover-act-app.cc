#include "nr-rl-handover-act-app.h"

#include "ns3/base-test.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <cstdint>
#include <fstream>

using namespace ns3;

// External globals from the scenario (outside namespace ns3 to match definitions in
// nr-rl-handover-scenario.cc)
extern NetDeviceContainer g_uavNrDevs;
extern NetDeviceContainer g_gnbNrDevs;
extern Ptr<NrHelper> g_nrHelper;
extern uint32_t g_totalHandovers;
extern bool g_tcpConnected;
extern bool g_handoverInProgress;
extern std::vector<double> g_lastRsrpValues;
extern std::vector<double> g_lastRsrqValues;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlHandoverActionApp");

NrRlHandoverActionApp::NrRlHandoverActionApp()
    : ActionApplication()
{
}

NrRlHandoverActionApp::~NrRlHandoverActionApp()
{
}

TypeId
NrRlHandoverActionApp::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrRlHandoverActionApp")
            .SetParent<ActionApplication>()
            .SetGroupName("defiance")
            .AddConstructor<NrRlHandoverActionApp>()
            .AddAttribute("NumBs",
                          "Number of base stations in the simulation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&NrRlHandoverActionApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverAlgorithm",
                          "Handover algorithm: agent, a3, or noop.",
                          StringValue("agent"),
                          MakeStringAccessor(&NrRlHandoverActionApp::m_handoverAlgorithm),
                          MakeStringChecker())
            .AddAttribute("HandoverMargin",
                          "RSRP margin (3GPP range, ~1 dB per step). "
                          "Target must have RSRP > serving + margin. "
                          "Set to -999 to disable gating.",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&NrRlHandoverActionApp::m_handoverMargin),
                          MakeDoubleChecker<double>());
    return tid;
}

void
NrRlHandoverActionApp::ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action)
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
    if (g_uavNrDevs.GetN() == 0)
    {
        NS_LOG_WARN("No UAV LTE device, skipping handover.");
        return;
    }

    auto ueNrDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
    if (!ueNrDev)
    {
        NS_LOG_WARN("UAV LTE device is null, skipping handover.");
        return;
    }

    // --- Precondition 2: Is the UE in CONNECTED_NORMALLY state? ---
    auto ueRrc = ueNrDev->GetRrc();
    if (!ueRrc)
    {
        NS_LOG_WARN("UAV RRC is null, skipping handover.");
        return;
    }
    if (ueRrc->GetState() != NrUeRrc::CONNECTED_NORMALLY)
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

    // Log every action received from Python (before precondition gates)
    if (g_logging)
    {
        std::ofstream actFile(g_outputDir + "rl_action.csv", std::ios_base::app);
        actFile << Simulator::Now().GetSeconds() << ","
                << currentCellId << ","
                << newCellId << ","
                << (currentCellId < g_lastRsrpValues.size() ? g_lastRsrpValues[currentCellId] : -200.0) << ","
                << (newCellId < g_lastRsrpValues.size() ? g_lastRsrpValues[newCellId] : -200.0) << std::endl;
    }

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

    // --- Find the source gNB device by cell ID ---
    Ptr<NetDevice> sourceEnbDev;
    for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> gnbNetDev = g_gnbNrDevs.Get(i)->GetObject<NrGnbNetDevice>();
        if (gnbNetDev && gnbNetDev->GetCellId() == currentCellId)
        {
            sourceEnbDev = g_gnbNrDevs.Get(i);
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
    auto sourceEnbNetDev = sourceEnbDev->GetObject<NrGnbNetDevice>();
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
    auto ueImsi = ueNrDev->GetImsi();
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
    if (ueMgr->GetState() != NrUeManager::CONNECTED_NORMALLY)
    {
        NS_LOG_DEBUG("UE is amidst handover at source eNB, skipping.");
        return;
    }

    // --- Execute the handover ---
    if (!g_nrHelper)
    {
        NS_LOG_WARN("g_nrHelper is null, cannot execute handover.");
        return;
    }

    // --- Note: RSRP margin gate removed. SAC does not use the action mask,
    // so the agent learns from reward signal which cells are worth choosing.
    // The action mask in the obs app only applies when trainable is PPO.

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Handover UE RNTI=" << rnti << " cell "
                                              << currentCellId << " -> " << newCellId);

    std::cout << "Time: " << Simulator::Now().GetSeconds() << "s: Handover UE RNTI=" << rnti
              << " cell " << currentCellId
              << " rsrp_curr: " << g_lastRsrpValues[currentCellId]
              << " rsrq_curr: " << (currentCellId < g_lastRsrqValues.size()
                                         ? g_lastRsrqValues[currentCellId] : -200.0)
              << " -> " << newCellId
              << " rsrp_target: " << g_lastRsrpValues[newCellId]
              << " rsrq_target: " << (newCellId < g_lastRsrqValues.size()
                                           ? g_lastRsrqValues[newCellId] : -200.0)
              << std::endl;

    g_handoverInProgress = true;
    g_nrHelper->HandoverRequest(Seconds(0), g_uavNrDevs.Get(0), sourceEnbDev, newCellId);
    g_totalHandovers++;
}

} // namespace ns3
