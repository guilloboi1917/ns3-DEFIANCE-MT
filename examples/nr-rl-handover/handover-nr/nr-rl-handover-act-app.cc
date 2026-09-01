#include "nr-rl-handover-act-app.h"

#include "ns3/base-test.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <cstdint>
#include <fstream>

using namespace ns3;

// External globals from the scenario
extern NetDeviceContainer g_uavNrDevs;
extern NetDeviceContainer g_gnbNrDevs;
extern Ptr<NrHelper> g_nrHelper;
extern uint32_t g_totalHandovers;
extern bool g_handoverInProgress;
extern uint32_t g_topNCells[6];
extern std::vector<double> g_lastRsrpValues;
extern std::vector<double> g_lastRsrqValues;
extern bool g_logging;
extern std::string g_outputDir;

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrRlHandoverActionApp");

// Effective action of the most recent RL step: the action index the act-app
// actually executed (0 = stay: noop OR blocked). Read by the agent-app's
// GetExtraInfo() and shipped to Python via the existing info channel, so the
// replay can store the EXECUTED action instead of the blocked intent.
int g_effectiveAction = 0;

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
                          UintegerValue(9),
                          MakeUintegerAccessor(&NrRlHandoverActionApp::m_numBs),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HandoverAlgorithm",
                          "Handover algorithm: agent, a3, or noop.",
                          StringValue("agent"),
                          MakeStringAccessor(&NrRlHandoverActionApp::m_handoverAlgorithm),
                          MakeStringChecker());
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

    // --- Extract action index (0..m_topN) ---
    auto actionIndexContainer = DynamicCast<OpenGymDiscreteContainer>(action->Get("actionIndex"));
    if (!actionIndexContainer)
    {
        NS_LOG_WARN("Action dict missing 'actionIndex', skipping.");
        return;
    }
    uint32_t actionIndex = actionIndexContainer->GetValue();

    // --- Map action index to target cellId via g_topNCells (0 for noop) ---
    uint32_t targetCellId = g_topNCells[actionIndex];

    // --- Current serving cell (safe lookup; 0 if not yet attached) ---
    uint32_t currentCellId = 0;
    if (g_uavNrDevs.GetN() > 0)
    {
        auto ueDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
        if (ueDev && ueDev->GetRrc())
        {
            currentCellId = ueDev->GetRrc()->GetCellId();
        }
    }

    // --- Log every received action (incl. noop and blocked) ---
    // Columns: time, actionIndex, targetCellId, currentCellId, outcome
    auto logAction = [&](const std::string& outcome) {
        if (g_logging)
        {
            std::ofstream actFile(g_outputDir + "rl_actions_full.csv", std::ios_base::app);
            actFile << Simulator::Now().GetSeconds() << ","
                    << actionIndex << ","
                    << targetCellId << ","
                    << currentCellId << ","
                    << outcome << std::endl;
        }
    };

    // --- Effective action: default to stay (noop or any block) ---
    g_effectiveAction = 0;

    // --- No-op? ---
    if (actionIndex == 0)
    {
        logAction("noop");
        NS_LOG_DEBUG("No-op (actionIndex=0), skipping handover.");
        return;
    }

    // --- Validate action index bounds ---
    if (actionIndex > m_topN)
    {
        logAction("invalid-action-index");
        NS_LOG_WARN("Invalid actionIndex " << actionIndex << " (max=" << m_topN << "), skipping.");
        return;
    }

    // --- Validate target cell ---
    if (targetCellId == 0)
    {
        logAction("no-valid-cell");
        NS_LOG_DEBUG("g_topNCells[" << actionIndex << "] = 0 (no valid cell), skipping.");
        return;
    }

    if (targetCellId > m_numBs)
    {
        logAction("invalid-cell");
        NS_LOG_WARN("Invalid target cell " << targetCellId << " (max=" << m_numBs << "), skipping.");
        return;
    }

    // --- Precondition: Is a handover already in progress? ---
    if (g_handoverInProgress)
    {
        logAction("blocked-in-progress");
        NS_LOG_DEBUG("Handover already in progress, deferring.");
        return;
    }

    // --- Precondition: Does the UAV NR device exist? ---
    if (g_uavNrDevs.GetN() == 0)
    {
        logAction("blocked-no-device");
        NS_LOG_WARN("No UAV NR device, skipping handover.");
        return;
    }

    auto ueNrDev = g_uavNrDevs.Get(0)->GetObject<NrUeNetDevice>();
    if (!ueNrDev)
    {
        logAction("blocked-no-uedev");
        NS_LOG_WARN("UAV NR device is null, skipping handover.");
        return;
    }

    // --- Precondition: Is the UE in CONNECTED_NORMALLY state? ---
    auto ueRrc = ueNrDev->GetRrc();
    if (!ueRrc)
    {
        logAction("blocked-no-rrc");
        NS_LOG_WARN("UAV RRC is null, skipping handover.");
        return;
    }
    if (ueRrc->GetState() != NrUeRrc::CONNECTED_NORMALLY)
    {
        logAction("blocked-rrc-state");
        NS_LOG_DEBUG("UE not in CONNECTED_NORMALLY (state=" << ueRrc->GetState()
                                                            << "), skipping.");
        return;
    }

    currentCellId = ueRrc->GetCellId();

    // --- Precondition: Same cell? ---
    if (targetCellId == currentCellId)
    {
        logAction("blocked-same-cell");
        NS_LOG_DEBUG("Target cell " << targetCellId
                      << " is same as current cell, skipping.");
        return;
    }

    // --- Log action ---
    if (g_logging)
    {
        std::ofstream actFile(g_outputDir + "rl_action.csv", std::ios_base::app);
        actFile << Simulator::Now().GetSeconds() << ","
                << currentCellId << ","
                << targetCellId << ","
                << (currentCellId < g_lastRsrpValues.size()
                        ? g_lastRsrpValues[currentCellId] : -200.0)
                << ","
                << (targetCellId < g_lastRsrpValues.size()
                        ? g_lastRsrpValues[targetCellId] : -200.0)
                << std::endl;
    }

    // --- Find the source gNB device by cell ID ---
    Ptr<NetDevice> sourceGnbDev;
    for (uint32_t i = 0; i < g_gnbNrDevs.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> gnbNetDev = g_gnbNrDevs.Get(i)->GetObject<NrGnbNetDevice>();
        if (gnbNetDev && gnbNetDev->GetCellId() == currentCellId)
        {
            sourceGnbDev = g_gnbNrDevs.Get(i);
            break;
        }
    }

    if (!sourceGnbDev)
    {
        logAction("blocked-no-source-gnb");
        NS_LOG_WARN("Could not find source gNB for cell " << currentCellId);
        return;
    }

    // --- Get source gNB RRC ---
    uint16_t rnti = ueRrc->GetRnti();
    auto sourceGnbNetDev = sourceGnbDev->GetObject<NrGnbNetDevice>();
    if (!sourceGnbNetDev)
    {
        logAction("blocked-no-source-dev");
        NS_LOG_WARN("Source gNB net device is null.");
        return;
    }
    auto sourceGnbRrc = sourceGnbNetDev->GetRrc();
    if (!sourceGnbRrc)
    {
        logAction("blocked-no-source-rrc");
        NS_LOG_WARN("Source gNB RRC is null.");
        return;
    }

    // --- Precondition: Does the source gNB have the UE's UeManager? ---
    if (!sourceGnbRrc->HasUeManager(rnti))
    {
        logAction("blocked-no-uemanager");
        NS_LOG_DEBUG("Source gNB does not have UeManager for RNTI " << rnti);
        return;
    }

    // --- Precondition: Is the UE connected to this gNB? ---
    auto ueImsi = ueNrDev->GetImsi();
    auto ueMgr = sourceGnbRrc->GetUeManager(rnti);
    if (!ueMgr)
    {
        logAction("blocked-no-uemanager");
        NS_LOG_DEBUG("UeManager is null for RNTI " << rnti);
        return;
    }
    if (ueImsi != ueMgr->GetImsi())
    {
        logAction("blocked-imsi-mismatch");
        NS_LOG_DEBUG("UE IMSI mismatch at source gNB");
        return;
    }

    // --- Precondition: Is UE amidst handover? ---
    if (ueMgr->GetState() != NrUeManager::CONNECTED_NORMALLY)
    {
        logAction("blocked-amidst-handover");
        NS_LOG_DEBUG("UE is amidst handover at source gNB, skipping.");
        return;
    }

    // --- Execute the handover ---
    if (!g_nrHelper)
    {
        logAction("blocked-no-helper");
        NS_LOG_WARN("g_nrHelper is null, cannot execute handover.");
        return;
    }

    logAction("executed");
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Handover UE RNTI=" << rnti
                << " cell " << currentCellId << " -> " << targetCellId
                << " (actionIndex=" << actionIndex << ")");

    g_effectiveAction = static_cast<int>(actionIndex);
    g_handoverInProgress = true;
    g_nrHelper->HandoverRequest(Seconds(0), g_uavNrDevs.Get(0), sourceGnbDev, targetCellId);
    g_totalHandovers++;
}

} // namespace ns3
