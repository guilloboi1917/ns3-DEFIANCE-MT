#include "uav-handover-action-application.h"
#include "uav-handover-agent-application.h"
#include "uav-handover-observation-application.h"
#include "uav-handover-reward-application.h"

#include "ns3/applications-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/communication-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"
#include "ns3/rl-application-helper.h"
#include "ns3/tcp-bbr.h"
#include "ns3/tcp-l4-protocol.h"
#include "ns3/tcp-socket-base.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

// Forward declaration from uav-handover-scenario.cc
extern std::string pathToNs3;

/**
 * Callback to track the TCP congestion window of the primary UE's TCP socket.
 * The callback signature matches TcpSocketBase::m_cWndTrace:
 * TracedCallback<uint32_t, uint32_t> (oldCwnd, newCwnd)
 */
uint32_t g_currentCwnd = 0;

void
CwndTracer(uint32_t oldCwnd, uint32_t newCwnd)
{
    g_currentCwnd = newCwnd;
    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s CWND: " << oldCwnd << " -> " << newCwnd);
}

/**
 * Save throughput and cwnd stats of an inferring agent to CSV files.
 */
void
SaveStatsTcp(uint32_t bytesReceived,
             Time statInterval,
             uint32_t& bytesReceivedAccum,
             Time& lastStatCalculation,
             const std::string& statsFilename,
             const std::string& cwndFilename)
{
    if (Simulator::Now() - lastStatCalculation >= statInterval)
    {
        auto throughput =
            static_cast<double>(bytesReceivedAccum) * 8.0 / statInterval.GetSeconds(); // in Bit/s

        // Save throughput stats
        std::ofstream statsFile(statsFilename, std::ios_base::app);
        statsFile << Simulator::Now().GetSeconds() << "," << throughput << std::endl;

        // Save cwnd stats
        std::ofstream cwndFile(cwndFilename, std::ios_base::app);
        cwndFile << Simulator::Now().GetSeconds() << "," << g_currentCwnd << std::endl;

        bytesReceivedAccum = 0;
        lastStatCalculation = Simulator::Now();
    }
}

inline void
scenarioSetup(uint16_t numberOfUes,
              uint16_t numberOfEnbs,
              uint16_t numBearersPerUe = 1,
              double distance = 200.0,
              double speed = 20.0,
              double simTime = 100.0,
              uint32_t stepTime = 420,
              uint32_t delay = 0,
              double enbTxPowerDbm = 43.0, // eNB Tx power in dBm (43.0 for UMa)
              uint32_t seed = 0,
              uint32_t runId = 0,
              bool visualize = false,
              std::string trialName = "1",
              std::string handoverAlgorithm = "agent",
              double handoverPenalty = 1000.0,
              double ueHeight = 10.0,
              double enbHeight = 25.0,
              std::string propagationModel = "ThreeGppUma",
              std::string statsDir = "")
{
    auto lteHelper = CreateObject<LteHelper>();
    auto epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    if (handoverAlgorithm == "agent") // disable automatic handover
    {
        lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
    }
    else if (handoverAlgorithm == "a2a4")
    {
        lteHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
    }
    else if (handoverAlgorithm == "a3")
    {
        lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    }
    else
    {
        NS_FATAL_ERROR("Unknown handover algorithm");
    }
    // Configure propagation loss model
    if (propagationModel == "LogDistance")
    {
        // Simple LogDistance model (fast, matches original handover example)
        lteHelper->SetPathlossModelType(LogDistancePropagationLossModel::GetTypeId());
        lteHelper->SetPathlossModelAttribute("Exponent", DoubleValue(2.2));
        lteHelper->SetPathlossModelAttribute("ReferenceDistance", DoubleValue(1.0));
    }
    else if (propagationModel == "ThreeGppUma")
    {
        // 3GPP TR 38.901 UMa (Urban Macro) — more accurate but slower
        lteHelper->SetPathlossModelType(ThreeGppUmaPropagationLossModel::GetTypeId());
        lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.0e9));
        lteHelper->SetPathlossModelAttribute("ShadowingEnabled", BooleanValue(false));
        Ptr<ThreeGppUmaChannelConditionModel> condModel =
            CreateObject<ThreeGppUmaChannelConditionModel>();
        lteHelper->SetPathlossModelAttribute("ChannelConditionModel", PointerValue(condModel));
    }
    else
    {
        NS_FATAL_ERROR("Unknown propagation model: " << propagationModel
                                                      << ". Use LogDistance or ThreeGppUma.");
    }

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    // Routing of the Internet Host (towards the LTE network)
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    // interface 0 is localhost, 1 is the p2p device
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // Install Mobility Model in eNB
    auto enbPositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
        double xPos = (i % 2 == 0) ? 0 : distance;
        double yPos = (i / 2) * distance;
        Vector enbPosition(xPos, yPos, enbHeight); // default 25m for UMa
        enbPositionAlloc->Add(enbPosition);
    }
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator(enbPositionAlloc);
    enbMobility.Install(enbNodes);

    RngSeedManager::SetSeed(seed == 0U ? time(nullptr) : seed);
    RngSeedManager::SetRun(runId == 0U ? 1 : runId);
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set(
        "X",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(distance) + "]"));
    NS_ASSERT_MSG(numberOfEnbs > 0, "Number of eNBs cannot be 0.");
    auto yBound = distance * ((numberOfEnbs - 1) / 2);
    pos.Set("Y",
            StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(yBound) + "]"));
    pos.Set("Z", StringValue(std::to_string(ueHeight)));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

    // Install Mobility Model in UE
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel(
        "ns3::RandomWaypointMobilityModel",
        "Speed",
        StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(speed) + "]"),
        "Pause",
        StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
        "PositionAllocator",
        PointerValue(taPositionAlloc));
    ueMobility.SetPositionAllocator(taPositionAlloc);
    ueMobility.Install(ueNodes);

    // Install LTE Devices in eNB and UEs
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(23.0)); // Standard UE max power (dBm)
    g_enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    g_ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    // Install the IP stack on the UEs using TCP BBR as the congestion control
    // We need to set the SocketType on the TcpL4Protocol before internet stack is installed
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpBbr::GetTypeId()));
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(g_ueLteDevs));

    // Attach all UEs to their respective closest eNodeB
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ueNode = ueNodes.Get(i);
        Ptr<NetDevice> ueLteDev = g_ueLteDevs.Get(i);

        double minDistance = std::numeric_limits<double>::max();
        Ptr<NetDevice> closestEnbLteDev;

        for (uint32_t j = 0; j < enbNodes.GetN(); ++j)
        {
            Ptr<Node> enbNode = enbNodes.Get(j);
            Ptr<NetDevice> enbLteDev = g_enbLteDevs.Get(j);

            Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
            Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
            double distance = ueMobility->GetDistanceFrom(enbMobility);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestEnbLteDev = enbLteDev;
            }
        }

        lteHelper->Attach(ueLteDev, closestEnbLteDev);
    }

    // Install and start TCP applications on UEs and remote host
    uint16_t dlPort = 10000;

    auto startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetAttribute("Min", DoubleValue(0));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.010));

    // Variables for tracking bytes received at UE for stats file (visualize mode)
    uint32_t bytesReceivedAccum = 0;
    Time lastStatCalculation = Seconds(0);
    Time statInterval = MilliSeconds(1000);
    std::string statsFilename;
    std::string cwndFilename;

    std::cout << "Visualize mode: " << (visualize ? "ON" : "OFF") << std::endl;

    if (visualize)
    {
        std::string baseDir = statsDir.empty()
            ? pathToNs3 + "/contrib/defiance/examples/uav-handover/"
            : statsDir + "/";

        std::cout << "Stats will be saved to "
                  << baseDir + "uav-handover-stats_" + std::to_string(seed) + "_" + std::to_string(runId) + ".csv" << std::endl;
        std::cout << "CWND stats will be saved to "
                  << baseDir + "uav-handover-cwnd_" + std::to_string(seed) + "_" + std::to_string(runId) + ".csv" << std::endl;
        statsFilename = baseDir + "uav-handover-stats_" + std::to_string(seed) + "_" + std::to_string(runId) + ".csv";
        cwndFilename = baseDir + "uav-handover-cwnd_" + std::to_string(seed) + "_" + std::to_string(runId) + ".csv";
    }
    else {
        std::cout << "Not visualizing, stats will not be saved." << std::endl;
    }

    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
            ++dlPort;

            // Uplink TCP: UE sends data to remoteHost, remoteHost receives
            // RemoteHost side: TCP sink (receiver)
            PacketSinkHelper tcpSinkHelper("ns3::TcpSocketFactory",
                                           InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            auto serverApps = tcpSinkHelper.Install(remoteHost);

            // UE side: BulkSend (sender, continuous)
            auto remoteHostAddr = internetIpIfaces.GetAddress(1);
            BulkSendHelper tcpClientHelper(
                "ns3::TcpSocketFactory",
                InetSocketAddress(remoteHostAddr, dlPort));
            tcpClientHelper.SetAttribute("MaxBytes", UintegerValue(0)); // unlimited send
            tcpClientHelper.SetAttribute("SendSize", UintegerValue(655));
            auto clientApps = tcpClientHelper.Install(ue);

            Time startTime = Seconds(startTimeSeconds->GetValue() + 2.5); // Start after LTE attach and RL apps
            serverApps.Start(startTime);
            clientApps.Start(startTime);

            // Only track TCP cwnd for the primary UE (UE0)
            if (u == 0 && b == 0)
            {
                // std::cout << "UE" << u << " TCP BulkSend: remoteHost=" << remoteHostAddr
                //           << ":" << dlPort << " start=" << startTime.GetSeconds() << "s"
                //           << std::endl;

                // Schedule a callback after applications have started to connect to
                // the congestion window trace on UE0's TCP socket (the sender).
                // The first TCP socket created on the UE gets key 0 in SocketList.
                auto ueNodeId = ueNodes.Get(0)->GetId();
                // std::cout << "Scheduling cwnd trace for UE node " << ueNodeId << " at 1.5s"
                //           << std::endl;
                Simulator::Schedule(Seconds(3.0), [ueNodeId]() {
                    std::string path = "/NodeList/" + std::to_string(ueNodeId) +
                                       "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow";
                    Config::ConnectWithoutContext(path, MakeCallback(&CwndTracer));
                });

                // Track bytes received at remoteHost for stats output (visualize mode)
                if (visualize)
                {
                    Ptr<PacketSink> sink = serverApps.Get(0)->GetObject<PacketSink>();
                    if (sink)
                    {
                        for (double t = 3.0; t < simTime; t += 1.0)
                        {
                            Simulator::Schedule(
                                Seconds(t),
                                [sink, statInterval, &bytesReceivedAccum, &lastStatCalculation,
                                 statsFilename, cwndFilename]()
                                {
                                    uint32_t newBytes =
                                        sink->GetTotalRx() - bytesReceivedAccum;
                                    bytesReceivedAccum += newBytes;
                                    SaveStatsTcp(newBytes,
                                                 statInterval,
                                                 bytesReceivedAccum,
                                                 lastStatCalculation,
                                                 statsFilename,
                                                 cwndFilename);
                                });
                        }
                    }
                }
            }
        }
    }

    lteHelper->AddX2Interface(enbNodes);

    // Add measurement configuration
    LteRrcSap::ReportConfigEutra reportConfig;
    reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS120;
    NetDeviceContainer::Iterator it;
    for (it = g_enbLteDevs.Begin(); it != g_enbLteDevs.End(); ++it)
    {
        Ptr<NetDevice> netDevice = *it;
        Ptr<LteEnbNetDevice> enbLteNetDevice = netDevice->GetObject<LteEnbNetDevice>();
        Ptr<LteEnbRrc> enbRrc = enbLteNetDevice->GetRrc();
        enbRrc->AddUeMeasReportConfig(reportConfig);
    }

    // Framework code
    RlApplicationHelper appHelper(UavHandoverRewardApplication::GetTypeId());
    appHelper.SetAttribute("StartTime", TimeValue(Seconds(1)));
    appHelper.SetAttribute("StopTime", TimeValue(Seconds(simTime)));
    appHelper.SetAttribute("SimulationTime", TimeValue(Seconds(simTime)));
    appHelper.SetAttribute("StepTime", TimeValue(MilliSeconds(stepTime)));
    appHelper.SetAttribute("LteHelper", PointerValue(lteHelper));
    appHelper.SetAttribute("HandoverPenalty", DoubleValue(handoverPenalty));
    // UE 0 will be the one we control
    auto rewardApps = appHelper.Install(ueNodes.Get(0));

    appHelper.SetTypeId(UavHandoverAgentApplication::GetTypeId());
    appHelper.SetAttribute("NumUes", UintegerValue(numberOfUes));
    appHelper.SetAttribute("NumBs", UintegerValue(numberOfEnbs));
    appHelper.SetAttribute("StepTime", UintegerValue(stepTime));
    auto agentApps = appHelper.Install(remoteHost);

    appHelper.SetTypeId(UavHandoverObservationApplication::GetTypeId());
    appHelper.SetAttribute("NumBs", UintegerValue(numberOfEnbs));
    appHelper.SetAttribute("StepTime", UintegerValue(stepTime));
    auto observationApps = appHelper.Install(enbNodes);

    appHelper.SetTypeId(UavHandoverActionApplication::GetTypeId());
    appHelper.SetAttribute("HandoverAlgorithm", StringValue(handoverAlgorithm));
    appHelper.SetAttribute("NumBs", UintegerValue(numberOfEnbs));
    appHelper.SetAttribute("LteHelper", PointerValue(lteHelper));
    auto actionApps = appHelper.Install(enbNodes);

    CommunicationHelper commHelper;
    commHelper.SetAgentApps(agentApps);
    commHelper.SetActionApps(actionApps);
    commHelper.SetObservationApps(observationApps);
    commHelper.SetRewardApps(rewardApps);
    commHelper.SetIds();

    // Set the communication attributes
    for (uint32_t i = 0; i < numberOfEnbs; i++)
    {
        commHelper.AddCommunication(
            {CommunicationPair{observationApps.GetId(i),
                               agentApps.GetId(0),
                               CommunicationAttributes{MilliSeconds(delay)}}});

        commHelper.AddCommunication(
            {CommunicationPair{actionApps.GetId(i),
                               agentApps.GetId(0),
                               CommunicationAttributes{MilliSeconds(delay)}}});
    }
    commHelper.AddCommunication({CommunicationPair{rewardApps.GetId(0),
                                                   agentApps.GetId(0),
                                                   CommunicationAttributes{MilliSeconds(delay)}}});

    commHelper.Configure();
}
