#include "static-environment.h"

#include <ns3/constant-velocity-mobility-model.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/isotropic-antenna-model.h>
#include <ns3/mobility-helper.h>
#include <ns3/packet-sink-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/pointer.h>
#include <ns3/udp-client-server-helper.h>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("StaticMobComEnv");

//   (0,0)-----x-----x-----x-----(lineLength,0)
void
StaticMobComEnv::CreateBsLine(uint16_t numBs, uint64_t lineLength)
{
    NodeContainer enbNodes;
    enbNodes.Create(numBs);

    MobilityHelper mobility;
    // set position of the BSs
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto bsPositionAlloc = CreateObject<ListPositionAllocator>();
    // place bs in a way that they equally partition the x-axis
    int currentPosition = 0;
    for (int i = 0; i < numBs; i++)
    {
        currentPosition += float(lineLength) / (numBs + 1);
        bsPositionAlloc->Add(Vector(currentPosition, 0, 1));
        std::cout << "BS placed at: " << Vector(currentPosition, 0, 1) << std::endl;
    }

    mobility.SetPositionAllocator(bsPositionAlloc);
    mobility.Install(enbNodes);

    m_deviceManager->SetBsNodes(enbNodes);
    m_deviceManager->SetBsDevices(m_lteHelper->InstallEnbDevice(m_deviceManager->GetBsNodes()));
}

// randomized position of ues in max distance from center coordinate
// base stations distributed along the x-axis
void
StaticMobComEnv::CreateRandomFixedTopology(uint16_t numUe, uint16_t numBs, uint64_t lineLength)
{
    NS_LOG_FUNCTION(this);
    CreateBsLine(numBs, lineLength);

    NodeContainer ueNodes;
    ueNodes.Create(numUe);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    auto x = CreateObject<UniformRandomVariable>();
    x->SetAttribute("Min", DoubleValue(0));
    x->SetAttribute("Max", DoubleValue(lineLength));

    auto y = CreateObject<UniformRandomVariable>();
    y->SetAttribute("Min", DoubleValue(-lineLength / 2));
    y->SetAttribute("Max", DoubleValue(lineLength / 2));

    auto uePositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numUe; i++)
    {
        auto xCoordinate = x->GetValue();
        auto yCoordinate = y->GetValue();
        uePositionAlloc->Add(Vector(xCoordinate, yCoordinate, 1));
        std::cout << "UE placed at: " << Vector(xCoordinate, yCoordinate, 1) << std::endl;
    }

    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    m_deviceManager->SetUtNodes(ueNodes);
    m_deviceManager->SetUtDevices(m_lteHelper->InstallUeDevice(m_deviceManager->GetUtNodes()));
}

// Test1: 1 BS at 0,0,0 - 1 UE at x,y,z; with velocity v -> set v to (0,0,0) to model static env
void
StaticMobComEnv::CreateTestTopology1(Vector position, Vector velocity)
{
    NodeContainer enbNodes;
    enbNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto bsPositionAlloc = CreateObject<ListPositionAllocator>();
    bsPositionAlloc->Add(Vector(0, 0, 1));
    mobility.SetPositionAllocator(bsPositionAlloc);
    mobility.Install(enbNodes);
    m_deviceManager->SetBsNodes(enbNodes);
    m_deviceManager->SetBsDevices(m_lteHelper->InstallEnbDevice(m_deviceManager->GetBsNodes()));

    NodeContainer ueNodes;
    ueNodes.Create(1);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    auto uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(position);
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(velocity);
    m_deviceManager->SetUtNodes(ueNodes);
    m_deviceManager->SetUtDevices(m_lteHelper->InstallUeDevice(m_deviceManager->GetUtNodes()));
}

// Test 2: " 2 Node pairs, static"
void
StaticMobComEnv::CreateTestTopology2()
{
    NodeContainer enbNodes;
    enbNodes.Create(2);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    auto bsPositionAlloc = CreateObject<ListPositionAllocator>();
    bsPositionAlloc->Add(Vector(0, 0, 1));
    bsPositionAlloc->Add(Vector(1000, 0, 1));
    mobility.SetPositionAllocator(bsPositionAlloc);
    mobility.Install(enbNodes);
    m_deviceManager->SetBsNodes(enbNodes);
    m_deviceManager->SetBsDevices(m_lteHelper->InstallEnbDevice(m_deviceManager->GetBsNodes()));

    NodeContainer ueNodes;
    ueNodes.Create(2);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    auto uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(Vector(0, 500, 1));
    uePositionAlloc->Add(Vector(1000, 500, 1));
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);
    ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
    ueNodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
    m_deviceManager->SetUtNodes(ueNodes);
    m_deviceManager->SetUtDevices(m_lteHelper->InstallUeDevice(m_deviceManager->GetUtNodes()));
}

void
StaticMobComEnv::CreateTopology()
{
    CreateTestTopology1(Vector(1000, 0, 1), Vector(200, 0, 0));
}

void
StaticMobComEnv::AddTraffic()
{
    Ptr<Node> pgw = m_epcHelper->GetPgwNode();

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
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer& ueNodes = m_deviceManager->GetUtNodes();
    NetDeviceContainer& ueDevices = m_deviceManager->GetUtDevices();
    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = m_epcHelper->AssignUeIpv4Address(ueDevices);
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(m_epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    AttachUEsToEnbs();

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 1100;
    uint16_t ulPort = 2000;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Time interPacketInterval = MilliSeconds(1); // TODO: we could use parameters for this

        // downlink traffic
        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(interPacketInterval));
        dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        clientApps.Add(dlClient.Install(remoteHost));

        // uplink traffic
        ++ulPort;
        PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

        UdpClientHelper ulClient(remoteHostAddr, ulPort);
        ulClient.SetAttribute("Interval", TimeValue(interPacketInterval));
        ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
        clientApps.Add(ulClient.Install(ueNodes.Get(u)));
    }

    serverApps.Start(MilliSeconds(100));
    clientApps.Start(MilliSeconds(100));
}

} // namespace ns3
