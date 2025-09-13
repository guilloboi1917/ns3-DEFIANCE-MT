#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
using namespace ns3;

void
NotifyConnectionEstablished(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::cout << "Connection established for UE-ID " << rnti << std::endl;
}

void
NotifyRandomAccessSuccessfulUe(RadioBearerStatsConnector* c,
                               std::string context,
                               uint64_t imsi,
                               uint16_t cellId,
                               uint16_t rnti)
{
    std::cout << "Random Access successful for UE-ID " << rnti << std::endl;
}

void
printProgress(int32_t time)
{
    std::cout << "Progress: Simulation time " << time << std::endl;
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("LteHelper", LOG_LEVEL_INFO);

    auto lteHelper = CreateObject<LteHelper>();
    // This will instantiate some common objects (e.g., the Channel object) and provide the methods
    // to add eNBs and UEs and configure them.

    // Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10000)));
    // Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(10000));
    // Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    // Create Node objects for the eNB(s) and the UEs:
    NodeContainer enbNodes;
    enbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(20);

    // Note that the above Node instances at this point still don’t have an LTE protocol stack
    // installed; they’re just empty nodes.

    // Configure the Mobility model for all the nodes:
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);

    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=1.5]"),
                              "Pause",
                              StringValue("ns3::ConstantRandomVariable[Constant=4.0]"),
                              "PositionAllocator",
                              PointerValue(taPositionAlloc));
    mobility.SetPositionAllocator(taPositionAlloc);
    mobility.Install(ueNodes);

    // Install an LTE protocol stack on the eNB(s) and UEs:
    NetDeviceContainer enbDevs;
    enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs;
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // Attach the UEs to an eNB. This will configure each UE according to the eNB configuration, and
    // create an RRC connection between them:
    // lteHelper->Attach(ueDevs, enbDevs.Get(0));

    // Activate a data radio bearer between each UE and the eNB it is attached to:

    /*enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
    EpsBearer bearer(q);
    lteHelper->ActivateDataRadioBearer(ueDevs, bearer);*/

    // this method will also activate two saturation traffic generators for that bearer, one in
    // uplink and one in downlink.

    /// This is needed otherwise the simulation will last forever, because (among others) the
    /// start-of-subframe event is scheduled repeatedly, and the ns-3 simulator scheduler will hence
    /// never run out of events.
    /*lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();*/

    AnimationInterface::SetConstantPosition(enbNodes.Get(0), 150, 150);
    AnimationInterface anim("lte-traces.xml");
    anim.SetMobilityPollInterval(Seconds(1.00));
    anim.SetMaxPktsPerTraceFile(1000000);
    anim.UpdateNodeDescription(enbNodes.Get(0), "eNb");
    anim.UpdateNodeColor(enbNodes.Get(0), 0, 255, 0);
    // anim.EnablePacketMetadata(true);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablished));
    // Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful",
    //                 MakeCallback(&NotifyRandomAccessSuccessfulUe));

    Simulator::Stop(Seconds(2000));

    // Schedule printing simulation progess every 10 seconds
    for (int i = 10; i < 2000; i += 10)
    {
        Simulator::Schedule(Seconds(i), &printProgress, i);
    }

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
