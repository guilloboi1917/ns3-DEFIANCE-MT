#include "environment-creator.h"

#include "pendulum-cart/pendulum-cart.h"

#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-helper.h"

#include <cstdint>
#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EnvironmentCreator");

EnvironmentCreator::EnvironmentCreator()
{
}

struct Point
{
    float x, y;
};

Point
FindCircleMidpoint(float x1, float x2, float radius)
{
    Point midpoint;
    float distance = abs(x1 - x2);

    if (distance > radius * 2)
    {
        midpoint.x = 0;
        midpoint.y = 0;
        return midpoint;
    }

    midpoint.x = (x1 + x2) / 2;
    midpoint.y = sqrt(pow(radius, 2) - pow(distance, 2));

    return midpoint;
}

float
ReweightDiameter(float diameter)
{
    float random =
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX); // TODO: Use a random variable

    return std::sqrt(random) * diameter;
}

void
PlaceEnb(Ptr<Node> node, Point point)
{
    MobilityHelper mobility;
    auto positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(point.x, point.y, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(node);
    NS_LOG_INFO("An ENB was places at x: " << point.x << " y: " << point.y);
}

NodeContainer&
EnvironmentCreator::GetCartNodes()
{
    return m_cartNodes;
}

NodeContainer&
EnvironmentCreator::GetEnbNodes()
{
    return m_enbNodes;
}

NodeContainer&
EnvironmentCreator::GetTrainingAgentNodes()
{
    return m_trainingAgentNodes;
}

NodeContainer&
EnvironmentCreator::GetInferenceAgentNodes()
{
    return m_inferenceAgentNodes;
}

Ptr<LteHelper>
EnvironmentCreator::GetLteHelper()
{
    return m_lteHelper;
}

Ptr<EpcHelper>
EnvironmentCreator::GetEpcHelper()
{
    return m_epcHelper;
}

void
EnvironmentCreator::PlaceEnbsRandomly()
{
    srand(43);

    float from = 0;
    float to = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (m_coverageRadius * 2)));

    // place the first enb
    Point circleMiddlePoint = FindCircleMidpoint(from, to, m_coverageRadius);
    PlaceEnb(m_enbNodes.Get(0), circleMiddlePoint);

    for (uint32_t i = 1; i < m_enbNodes.GetN(); i++)
    {
        // Make the next starting point more likely to be further away
        from = from + ReweightDiameter(m_coverageRadius * 2);
        to = from +
             static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (m_coverageRadius * 2)));

        Point circleMiddlePoint = FindCircleMidpoint(from, to, m_coverageRadius);
        PlaceEnb(m_enbNodes.Get(i), circleMiddlePoint);
    }

    m_trackEnd = to; // save how long the overall track is
}

void
EnvironmentCreator::CreateEnbs(int numEnbs, float coverageRadius)
{
    m_enbNodes.Create(numEnbs);
    m_coverageRadius = coverageRadius;
    PlaceEnbsRandomly();
}

void
EnvironmentCreator::CreateCarts(int numberOfCarts, float trackLength)
{
    for (int i = 0; i < numberOfCarts; i++)
    {
        auto cartNode = CreateObject<PendulumCart>(0);
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> randomAngle(-0.1, 0.1);
        cartNode->SetAngle(randomAngle(gen));
        std::uniform_real_distribution<float> randomAngleVelocity(-0.02, 0.02);
        cartNode->SetAngleVelocity(randomAngleVelocity(gen));
        m_cartNodes.Add(cartNode);
        Simulator::Schedule(Seconds(1.002), &PendulumCart::SimulateTimeTic, cartNode, 0.002);
        NS_LOG_INFO("A cart was placed at x: 0.0 y: 0.0");
    }
}

void
EnvironmentCreator::SetupPendulumScenario(uint32_t numCarts,
                                          uint32_t numEnbs,
                                          bool EnableTrainingServer)
{
    m_coverageRadius = 500;
    m_mode = ScenarioMode::PendulumBaseStations;
    // Create the nodes and place them at their starting positions
    CreateEnbs(numEnbs, m_coverageRadius);
    CreateCarts(numCarts, m_trackEnd);

    // for each enb we have one inference agent
    m_inferenceAgentNodes.Create(numEnbs);

    m_lteHelper = CreateObject<LteHelper>();
    m_epcHelper = CreateObject<PointToPointEpcHelper>();
    m_lteHelper->SetEpcHelper(m_epcHelper);

    // arguments for the lte helper that are useful for this scenario
    m_lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisPropagationLossModel"));
    m_lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
    m_lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
    m_lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));

    InternetStackHelper internet;
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    PointToPointHelper p2ph; // TODO: Choose useful arguments
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));

    Ptr<Node> pgw = m_epcHelper->GetPgwNode();

    // router
    // create a router that is connected to the pgw and the inference agent
    auto router = CreateObject<Node>();
    NS_LOG_INFO("Node Index of the router: " << router->GetId());
    internet.Install(router);

    // p2p link between router and pgw
    NetDeviceContainer pgwRouterDevices = p2ph.Install(pgw, router);

    ipv4h.SetBase("192.168.0.0", "255.255.0.0"); // network where the pgw and router live in
    Ipv4InterfaceContainer pgwRouterInterfaces = ipv4h.Assign(pgwRouterDevices);

    // Set up routing from router to ue subnet
    Ptr<Ipv4StaticRouting> routerStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(router->GetObject<Ipv4>());
    routerStaticRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"),
        Ipv4Mask("255.0.0.0"),
        Ipv4Address("192.168.0.1"),
        1); // 192.168.0.1 is the address of the pgw in this network (next hop)

    // and also in the other direction from pgw to 1.0.0.0 network
    Ptr<Ipv4StaticRouting> pgwStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
    pgwStaticRouting->AddNetworkRouteTo(
        Ipv4Address("1.0.0.0"),
        Ipv4Mask("255.0.0.0"),
        Ipv4Address("192.168.0.2"),
        3); // 192.168.0.2 is the address of the router which is chosen as the next hop

    // inference agents
    internet.Install(m_inferenceAgentNodes);
    // now we set the ip address for the inference agents (in this case only one)
    ipv4h.SetBase("1.0.0.0", "255.0.0.0"); // network where router and inference agents live in
    // now we connect each inference agent to the router
    for (uint32_t i = 0; i < m_inferenceAgentNodes.GetN(); i++)
    {
        NetDeviceContainer routerInferenceAgentDevices =
            p2ph.Install(router, m_inferenceAgentNodes.Get(i));
        Ipv4InterfaceContainer routerInferenceAgentInterfaces =
            ipv4h.Assign(routerInferenceAgentDevices);
        NS_LOG_INFO("IP Address of inference agent node "
                    << i << " ( global node index is " << m_inferenceAgentNodes.Get(i)->GetId()
                    << " ) : " << routerInferenceAgentInterfaces.GetAddress(1));
    }

    // we need to set routing from each inference agents to the 7.0.0.0 address via the router
    // address for the next hop
    uint16_t j = 1;
    for (uint32_t u = 0; u < m_inferenceAgentNodes.GetN(); u++)
    {
        std::stringstream ss;
        ss << u + j;
        Ptr<Ipv4StaticRouting> inferenceAgentStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(m_inferenceAgentNodes.Get(u)->GetObject<Ipv4>());
        inferenceAgentStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                       Ipv4Mask("255.0.0.0"),
                                                       Ipv4Address(("1.0.0." + ss.str()).c_str()),
                                                       1);
        j++;
    }

    // and we also set host routing from the router to the inference agents
    j = 2;
    for (uint32_t u = 0; u < m_inferenceAgentNodes.GetN(); u++)
    {
        std::stringstream ss;
        ss << u + j; // basically increment by 2 each iteration
        routerStaticRouting->AddHostRouteTo(Ipv4Address(("1.0.0." + ss.str()).c_str()), j);
        j++;
    }

    if (EnableTrainingServer)
    {
        // now we also setup our one share trainingAgent and allow routing to the inference agents
        // (and vice versa)
        m_trainingAgentNodes.Create(1);
        Ptr<Node> trainingAgentNode = m_trainingAgentNodes.Get(0);
        internet.Install(trainingAgentNode);

        NetDeviceContainer routerTrainingAgentDevices = p2ph.Install(router, trainingAgentNode);
        ipv4h.SetBase("2.0.0.0", "255.0.0.0"); // network where router and training agent live in
        Ipv4InterfaceContainer routerTrainingAgentInterfaces =
            ipv4h.Assign(routerTrainingAgentDevices);

        NS_LOG_INFO("IP Address of training agent node "
                    << 0 << " ( global node index is " << trainingAgentNode->GetId()
                    << " ) : " << routerTrainingAgentInterfaces.GetAddress(1));

        // now we do routing from the training agent to the inference agents
        Ptr<Ipv4StaticRouting> trainingAgentStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(trainingAgentNode->GetObject<Ipv4>());
        trainingAgentStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"),
                                                      Ipv4Mask("255.0.0.0"),
                                                      Ipv4Address("2.0.0.1"),
                                                      1);

        // and also in the oppsite direction so that inference agent can talk to the training agent
        routerStaticRouting->AddHostRouteTo(Ipv4Address("2.0.0.0"),
                                            numEnbs +
                                                2); // first interface connects router and pgw, i+1
                                                    // connects to the i-th inference agent

        // we need to set routing from each inference agent to the 2.0.0.0 subnet via the
        // appropriate router address as next hop
        j = 1;
        for (uint32_t u = 0; u < numEnbs; u++)
        {
            std::stringstream ss;
            ss << u + j;
            Ptr<Ipv4StaticRouting> inferenceAgentStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(m_inferenceAgentNodes.Get(u)->GetObject<Ipv4>());
            inferenceAgentStaticRouting->AddNetworkRouteTo(
                Ipv4Address("2.0.0.0"),
                Ipv4Mask("255.0.0.0"),
                Ipv4Address(("1.0.0." + ss.str()).c_str()),
                1);
            j++;
        }
    }

    // from now on only setup of ue and enb devices has to follow
    // Create the networking devices for carts and enb nodes
    NetDeviceContainer enbDevices;
    NetDeviceContainer cartDevices;

    enbDevices = m_lteHelper->InstallEnbDevice(m_enbNodes);
    cartDevices = m_lteHelper->InstallUeDevice(m_cartNodes);

    internet.Install(m_cartNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = m_epcHelper->AssignUeIpv4Address(NetDeviceContainer(cartDevices));

    for (uint32_t i = 0; i < m_cartNodes.GetN(); i++)
    {
        NS_LOG_INFO("IP Address of the cart node "
                    << i << " ( global node index is " << m_cartNodes.Get(i)->GetId() << " ) : "
                    << m_cartNodes.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0).GetAddress());
    }

    // Set up routing from the UEs to the PGW
    for (uint32_t i = 0; i < m_cartNodes.GetN(); ++i)
    {
        Ptr<Node> cart = m_cartNodes.Get(i);
        // Set the default gateway for the carts
        Ptr<Ipv4StaticRouting> cartStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(cart->GetObject<Ipv4>());
        cartStaticRouting->SetDefaultRoute(m_epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach carts and base stations
    m_lteHelper->Attach(cartDevices);
    m_lteHelper->AddX2Interface(m_enbNodes); // we need this enabled for the handover

    m_lteHelper->GetDownlinkSpectrumChannel()->AddPropagationLossModel(
        CreateObjectWithAttributes<RangePropagationLossModel>("MaxRange",
                                                              DoubleValue(m_coverageRadius)));
}
