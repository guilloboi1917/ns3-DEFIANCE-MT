#include "ns3/base-test.h"
#include "ns3/communication-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/reward-application.h"
#include "ns3/rl-application-helper.h"

#include <iostream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ApplicationCommunicationExample");

/**
 * @ingroup defiance
 * Observation application that registers to a callback to observe
 * the current position of the node it is installed on. This position is
 * packed into a DictContainer and sent to all registered agent applications.
 */
class PositionObservationApp : public ObservationApplication
{
  public:
    PositionObservationApp() {};
    ~PositionObservationApp() override {};
    static TypeId GetTypeId();
    Ptr<OpenGymDictContainer> CreateDictContainer(float x, float y, float z);
    void Observe(Ptr<const MobilityModel> observation);
    void RegisterCallbacks() override;
};

NS_OBJECT_ENSURE_REGISTERED(PositionObservationApp);

TypeId
PositionObservationApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::TestObservation")
                            .SetParent<ObservationApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PositionObservationApp>();
    return tid;
}

Ptr<OpenGymDictContainer>
PositionObservationApp::CreateDictContainer(float x, float y, float z)
{
    auto dictContainer = CreateObject<OpenGymDictContainer>();
    dictContainer->Add("position-x", MakeBoxContainer<float, float>(1, x));
    dictContainer->Add("position-y", MakeBoxContainer<float, float>(1, y));
    dictContainer->Add("position-z", MakeBoxContainer<float, float>(1, z));
    return dictContainer;
}

void
PositionObservationApp::Observe(Ptr<const MobilityModel> observation)
{
    std::vector<float> obs;
    Send(CreateDictContainer(observation->GetPosition().x, observation->GetPosition().y, observation->GetPosition().z), 0);
}

void
PositionObservationApp::RegisterCallbacks()
{
    Config::ConnectWithoutContext("/NodeList/*/$ns3::MobilityModel/CourseChange",
                                  MakeCallback(&PositionObservationApp::Observe, this));
}

/**
 * @ingroup defiance
 * Reward application that observes the position of the node it is installed on.
 * The distance to the origin of coordinates is sent as a reward to all registered
 * agent applications.
 */
class PositionRewardApp : public RewardApplication
{
  public:
    PositionRewardApp() {};
    ~PositionRewardApp() override {};
    static TypeId GetTypeId();
    Ptr<OpenGymDictContainer> CreateDictContainer(std::vector<float> reward);
    void Reward(Ptr<const MobilityModel> observation);
    void RegisterCallbacks() override;
};

NS_OBJECT_ENSURE_REGISTERED(PositionRewardApp);

TypeId
PositionRewardApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::CallbackRewardApp")
                            .SetParent<RewardApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PositionRewardApp>();
    return tid;
}

Ptr<OpenGymDictContainer>
PositionRewardApp::CreateDictContainer(std::vector<float> reward)
{
    auto box = MakeBoxContainer<float>(3);
    for (auto val : reward)
    {
        box->AddValue(val);
    }

    auto dictContainer = CreateObject<OpenGymDictContainer>();
    dictContainer->Add("distance", box);
    return dictContainer;
}

void
PositionRewardApp::Reward(Ptr<const MobilityModel> observation)
{
    std::vector<float> obs;
    obs.push_back(observation->GetPosition().x);
    obs.push_back(observation->GetPosition().y);
    obs.push_back(observation->GetPosition().z);
    // reward gets calculated by a function over the length of the vector
    float length = std::sqrt(std::pow(obs[0], 2) + std::pow(obs[1], 2) + std::pow(obs[2], 2));
    std::vector<float> reward = {length};
    Send(CreateDictContainer(reward), 0);
}

void
PositionRewardApp::RegisterCallbacks()
{
    Config::ConnectWithoutContext("/NodeList/*/$ns3::MobilityModel/CourseChange",
                                  MakeCallback(&PositionRewardApp::Reward, this));
}

/**
 * @ingroup defiance
 * Agent application that receives positions from observation and reward applications
 * and outputs some aggregated statistics about them.
 */
class PositionAgentApp : public AgentApplication
{
  public:
    PositionAgentApp() {};

    ~PositionAgentApp() override = default;
    static TypeId GetTypeId();

    void OnRecvObs(uint remoteAppId) override
    {
        NS_LOG_INFO("Received observation from observation interface " << remoteAppId << ":");
        NS_LOG_INFO(
            "\t Last x coordinate: " << m_obsDataStruct.GetNewestByID(remoteAppId)->data->Get("position-x")->GetObject<OpenGymBoxContainer<float>>()->GetValue(0));
        NS_LOG_INFO(
            "\t Average over 10 last y coordinates:" << m_obsDataStruct.AggregateNewest(remoteAppId, 10)["position-y"].GetAvg());
        NS_LOG_INFO(
            "\t Minimum of 10 last z coordinates: " << m_obsDataStruct.AggregateNewest(remoteAppId, 10)["position-z"].GetMin());
        NS_LOG_INFO(
            "\t Maximum of 10 last z coordinates: " << m_obsDataStruct.AggregateNewest(remoteAppId, 10)["position-z"].GetMax());
    }

    void OnRecvReward(uint remoteAppId) override
    {
        NS_LOG_INFO("Received reward from reward interface " << remoteAppId << ":");
        NS_LOG_INFO(
            "\t Last distance: " << m_rewardDataStruct.AggregateNewest(remoteAppId)["distance"].GetAvg());
    }

    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymSpace> GetActionSpace() override;
};

TypeId
PositionAgentApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PositionAgentApp")
                            .SetParent<AgentApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PositionAgentApp>();
    return tid;
}

Ptr<OpenGymSpace>
PositionAgentApp::GetObservationSpace()
{
    return {};
}

Ptr<OpenGymSpace>
PositionAgentApp::GetActionSpace()
{
    return {};
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("ApplicationCommunicationExample", LOG_LEVEL_INFO);

    uint32_t seed = 1;
    uint32_t runId = 0;
    uint32_t parallel = 0;
    std::string trialName = "";

    CommandLine cmd(__FILE__);
    cmd.AddValue("seed", "Seed for random number generator", seed);
    cmd.AddValue("runId", "Run ID. Is increased for every reset of the environment", runId);
    cmd.AddValue("parallel",
                 "Parallel ID. When running multiple environments in parallel, this is the index.",
                 parallel);
    cmd.AddValue("trial_name", "name of the trial", trialName);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed + parallel);
    RngSeedManager::SetRun(runId);
    Ns3AiMsgInterface::Get()->SetTrialName(trialName);

    // Set up nodes with mobility model
    NodeContainer observationNodes;
    observationNodes.Create(2);
    auto agentNode = CreateObject<Node>();
    NodeContainer rewardNodes;
    rewardNodes.Create(1);

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                  "X",
                                  StringValue("100.0"),
                                  "Y",
                                  StringValue("100.0"),
                                  "Rho",
                                  StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Mode",
                              StringValue("Time"),
                              "Time",
                              StringValue("2s"),
                              "Speed",
                              StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "Bounds",
                              StringValue("0|200|0|200"));
    mobility.InstallAll();

    // Create and install observation apps
    RlApplicationHelper helper(TypeId::LookupByName("ns3::TestObservation"));
    helper.SetAttribute("StartTime", TimeValue(Seconds(0)));
    helper.SetAttribute("StopTime", TimeValue(Seconds(10)));
    RlApplicationContainer observationApps = helper.Install(observationNodes);

    // Create and install reward app
    helper.SetTypeId("ns3::CallbackRewardApp");
    RlApplicationContainer rewardApps = helper.Install(rewardNodes);

    // Create and install agent app
    auto agentApp = CreateObjectWithAttributes<PositionAgentApp>("MaxObservationHistoryLength",
                                                          UintegerValue(10),
                                                          "MaxRewardHistoryLength",
                                                          UintegerValue(5));
    agentNode->AddApplication(agentApp);

    // Create a CommunicationHelper and use it to assign IDs to the apps
    CommunicationHelper commHelper = CommunicationHelper();
    commHelper.SetObservationApps(observationApps);
    commHelper.SetAgentApps(RlApplicationContainer(agentApp));
    commHelper.SetRewardApps(rewardApps);
    commHelper.SetIds();

    // Install the internet on the first observation node and the agent node
    NodeContainer internetNodes{observationNodes.Get(0), agentNode};
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    auto internetDevices = p2p.Install(internetNodes);
    InternetStackHelper internet;
    internet.Install(internetNodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(internetDevices);

    commHelper.AddCommunication(
        {
            // Connect the first observation app and the agent app via a socket channel interface
            {observationApps.GetId(0),
          agentApp->GetId(),
          SocketCommunicationAttributes{interfaces.GetAddress(0), interfaces.GetAddress(1)}},
            // Connect the second observation app and the agent app via a simple channel interface
          {observationApps.GetId(1), agentApp->GetId(), {}},
          // Connect the reward app and the agent app via a simple channel interface
         {rewardApps.GetId(0), agentApp->GetId(), {}}
        }
    );

    commHelper.Configure();
    Simulator::Stop(Seconds(10));
    Simulator::Run();
    Simulator::Destroy();
    NS_LOG_INFO("Simulation ended");

    return 0;
}
