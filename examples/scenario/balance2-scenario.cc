#include "environment-creator.h"
#include "pendulum-cart/pendulum-cart.h"

#include <ns3/action-application.h>
#include <ns3/agent-application.h>
#include <ns3/base-test.h>
#include <ns3/defiance-module.h>
#include <ns3/mobility-module.h>
#include <ns3/netanim-module.h>
#include <ns3/observation-application.h>
#include <ns3/reward-application.h>

#include <math.h>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BalanceScenario2");

void
SaveStats(Ptr<OutputStreamWrapper> stats_file,
          uint32_t nodeId,
          double cartMass,
          double pendulumMass,
          double pendulumLength,
          double angle,
          double angleVelocity,
          Vector position,
          double velocity,
          double acceleration,
          double nextAcceleration)
{
    std::ostream* stream = stats_file->GetStream();
    *stream << nodeId << "," << pendulumLength << "," << angle << "," << position.x << std::endl;
    stream->flush();
}

class PendulumObservationApp : public ObservationApplication
{
  public:
    PendulumObservationApp(){};
    ~PendulumObservationApp() override{};
    static TypeId GetTypeId();
    void Observe(uint32_t nodeId,
                 double cartMass,
                 double pendulumMass,
                 double pendulumLength,
                 double angle,
                 double angleVelocity,
                 Vector position,
                 double velocity,
                 double acceleration,
                 double nextAcceleration);
    void RegisterCallbacks() override;
    void SendObservation(double delay);

  private:
    Ptr<OpenGymDataContainer> m_observation = MakeBoxContainer<float>(4, 0, 0, 0, 0);
};

NS_OBJECT_ENSURE_REGISTERED(PendulumObservationApp);

TypeId
PendulumObservationApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PendulumObservationApp")
                            .SetParent<ObservationApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PendulumObservationApp>();
    return tid;
}

void
PendulumObservationApp::Observe(uint32_t nodeId,
                                double cartMass,
                                double pendulumMass,
                                double pendulumLength,
                                double angle,
                                double angleVelocity,
                                Vector position,
                                double velocity,
                                double acceleration,
                                double nextAcceleration)
{
    m_observation = MakeBoxContainer<float>(4, position.x, velocity, angle, angleVelocity);

    if (std::abs(angle) > 0.418 || std::abs(position.x) > 4.8)
    {
        auto terminateTime = Simulator::Now().GetSeconds();
        std::map<std::string, std::string> terminateInfo = {
            {std::string{"terminateTime"}, std::to_string(terminateTime)}};
        for (const auto& [agentId, agentInterface] : m_interfaces)
        {
            std::string agentIdString = "agent_" + std::to_string(agentId);
            NS_LOG_WARN("Terminate agent " << agentIdString << "! Time: " << terminateTime
                                           << " | angle: " << angle
                                           << " | position: " << position.x);
            OpenGymMultiAgentInterface::Get()->NotifyCurrentState(agentIdString,
                                                                  m_observation,
                                                                  -1,
                                                                  true,
                                                                  terminateInfo,
                                                                  Seconds(0),
                                                                  noopCallback);
        }
    }
}

void
PendulumObservationApp::SendObservation(double delay)
{
    Send(MakeDictContainer("floatObs", m_observation));
    Simulator::Schedule(Seconds(delay), &PendulumObservationApp::SendObservation, this, delay);
}

void
PendulumObservationApp::RegisterCallbacks()
{
    DynamicCast<PendulumCart>(GetNode())->m_reportCarStatsTrace.ConnectWithoutContext(
        MakeCallback(&PendulumObservationApp::Observe, this));
}

class PendulumRewardApp : public RewardApplication
{
  public:
    PendulumRewardApp(){};
    ~PendulumRewardApp() override{};
    static TypeId GetTypeId();
    void Reward(uint32_t nodeId,
                double cartMass,
                double pendulumMass,
                double pendulumLength,
                double angle,
                double angleVelocity,
                Vector position,
                double velocity,
                double acceleration,
                double nextAcceleration);
    void RegisterCallbacks() override;
};

NS_OBJECT_ENSURE_REGISTERED(PendulumRewardApp);

TypeId
PendulumRewardApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::PendulumRewardApp")
                            .SetParent<ObservationApplication>()
                            .SetGroupName("defiance")
                            .AddConstructor<PendulumRewardApp>();
    return tid;
}

void
PendulumRewardApp::Reward(uint32_t nodeId,
                          double cartMass,
                          double pendulumMass,
                          double pendulumLength,
                          double angle,
                          double angleVelocity,
                          Vector position,
                          double velocity,
                          double acceleration,
                          double nextAcceleration)
{
    if (angle > 0.418 || angle < -0.418)
    {
        Send(MakeDictBoxContainer<float>(1, "reward", 0));
    }
    else
    {
        Send(MakeDictBoxContainer<float>(1, "reward", 1));
    }
}

void
PendulumRewardApp::RegisterCallbacks()
{
    DynamicCast<PendulumCart>(GetNode())->m_reportCarStatsTrace.ConnectWithoutContext(
        MakeCallback(&PendulumRewardApp::Reward, this));
}

class InferenceAgentApp : public AgentApplication
{
  public:
    InferenceAgentApp()
        : AgentApplication(){

          };

    ~InferenceAgentApp() override{};

    Time m_stepTime;

    void PerformInferenceStep()
    {
        for (const auto& [appId, interface] : m_observationInterfaces)
        {
            // use observation for current cart
            if (m_obsDataStruct.HistoryExists(appId))
            {
                m_observation = m_obsDataStruct.GetNewestByID(appId)
                                    ->data->Get("floatObs")
                                    ->GetObject<OpenGymBoxContainer<float>>();
            }
            else
            {
                m_observation = MakeBoxContainer<float>(4, 0, 0, 0, 0);
            }
            // use reward for current cart
            if (m_rewardDataStruct.HistoryExists(appId))
            {
                m_reward = m_rewardDataStruct.GetNewestByID(appId)
                               ->data->Get("reward")
                               ->GetObject<OpenGymBoxContainer<float>>()
                               ->GetValue(0);
            }
            else
            {
                m_reward = 0;
            }
            // set id to sent action to corresponding ActionApp
            InferAction(appId);
        }
        Simulator::Schedule(m_stepTime, &InferenceAgentApp::PerformInferenceStep, this);
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::InferenceAgentApp")
                                .SetParent<AgentApplication>()
                                .SetGroupName("defiance")
                                .AddConstructor<InferenceAgentApp>()
                                .AddAttribute("StepTime",
                                              "the step time delay between inference steps",
                                              TimeValue(MilliSeconds(10)),
                                              MakeTimeAccessor(&InferenceAgentApp::m_stepTime),
                                              MakeTimeChecker());
        return tid;
    }

    void Setup() override
    {
        AgentApplication::Setup();
        m_observation = GetResetObservation();
        m_reward = GetResetReward();
    }

    void OnRecvObs(uint id) override
    {
        auto observation = m_obsDataStruct.GetNewestByID(id)
                               ->data->Get("floatObs")
                               ->GetObject<OpenGymBoxContainer<float>>();
    }

    void OnRecvReward(uint id) override
    {
    }

    Ptr<OpenGymDataContainer> GetResetObservation()
    {
        // This method returns the initial observation that is used after resetting the environment.
        return MakeBoxContainer<float>(4, 0, 0, 0, 0);
    }

    float GetResetReward()
    {
        // This method returns the initial reward that is used after resetting the environment.
        return 0.0;
    }

  private:
    Ptr<OpenGymSpace> GetObservationSpace() override
    {
        return MakeBoxSpace<float>(4,
                                   {-4.8 * 2, -INFINITY, -0.418 * 2, -INFINITY},
                                   {4.8 * 2, INFINITY, 0.418 * 2, INFINITY});
    }

    Ptr<OpenGymSpace> GetActionSpace() override
    {
        return MakeBoxSpace<int>(1, 0, 1);
    }
};

class PendulumActionApp : public ActionApplication
{
  public:
    PendulumActionApp(){};
    ~PendulumActionApp() override{};

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::PendulumActionApp")
                                .SetParent<ActionApplication>()
                                .SetGroupName("defiance")
                                .AddConstructor<PendulumActionApp>();
        return tid;
    }

    void ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action) override
    {
        auto cart = DynamicCast<PendulumCart>(GetNode());
        auto act = action->Get("default")->GetObject<OpenGymBoxContainer<int>>();
        auto acc = 20;
        acc *= (act->GetValue(0));
        acc -= 10;

        cart->SetAcceleration(acc);
    }

    void SetObservationApp(Ptr<PendulumObservationApp> observationApp)
    {
        m_observationApp = observationApp;
    }

  private:
    Ptr<PendulumObservationApp> m_observationApp;
};

NS_OBJECT_ENSURE_REGISTERED(PendulumActionApp);
NS_OBJECT_ENSURE_REGISTERED(InferenceAgentApp);

int
main(int argc, char* argv[])
{
    LogComponentEnable("BalanceScenario2", LOG_LEVEL_ALL);

    int offset = 1;

    // first of all we need to parse the command line arguments
    std::string interfaceType = "SIMPLE"; // use simple channel interface per default
    bool visualize = false;

    // multiple agents and carts are only supported with SimpleChannelInterfaces
    uint32_t seed = 1;
    uint32_t runId = 1;
    uint32_t parallel = 0;
    uint agentNum = 1;
    uint cartsPerAgent = 1;
    std::string trialName = "";

    CommandLine cmd(__FILE__);
    cmd.AddValue("trial_name", "name of the trial", trialName);
    cmd.AddValue("seed", "Seed to create comparable scenarios", seed);
    cmd.AddValue("runId", "Run ID. Is increased for every reset of the environment", runId);
    cmd.AddValue("parallel",
                 "Parallel ID. When running multiple environments in parallel, this is the index.",
                 parallel);
    cmd.AddValue("interfaceType", "The type of the channel interface to use", interfaceType);
    cmd.AddValue("numberOfAgents",
                 "The number of agents and base stations used in the simulation",
                 agentNum);
    cmd.AddValue("cartsPerAgent", "The number of carts per agent or base station", cartsPerAgent);
    cmd.AddValue("visualize", "Log visualization traces", visualize);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed + parallel);
    RngSeedManager::SetRun(runId);
    Ns3AiMsgInterface::Get()->SetTrialName(trialName);

    if (interfaceType != "SIMPLE" && (agentNum != 1 || cartsPerAgent != 1))
    {
        NS_FATAL_ERROR("numberOfAgents and cartsPerAgent are currently only supported with "
                       "SimpleChannelInterface");
    }

    auto environmentCreator = EnvironmentCreator();
    environmentCreator.SetupPendulumScenario(agentNum * cartsPerAgent, agentNum, false);

    auto cartNodes = environmentCreator.GetCartNodes();
    auto enbNodes = environmentCreator.GetEnbNodes();
    auto inferenceAgentNodes = environmentCreator.GetInferenceAgentNodes();

    RlApplicationHelper helper(TypeId::LookupByName("ns3::PendulumObservationApp"));
    helper.SetAttribute("StartTime", TimeValue(Seconds(0)));
    helper.SetAttribute("StopTime", TimeValue(Seconds(offset + 10)));
    RlApplicationContainer observationApps = helper.Install(cartNodes);

    helper.SetTypeId("ns3::PendulumRewardApp");
    RlApplicationContainer rewardApps = helper.Install(cartNodes);

    helper.SetTypeId("ns3::PendulumActionApp");
    RlApplicationContainer actionApps = helper.Install(cartNodes);

    helper.SetTypeId("ns3::InferenceAgentApp");
    RlApplicationContainer agentApps = helper.Install(inferenceAgentNodes);

    CommunicationHelper commHelper = CommunicationHelper();

    commHelper.SetObservationApps(observationApps);
    commHelper.SetAgentApps(agentApps);
    commHelper.SetRewardApps(rewardApps);
    commHelper.SetActionApps(actionApps);
    commHelper.SetIds();

    std::vector<CommunicationPair> adjacency = {};

    // add communications depending on what run type is chosen
    if (interfaceType == "SIMPLE")
    {
        for (uint i = 0; i < inferenceAgentNodes.GetN(); i++)
        {
            for (uint j = 0; j < cartsPerAgent; j++)
            {
                uint cartId = i * cartsPerAgent + j;
                CommunicationPair observationCommPair = {observationApps.GetId(cartId),
                                                         agentApps.GetId(i),
                                                         {}};
                CommunicationPair rewardCommPair = {rewardApps.GetId(cartId),
                                                    agentApps.GetId(i),
                                                    {}};
                CommunicationPair actionCommPair = {actionApps.GetId(cartId),
                                                    agentApps.GetId(i),
                                                    {}};
                adjacency.emplace_back(observationCommPair);
                adjacency.emplace_back(rewardCommPair);
                adjacency.emplace_back(actionCommPair);
            }
        }
    }
    if (interfaceType == "UDP")
    {
        CommunicationPair observationCommPair = {
            observationApps.GetId(0),
            agentApps.GetId(0),
            SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", UdpSocketFactory::GetTypeId()}};
        CommunicationPair rewardCommPair = {rewardApps.GetId(0), agentApps.GetId(0), {}};
        CommunicationPair actionCommPair = {
            actionApps.GetId(0),
            agentApps.GetId(0),
            SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", UdpSocketFactory::GetTypeId()}};
        adjacency.emplace_back(observationCommPair);
        adjacency.emplace_back(rewardCommPair);
        adjacency.emplace_back(actionCommPair);
    }
    if (interfaceType == "TCP")
    {
        CommunicationPair observationCommPair = {
            observationApps.GetId(0),
            agentApps.GetId(0),
            SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", TcpSocketFactory::GetTypeId()}};
        CommunicationPair rewardCommPair = {rewardApps.GetId(0), agentApps.GetId(0), {}};
        CommunicationPair actionCommPair = {
            actionApps.GetId(0),
            agentApps.GetId(0),
            SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", TcpSocketFactory::GetTypeId()}};
        adjacency.emplace_back(observationCommPair);
        adjacency.emplace_back(rewardCommPair);
        adjacency.emplace_back(actionCommPair);
    }

    commHelper.AddCommunication(adjacency);
    commHelper.Configure();

    for (uint i = 0; i < cartNodes.GetN(); i++)
    {
        DynamicCast<PendulumActionApp>(actionApps.Get(i))
            ->SetObservationApp(DynamicCast<PendulumObservationApp>(observationApps.Get(i)));
    }

    for (uint i = 0; i < observationApps.GetN(); i++)
    {
        Simulator::Schedule(Seconds(offset),
                            &PendulumObservationApp::SendObservation,
                            DynamicCast<PendulumObservationApp>(observationApps.Get(i)),
                            0.01);
    }

    for (uint i = 0; i < agentApps.GetN(); i++)
    {
        Simulator::Schedule(Seconds(offset + 0.001),
                            &InferenceAgentApp::PerformInferenceStep,
                            DynamicCast<InferenceAgentApp>(agentApps.Get(i)));
    }

    std::string pathToNs3 = std::getenv("NS3_HOME");
    std::ofstream stats_file(pathToNs3 + "/stats.csv", std::ios_base::app);
    if (visualize)
    {
        Ptr<OutputStreamWrapper> stats_file_ptr = Create<OutputStreamWrapper>(&stats_file);
        for (uint i = 0; i < cartNodes.GetN(); i++)
        {
            DynamicCast<PendulumCart>(cartNodes.Get(i))
                ->m_reportCarStatsTrace.ConnectWithoutContext(
                    MakeBoundCallback(&SaveStats, stats_file_ptr));
        }
    }
    for (auto it = agentApps.Begin(); it != agentApps.End(); it++)
    {
        Simulator::Schedule(Seconds(5 + offset),
                            MakeCallback(&AgentApplication::RequestTruncation,
                                         PeekPointer((*it)->GetObject<AgentApplication>())));
    }

    Simulator::Run();

    return 0;
}
