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
#include <ns3/rl-application-helper.h>

#include <cstdint>
#include <math.h>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BalanceScenario1");

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

  private:
    float m_positionX{0};
    float m_velocity{0};
    float m_angle{0};
    float m_angleVelocity{0};
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
    auto obs = MakeBoxContainer<float>(4, position.x, velocity, angle, angleVelocity);

    Send(MakeDictContainer("floatObs", obs));
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
        : AgentApplication()
    {
        m_reward = 0;
        m_observation = MakeBoxContainer<float>(4, 0, 0, 0, 0);
    };

    ~InferenceAgentApp() override{};

    Time m_stepTime;

    void PerformInferenceStep()
    {
        InferAction();
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
        // reward is calculated often, so we can assume that M_reward is the "current" reward
        // already
        auto obs = m_obsDataStruct.GetNewestByID(0)
                       ->data->Get("floatObs")
                       ->GetObject<OpenGymBoxContainer<float>>();
        m_observation = obs;
        float angle = obs->GetValue(3);
        if (std::abs(angle) > 0.418 || std::abs(angle) > 4.8)
        {
            auto terminateTime = Simulator::Now().GetSeconds();
            std::map<std::string, std::string> terminateInfo = {
                {std::string{"terminateTime"}, std::to_string(terminateTime)}};
            NS_LOG_WARN("Terminate! time: " << terminateTime << " angle: " << angle);
            OpenGymMultiAgentInterface::Get()->NotifyCurrentState("agent_0",
                                                                  obs,
                                                                  -1,
                                                                  true,
                                                                  terminateInfo,
                                                                  Seconds(0),
                                                                  noopCallback);
        }

        // check if the episode is terminated
    }

    void OnRecvReward(uint id) override
    {
        m_reward = m_rewardDataStruct.GetNewestByID(0)
                       ->data->Get("reward")
                       ->GetObject<OpenGymBoxContainer<float>>()
                       ->GetValue(0);
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
    NS_LOG_APPEND_TIME_PREFIX;
    // LogComponentEnable("SocketChannelInterface", ns3::LOG_LEVEL_ALL);
    // LogComponentEnable("EnvironmentCreator", ns3::LOG_LEVEL_ALL);
    // LogComponentEnable("Socket", ns3::LOG_LEVEL_ALL);

    // first of all we need to parse the command line arguments
    uint32_t seed = 1;
    uint32_t runId = 1;
    uint32_t parallel = 0;
    std::string interfaceType = "SIMPLE"; // use simple channel interface per default
    bool visualize = false;
    std::string trialName = "";

    CommandLine cmd(__FILE__);
    cmd.AddValue("trial_name", "name of the trial", trialName);
    cmd.AddValue("seed", "Seed to create comparable scenarios", seed);
    cmd.AddValue("runId", "Run ID. Is increased for every reset of the environment", runId);
    cmd.AddValue("parallel",
                 "Parallel ID. When running multiple environments in parallel, this is the index.",
                 parallel);
    cmd.AddValue("visualize", "Log visualization traces", visualize);
    cmd.AddValue("interfaceType", "The type of the channel interface to use", interfaceType);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed + parallel);
    RngSeedManager::SetRun(runId);
    Ns3AiMsgInterface::Get()->SetTrialName(trialName);

    LogComponentEnable("BalanceScenario1", LOG_LEVEL_ALL);
    auto environmentCreator = EnvironmentCreator();
    environmentCreator.SetupPendulumScenario(1, 1, false);

    auto cartNode = environmentCreator.GetCartNodes().Get(0);
    auto enbNode = environmentCreator.GetEnbNodes().Get(0);
    auto inferenceAgentNode = environmentCreator.GetInferenceAgentNodes().Get(0);

    RlApplicationHelper helper(TypeId::LookupByName("ns3::PendulumObservationApp"));
    helper.SetAttribute("StartTime", TimeValue(Seconds(0)));
    helper.SetAttribute("StopTime", TimeValue(Seconds(10)));
    RlApplicationContainer observationApps = helper.Install(cartNode);

    helper.SetTypeId("ns3::PendulumRewardApp");
    RlApplicationContainer rewardApps = helper.Install(cartNode);

    helper.SetTypeId("ns3::PendulumActionApp");
    RlApplicationContainer actionApps = helper.Install(cartNode);

    helper.SetTypeId("ns3::InferenceAgentApp");
    // Config::Set("InferenceAgentApp", const AttributeValue &value)
    RlApplicationContainer agentApps = helper.Install(inferenceAgentNode);

    CommunicationHelper commHelper = CommunicationHelper();

    commHelper.SetObservationApps(observationApps);
    commHelper.SetAgentApps(agentApps);
    commHelper.SetRewardApps(rewardApps);
    commHelper.SetActionApps(actionApps);
    commHelper.SetIds();

    if (interfaceType == "SIMPLE")
    {
        commHelper.AddCommunication(
            {{observationApps.GetId(0), agentApps.GetId(0), {}},
             {rewardApps.GetId(0), agentApps.GetId(0), CommunicationAttributes{Seconds(0)}},
             {agentApps.GetId(0), actionApps.GetId(0), {}}});
    }
    else if (interfaceType == "UDP")
    {
        commHelper.AddCommunication(
            {{observationApps.GetId(0),
              agentApps.GetId(0),
              SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", UdpSocketFactory::GetTypeId()}},
             {rewardApps.GetId(0), agentApps.GetId(0), CommunicationAttributes{Seconds(0)}},
             {agentApps.GetId(0),
              actionApps.GetId(0),
              SocketCommunicationAttributes{"1.0.0.2", "7.0.0.2", UdpSocketFactory::GetTypeId()}}});
    }
    else if (interfaceType == "TCP")
    {
        commHelper.AddCommunication(
            {{observationApps.GetId(0),
              agentApps.GetId(0),
              SocketCommunicationAttributes{"7.0.0.2", "1.0.0.2", TcpSocketFactory::GetTypeId()}},
             {rewardApps.GetId(0), agentApps.GetId(0), CommunicationAttributes{Seconds(0)}},
             {agentApps.GetId(0),
              actionApps.GetId(0),
              SocketCommunicationAttributes{"1.0.0.2", "7.0.0.2", TcpSocketFactory::GetTypeId()}}});
    }

    commHelper.Configure();

    DynamicCast<PendulumActionApp>(actionApps.Get(0))
        ->SetObservationApp(DynamicCast<PendulumObservationApp>(observationApps.Get(0)));

    // AnimationInterface anim("balance1-scenario-animation.xml");

    // anim.SetMobilityPollInterval(Seconds(0.25));
    // anim.SetMaxPktsPerTraceFile(100000);
    // anim.UpdateNodeDescription(cartNode, "cart");
    // anim.UpdateNodeColor(cartNode, 0, 0, 255);
    // anim.UpdateNodeDescription(enbNode, "eNB");
    // anim.UpdateNodeColor(enbNode, 0, 255, 0);
    // anim.UpdateNodeDescription(inferenceAgentNode, "inferenceAgent");
    // anim.UpdateNodeColor(inferenceAgentNode, 255, 255, 0);
    // anim.EnablePacketMetadata(true);

    Simulator::Schedule(Seconds(0),
                        &InferenceAgentApp::PerformInferenceStep,
                        DynamicCast<InferenceAgentApp>(agentApps.Get(0)));

    std::string pathToNs3 = std::getenv("NS3_HOME");
    std::ofstream stats_file(pathToNs3 + "/stats.csv", std::ios_base::app);
    if (visualize)
    {
        Ptr<OutputStreamWrapper> stats_file_ptr = Create<OutputStreamWrapper>(&stats_file);
        DynamicCast<PendulumCart>(cartNode)->m_reportCarStatsTrace.ConnectWithoutContext(
            MakeBoundCallback(&SaveStats, stats_file_ptr));
    }

    Simulator::Stop(Seconds(10));
    Simulator::Run();
    OpenGymMultiAgentInterface::Get()->NotifySimulationEnd();

    return 0;
}
