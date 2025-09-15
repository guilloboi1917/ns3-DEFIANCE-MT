#include "environment-creator.h"

#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/socket-channel-interface.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EnvironmentCreatorExample");

// Helper method to create a simple message easily.
Ptr<OpenGymDictContainer>
CreateTestMessage(float value)
{
    auto msg = CreateObject<OpenGymDictContainer>();
    auto box = CreateObject<OpenGymBoxContainer<float>>();
    box->AddValue(value);
    msg->Add("box", box);
    return msg;
}

int
main(int argc, char* argv[])
{
    auto env = EnvironmentCreator();
    env.SetupPendulumScenario(7, 4, true);

    auto tcpProtocol = TcpSocketFactory::GetTypeId();

    auto cartInterface =
        CreateObject<SocketChannelInterface>(env.GetCartNodes().Get(0), "7.0.0.2", tcpProtocol);
    auto inferenceAgentAInterface =
        CreateObject<SocketChannelInterface>(env.GetInferenceAgentNodes().Get(0),
                                             "1.0.0.2",
                                             tcpProtocol);
    auto inferenceAgentBInterface =
        CreateObject<SocketChannelInterface>(env.GetInferenceAgentNodes().Get(1),
                                             "7.0.0.3",
                                             tcpProtocol);
    auto trainingAgentInterface =
        CreateObject<SocketChannelInterface>(env.GetTrainingAgentNodes().Get(0),
                                             "2.0.0.2",
                                             tcpProtocol);

    auto recvCallback = Callback<void, Ptr<OpenGymDictContainer>>(
        [](Ptr<OpenGymDictContainer> msg) { std::cout << (msg->Get("box")) << std::endl; });
    cartInterface->AddRecvCallback(recvCallback);
    inferenceAgentAInterface->AddRecvCallback(recvCallback);
    inferenceAgentBInterface->AddRecvCallback(recvCallback);
    trainingAgentInterface->AddRecvCallback(recvCallback);

    Simulator::Schedule(Seconds(0.5),
                        &SocketChannelInterface::Connect,
                        cartInterface,
                        inferenceAgentAInterface);

    Simulator::Schedule(Seconds(0.5),
                        &SocketChannelInterface::Connect,
                        inferenceAgentBInterface,
                        trainingAgentInterface);

    Simulator::Schedule(Seconds(1),
                        &SocketChannelInterface::Send,
                        cartInterface,
                        CreateTestMessage(6));

    Simulator::Schedule(Seconds(2),
                        &SocketChannelInterface::Send,
                        cartInterface,
                        CreateTestMessage(9));

    // Trace routing tables
    Ptr<OutputStreamWrapper> routingStream =
        Create<OutputStreamWrapper>("dynamic-global-routing.routes", std::ios::out);
    Ipv4GlobalRoutingHelper::PrintRoutingTableAllAt(Seconds(2.5), routingStream);

    Simulator::Stop(Seconds(3.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
