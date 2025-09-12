#include "pendulum-cart.h"

#include "ns3/log.h"
#include "ns3/lte-helper.h"

#include <fstream>
#include <sstream>
#include <string>

using namespace ns3;

/*
 * This example demonstrates how to create a PendulumCart object and simulate it. It also includes
 * logging of the cart. The generated output can be visualized with the script visualize.ipynb.
 * You can run the script with logging enabled with the following command:
 * NS_LOG="PendulumCartExample" ./ns3 run defiance-pendulum
 * To set one of the carts attributes you can run for example:
 * NS_LOG="PendulumCartExample" ./ns3 run "defiance-pendulum --ns3::PendulumCart::CartMass=4.0"
 * The file to the output can be set with the OutFilePath attribute. (e.g. --OutFilePath=./log.txt)
 */

NS_LOG_COMPONENT_DEFINE("PendulumCartExample");

// an example of a callback function that can be used to print the stats of the cart

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
    std::stringstream ss;
    ss << nodeId << "," << cartMass << "," << pendulumMass << "," << pendulumLength << "," << angle
       << "," << angleVelocity << "," << position.x << "," << velocity << "," << acceleration << ","
       << nextAcceleration;
    std::ostream* stream = stats_file->GetStream();
    *stream << ss.str() << std::endl;
}

// an example of an event that can be used to set the acceleration of the cart to a
// random value and is peridoically called
void
SetRandomAcceleration(Ptr<PendulumCart> cart)
{
    // ns3 random variable in ns3 with values between -1 and 1
    auto randomVariable = CreateObject<UniformRandomVariable>();
    randomVariable->SetAttribute("Min", DoubleValue(-1.0));
    randomVariable->SetAttribute("Max", DoubleValue(1.0));

    double randomAcceleration = randomVariable->GetValue();

    cart->SetAcceleration(randomAcceleration);
    Simulator::Schedule(Seconds(0.01), &SetRandomAcceleration, cart);
}

int
main(int argc, char* argv[])
{
    // default path were the log_file will be stored
    std::string outFilePath =
        std::string(std::getenv("NS3_HOME")) + "/contrib/defiance/examples/pendulum-cart/log.txt";

    // the following also maps the command line arguments to attributes of the cart
    CommandLine cmd(__FILE__);
    cmd.AddValue("OutFilePath", "The path were the output file is saved.", outFilePath);
    cmd.Parse(argc, argv);

    std::ofstream stats_file(outFilePath);
    stats_file
        << "nodeId,cartMass,pendulumMass,pendulumLength,angle,angleVelocity,position,velocity,"
           "acceleration,nextAcceleration"
        << std::endl;

    Ptr<OutputStreamWrapper> stats_file_ptr = Create<OutputStreamWrapper>(&stats_file);

    // Other ways of creating a cart with specific attributes include :
    // Config::SetDefault("ns3::PendulumCart::CartMass", DoubleValue(5.0));
    // auto cart = CreateObjectWithAttributes<PendulumCart>("CartMass",
    // DoubleValue(5.0));

    auto cart = CreateObject<PendulumCart>();
    auto lteHelper = CreateObject<LteHelper>();
    NodeContainer ueNodes;
    ueNodes.Add(cart);
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(ueNodes);

    cart->m_reportCarStatsTrace.ConnectWithoutContext(
        MakeBoundCallback(&SaveStats, stats_file_ptr));
    Simulator::Schedule(Seconds(0.01), &PendulumCart::SimulateTimeTic, cart, 0.01);
    Simulator::Schedule(Seconds(0.1), &SetRandomAcceleration, cart);
    Simulator::Stop(Seconds(50));
    Simulator::Run();
    Simulator::Destroy();
}
