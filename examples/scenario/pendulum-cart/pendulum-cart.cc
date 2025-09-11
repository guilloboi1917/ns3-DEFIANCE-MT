#include "pendulum-cart.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("PendulumCart");

NS_OBJECT_ENSURE_REGISTERED(PendulumCart);

TypeId
PendulumCart::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::PendulumCart")
            .SetParent<Node>()
            .AddConstructor<PendulumCart>()
            .AddAttribute("CartMass",
                          "The mass of the cart",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&PendulumCart::m_cartMass),
                          MakeDoubleChecker<double>())
            .AddAttribute("PendulumMass",
                          "The mass of the pendulum",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&PendulumCart::m_pendulumMass),
                          MakeDoubleChecker<double>())
            .AddAttribute("PendulumLength",
                          "The length of the pendulum",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&PendulumCart::m_pendulumLength),
                          MakeDoubleChecker<double>())
            .AddAttribute("Angle",
                          "The angle of the pendulum",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&PendulumCart::m_angle),
                          MakeDoubleChecker<double>())
            .AddAttribute("AngleVelocity",
                          "The velocity of the pendulum",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PendulumCart::m_angleVelocity),
                          MakeDoubleChecker<double>())
            .AddAttribute("Velocity",
                          "The velocity of the cart",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PendulumCart::m_velocity),
                          MakeDoubleChecker<double>())
            .AddAttribute("Acceleration",
                          "The acceleration of the cart",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PendulumCart::m_acceleration),
                          MakeDoubleChecker<double>())
            .AddAttribute("NextAcceleration",
                          "The next acceleration of the cart",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PendulumCart::m_nextAcceleration),
                          MakeDoubleChecker<double>())
            .AddTraceSource(
                "ReportCarStats",
                "Reports all the physical properties of the car after one timestep update",
                MakeTraceSourceAccessor(&PendulumCart::m_reportCarStatsTrace),
                "ns3::PendulumCart::ReportCarStatsCallback");

    return tid;
}

PendulumCart::PendulumCart(double xPos)
    : Node()
{
    NS_LOG_FUNCTION(this);

    MobilityHelper mobility;

    auto positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(xPos, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(this);
}

PendulumCart::PendulumCart()
    : Node()
{
    NS_LOG_FUNCTION(this);

    MobilityHelper mobility;

    auto positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(this);
}

PendulumCart::~PendulumCart()
{
    NS_LOG_FUNCTION(this);
}

void
PendulumCart::DoDispose()
{
    NS_LOG_FUNCTION(this);
    Node::DoDispose();
}

void
PendulumCart::SetAngle(double angle)
{
    NS_LOG_FUNCTION(this << angle);
    // ensure the angle is between -pi and pi
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    m_angle = angle;
}

void
PendulumCart::SetAngleVelocity(double angleVelocity)
{
    NS_LOG_FUNCTION(this << angleVelocity);
    m_angleVelocity = angleVelocity;
}

void
PendulumCart::SetAcceleration(double acceleration)
{
    NS_LOG_FUNCTION(this << acceleration);
    m_nextAcceleration = acceleration;
}

void
PendulumCart::SimulateTimeTic(double deltaT)
{
    NS_LOG_FUNCTION(this << deltaT);
    // Simulate the physics of the pendulum-cart
    // based on https://en.wikipedia.org/wiki/Inverted_pendulum
    // 1. (M + m) x'' - ml theta'' cos(theta) + ml theta'^2 sin(theta) = F
    // 2. l theta'' - x'' cos(theta) - g sin(theta) = 0

    // F = (M + m) a
    // 1. (M + m) x'' - ml theta'' cos(theta) + ml theta'^2 sin(theta) = (M + m) a
    // 2. l theta'' - x'' cos(theta) - g sin(theta) = 0

    // 1. x'' = (ml theta'' cos(theta) - ml theta'^2 sin(theta) + (M + m) a) / (M + m)
    // 2. theta'' = (x'' cos(theta) + g sin(theta)) / l

    Ptr<ConstantPositionMobilityModel> mobility = this->GetObject<ConstantPositionMobilityModel>();

    double g = 9.81; // m/s^2
    double thetaAcceleration =
        (this->m_acceleration * std::cos(this->m_angle) + g * std::sin(this->m_angle)) /
        this->m_pendulumLength;
    this->m_angleVelocity += thetaAcceleration * deltaT;

    this->SetAngle(this->m_angle + this->m_angleVelocity * deltaT);

    double cartAcceleration =
        (this->m_pendulumMass * this->m_pendulumLength * thetaAcceleration *
             std::cos(this->m_angle) -
         this->m_pendulumMass * this->m_pendulumLength * this->m_angleVelocity *
             this->m_angleVelocity * std::sin(this->m_angle) +
         (this->m_pendulumMass + this->m_cartMass) * this->m_acceleration) /
        (this->m_pendulumMass + this->m_cartMass);

    this->m_velocity += cartAcceleration * deltaT;
    mobility->SetPosition(mobility->GetPosition() + Vector(this->m_velocity * deltaT, 0, 0));
    this->m_acceleration = this->m_nextAcceleration;

    // finally we update the trace source so that we have access to the values once they have
    // changed
    m_reportCarStatsTrace(GetId(),
                          this->m_cartMass,
                          this->m_pendulumMass,
                          this->m_pendulumLength,
                          this->m_angle,
                          this->m_angleVelocity,
                          mobility->GetPosition(),
                          this->m_velocity,
                          this->m_acceleration,
                          this->m_nextAcceleration);

    // reschedule the next update
    Simulator::Schedule(Seconds(deltaT), &PendulumCart::SimulateTimeTic, this, deltaT);
}

} // namespace ns3
