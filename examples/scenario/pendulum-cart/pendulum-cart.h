#ifndef INVERTED_PENDULUM_H
#define INVERTED_PENDULUM_H

#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/mobility-helper.h>
#include <ns3/mobility-model.h>
#include <ns3/node.h>

#include <cstdint>

namespace ns3
{

/**
 * \ingroup defiance
 * \class PendulumCart
 * \brief Node that simulates the behaviour and physics of an inverted pendulum cart and can be used
 * like any other ns3 node.
 */
class PendulumCart : public Node
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    PendulumCart();
    PendulumCart(double xPos);

    ~PendulumCart() override;
    void DoDispose() override;

    /**
     * \brief Set the angle of the cart in radians: ensures that the angle stays between -pi and pi.
     * \param angle the angle of the cart in radians
     */
    void SetAngle(double angle);

    /**
     * \brief Set the angular velocity of the cart.
     * \param angleVelocity the angular velocity of the cart in rad/s
     */
    void SetAngleVelocity(double angleVelocity);

    /**
     * \brief Sets the acceleration of the cart in m/s² used in the next time step of the physics
     * simulation.
     */
    void SetAcceleration(double acceleration);

    /**
     * \brief Simulates the physics of the pendulum-cart by using the equations of motion of an
     * inverted pendulum. The equations of motion are based on
     * https://en.wikipedia.org/wiki/Inverted_pendulum
     * \param deltaT the time step in seconds
     * between recalculating the physics of the cart
     */
    void SimulateTimeTic(double deltaT);

    /**
     * \brief The `ReportCartStats` trace source. Exporting node ID, cartMass, pendulumMass,
     * pendulumLength, angle, angleVelocity, position, velocity, acceleration, nextAcceleration.
     */
    TracedCallback<uint32_t, double, double, double, double, double, Vector, double, double, double>
        m_reportCarStatsTrace;

    /**
     * \brief TracedCallback signature for car stats report.
     *
     * \param [in] nodeId
     * \param [in] cartMass
     * \param [in] pendulumMass
     * \param [in] pendulumLength
     * \param [in] angle
     * \param [in] angleVelocity
     * \param [in] position
     * \param [in] velocity
     * \param [in] acceleration
     * \param [in] nextAcceleration
     */
    typedef void (*RsrpSinrTracedCallback)(uint32_t nodeId,
                                           double cartMass,
                                           double pendulumMass,
                                           double pendulumLength,
                                           double angle,
                                           double angleVelocity,
                                           Vector position,
                                           double velocity,
                                           double acceleration,
                                           double nextAcceleration);

  protected:
    double m_cartMass;         ///< mass of the cart in kg
    double m_pendulumMass;     ///< mass of the pendulum in kg
    double m_pendulumLength;   ///< length of the pendulum in m
    double m_angle;            ///< angle of the pendulum in radians
    double m_angleVelocity;    ///< angular velocity of the pendulum in rad/s
    double m_velocity;         ///< velocity of the cart in m/s
    double m_acceleration;     ///< acceleration of the cart in m/s^2
    double m_nextAcceleration; ///< acceleration that is applied in the next timestep to
                               ///< synchronize physics of the cart in m/s^2
};

} // namespace ns3

#endif /* INVERTED_PENDULUM_H */
