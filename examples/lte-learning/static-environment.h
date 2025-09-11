#ifndef STATIC_ENVIRONMENT_H
#define STATIC_ENVIRONMENT_H

#include "base-environment.h"

#include <cstdint>

namespace ns3
{

/**
 * \ingroup defiance
 * \class StaticMobComEnv
 * \brief MobComEnv with static eNodeBs and UEs
 */
class StaticMobComEnv : public MobComEnv
{
  public:
    /**
     * \brief Create the topology for the simulation.
     */
    void CreateTopology() override;

    /**
     * \brief Add internet traffic from base stations to user equipments.
     */
    void AddTraffic() override;

  private:
    void CreateStaticTopology(uint16_t numUe, uint16_t numBs, uint64_t maxDistance);

    /**
     * \brief Create a line of base stations along the x-axis.
     * \param numBs the number of base stations.
     * \param lineLength the length of the line base stations are placed along.
     */
    void CreateBsLine(uint16_t numBs, uint64_t lineLength);

    /**
     * \brief Create a test topology with a line of base stations along the x-axis. UEs are placed
     * randomly within a maximum distance from the center of the line of \c lineLength/2 in either
     * coordinate.
     * \param numUe the number of user equipments.
     * \param numBs the number of base stations.
     * \param lineLength the length of the line base stations are placed along.
     */
    void CreateRandomFixedTopology(uint16_t numUe, uint16_t numB, uint64_t lineLength);

    /**
     * \brief Create a test topology with one base station at (0, 0, 1) and one moving user
     * equipment.
     * \param position the initial position of the user equipment.
     * \param velocity The velocity of the user equipment.
     */
    void CreateTestTopology1(Vector position, Vector velocity);

    /**
     * \brief Create a test topology with two base stations at (0, 0, 1) and (100, 0, 1) and two
     * stationary user equipments at (0, 500, 1) and (1000, 500, 1).
     */
    void CreateTestTopology2();
};

} // namespace ns3
#endif /* STATIC_ENVIRONMENT_H */
