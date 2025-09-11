#ifndef DEFIANCE_SUMO_ENVIRONMENT_H
#define DEFIANCE_SUMO_ENVIRONMENT_H

#include "static-environment.h"

namespace ns3
{

/**
 * \ingroup defiance
 * \class SumoMobComEnv
 * \brief A MobComEnv with integration with SUMO - Simulation of Urban MObility
 */
class SumoMobComEnv : public StaticMobComEnv
{
    /**
     * \brief Create the topology for the simulation. One base station is placed at (100, 320, 31)
     * and two user equipments will randomly walk along the streets of Babelsberg.
     */
    void CreateTopology() override;
};
} // namespace ns3

#endif /* DEFIANCE__SUMO_ENVIRONMENT_H */
