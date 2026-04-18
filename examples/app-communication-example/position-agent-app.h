#ifndef NS3_POSITION_AGENT_APP_H
#define NS3_POSITION_AGENT_APP_H

#include "ns3/agent-application.h"

namespace ns3
{

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

    void OnRecvObs(uint remoteAppId) override;
    void OnRecvReward(uint remoteAppId) override;

    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymSpace> GetActionSpace() override;
};

} // namespace ns3
#endif // NS3_POSITION_AGENT_APP_H