#ifndef NS3_ADDITION_AGENT_APP_H
#define NS3_ADDITION_AGENT_APP_H

#include "ns3/agent-application.h"

namespace ns3
{

class AdditionAgentApp : public AgentApplication
{
  public:
    AdditionAgentApp();
    static TypeId GetTypeId();

  protected:
    void OnRecvObs(uint id) override;
    void OnRecvReward(uint remoteAppId) override;

  private:
    /// the last action of the agent is buffered for the reward calculation.
    int m_last_action;
    int m_last_observation;

    Ptr<OpenGymSpace> GetObservationSpace() override;

    Ptr<OpenGymSpace> GetActionSpace() override;

    void InitiateAction(Ptr<OpenGymDataContainer> action) override;
};
} // namespace ns3
#endif // NS3_ADDITION_AGENT_APP_H
