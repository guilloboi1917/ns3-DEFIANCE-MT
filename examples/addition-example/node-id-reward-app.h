#ifndef NS3_NODE_ID_REWARD_APP_H
#define NS3_NODE_ID_REWARD_APP_H

#include "ns3/observation-application.h"
#include "ns3/random-variable-stream.h"

namespace ns3
{

class NodeListRewardApp : public ObservationApplication
{
  public:
    NodeListRewardApp();
    static TypeId GetTypeId();
    void RegisterCallbacks() override;

  private:
    /**
     * Creates the hidden reward component, the normalized node-Id.
     */
    void Observe();
};
} // namespace ns3
#endif // NS3_NODE_ID_REWARD_APP_H
