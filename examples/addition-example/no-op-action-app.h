#ifndef NS3_NO_OP_ACTION_APP_H
#define NS3_NO_OP_ACTION_APP_H

#include "ns3/action-application.h"

namespace ns3
{

class NoOpActionApp : public ActionApplication
{
  public:
    NoOpActionApp();
    static TypeId GetTypeId();
    void ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action) override;
};
} // namespace ns3
#endif // NS3_NO_OP_ACTION_APP_H
