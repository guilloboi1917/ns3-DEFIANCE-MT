#ifndef REWARD_APPLICATION_H
#define REWARD_APPLICATION_H

#include "data-collector-application.h"

namespace ns3
{
/**
 * @ingroup defiance
 * @class RewardApplication
 * @brief This application is installed on a node where rewards are calculated.
 *
 * Instances of this class do not send any data if they are not registered at a callback. To send
 * rewards this class should be inherited by a class which is registered at a callback and which
 * sends data received from the callback as rewards. In the child class, a method like  \c Reward()
 * should be created and registered at a callback to make sure it is regularly called. This method's
 * signature has to conform to the signature expected by the callback. The purpose of \c Reward() is
 * to aggregate data received from the callback into a reward and specify to which ChannelInterfaces
 * it shall be sent. Therefore, in the end of the \c Reward() method, \c Send() should be called
 * with the according reward and InterfaceIds. If the reward shall be sent to all ChannelInterfaces,
 * the overloaded \c Send() method without specifying InterfaceIds should be called.
 */
class RewardApplication : public DataCollectorApplication
{
  public:
    RewardApplication();
    ~RewardApplication();

    /**
     * @brief Get the type ID.
     * @return the object TypeId.
     */
    static TypeId GetTypeId();
};

} // namespace ns3

#endif /* REWARD_APPLICATION_H */
