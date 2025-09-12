#ifndef OBSERVATION_APPLICATION_H
#define OBSERVATION_APPLICATION_H

#include "data-collector-application.h"

namespace ns3
{
/**
 * @ingroup defiance
 * @class ObservationApplication
 * @brief This application is installed on a node where observations occur.
 *
 * Instances of this class do not send any data if they are not registered at a callback. To send
 * observations this class should be inherited by a class which is registered at a callback and
 * which sends data received from the callback as observations. In the child class, a method like
 * \c Observe() should be created and registered at a callback to make sure it is regularly called.
 * This method's signature has to conform to the signature expected by the callback. The purpose of
 * \c Observe() is to aggregate data received from the callback into an observation and specify to
 * which ChannelInterfaces it shall be sent. Therefore, in the end of the \c Observe() method,
 * \c Send() should be called with the according observation and InterfaceIds. If the observation
 * shall be sent to all ChannelInterfaces, the overloaded \c Send() method without specifying
 * InterfaceIds should be called.
 */
class ObservationApplication : public DataCollectorApplication
{
  public:
    ObservationApplication();
    ~ObservationApplication() override;

    /**
     * @brief Get the type ID.
     * @return the object TypeId.
     */
    static TypeId GetTypeId();
};

} // namespace ns3

#endif /* OBSERVATION_APPLICATION_H */
