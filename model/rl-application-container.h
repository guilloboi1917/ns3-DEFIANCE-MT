#ifndef RL_APPLICATION_CONTAINER_H
#define RL_APPLICATION_CONTAINER_H

#include "rl-application.h"

#include "ns3/application-container.h"

namespace ns3
{

/**
 * @ingroup defiance
 * @class RlApplicationContainer
 * @brief Instances of this class hold a vector of ns3::Application pointers.
 *
 * Typically ns-3 Applications are installed on nodes using an Application helper. The helper
 * Install method takes a NodeContainer which holds some number of Ptr<Node>. For each of the Nodes
 * in the NodeContainer the helper will instantiate an application, install it in a node and add a
 * Ptr<Application> to that application into a Container for use by the caller. This is that
 * container used to hold the Ptr<Application> which are instantiated by the Application helper.
 */
class RlApplicationContainer : public ApplicationContainer
{
  public:
    /**
     * Create an empty RlApplicationContainer.
     */
    RlApplicationContainer()
        : ApplicationContainer()
    {
    }

    /**
     * @brief Create an RlApplicationContainer from a plain ApplicationContainer.
     */
    RlApplicationContainer(ApplicationContainer c)
        : ApplicationContainer(c)
    {
    }

    /**
     * Create an RlApplicationContainer with exactly one application which has been previously
     * instantiated. The single application is specified by a smart pointer.
     *
     * @param application The Ptr<Application> to add to the container. If you want to use the
     * application as an RlApplication, you should add it to the container as such.
     */
    RlApplicationContainer(Ptr<Application> application)
        : ApplicationContainer(application)
    {
    }

    /**
     * Create an RlApplicationContainer with exactly one application which has been previously
     * instantiated and assigned a name using the Object Name Service. This Application is then
     * specified by its assigned name.
     *
     * @param name The name of the Application Object to add to the container.
     */
    RlApplicationContainer(std::string name)
        : ApplicationContainer(name)
    {
    }

    /**
     * @brief Get the RlApplication stored in this container at a given index.
     *
     * @param index the index of the requested RlApplication pointer.
     * @return the requested RlApplication pointer.
     */
    Ptr<RlApplication> Get(uint32_t index) const;

    /**
     * @brief Get the RlApplicationId of the RlApplication stored in this container at a given
     * index.
     *
     * @param index the index of the requested RlApplication.
     * @return the RlApplicationId of the requested RlApplication.
     */
    RlApplicationId GetId(uint32_t index) const;
};

} // namespace ns3

#endif // APP_CONTAINER_H
