#ifndef RL_APPLICATIONS_HELPER_H
#define RL_APPLICATIONS_HELPER_H

#include "ns3/address.h"
#include "ns3/attribute.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/rl-application-container.h"

#include <string>

namespace ns3
{

/**
 * @ingroup defiance
 * @class RlApplicationHelper
 * @brief A helper to make it easier to instantiate an RlApplication on a set of nodes.
 */
class RlApplicationHelper
{
  public:
    /**
     * Create an RlApplication of a given type ID
     *
     * @param typeId the type ID.
     */
    explicit RlApplicationHelper(TypeId typeId);

    /**
     * Create an RlApplication of a given type ID
     *
     * @param typeId the type ID expressed as a string.
     */
    explicit RlApplicationHelper(const std::string& typeId);

    /**
     * Allow the helper to be repurposed for another RlApplication type
     *
     * @param typeId the type ID.
     */
    void SetTypeId(TypeId typeId);

    /**
     * Allow the helper to be repurposed for another RlApplication type
     *
     * @param typeId the type ID expressed as a string.
     */
    void SetTypeId(const std::string& typeId);

    /**
     * Helper function used to set the underlying application attributes.
     *
     * @param name the name of the application attribute to set
     * @param value the value of the application attribute to set
     */
    void SetAttribute(const std::string& name, const AttributeValue& value);

    /**
     * Install an RlApplication on each node of the input container
     * configured with all the attributes set with SetAttribute.
     *
     * @param c NodeContainer of the set of nodes on which an application
     * will be installed.
     * @return Container of Ptr to the applications installed.
     */
    RlApplicationContainer Install(NodeContainer c);

    /**
     * Install an RlApplication on the node configured with all the
     * attributes set with SetAttribute.
     *
     * @param node The node on which an application will be installed.
     * @return Container of Ptr to the applications installed.
     */
    RlApplicationContainer Install(Ptr<Node> node);

    /**
     * Install an RlApplication on the node configured with all the
     * attributes set with SetAttribute.
     *
     * @param nodeName The node on which an application will be installed.
     * @return Container of Ptr to the applications installed.
     */
    RlApplicationContainer Install(const std::string& nodeName);

  protected:
    /**
     * Install an RlApplication on the node configured with all the
     * attributes set with SetAttribute.
     *
     * @param node The node on which an application will be installed.
     * @return Ptr to the Application installed.
     */
    virtual Ptr<RlApplication> DoInstall(Ptr<Node> node);

    ObjectFactory m_factory; //!< Object factory.
};

} // namespace ns3

#endif /* RL_APPLICATIONS_HELPER_H */
