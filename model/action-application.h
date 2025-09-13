#ifndef ACTION_APPLICATION_H
#define ACTION_APPLICATION_H

#include "channel-interface.h"
#include "rl-application.h"

namespace ns3
{
/**
 * @ingroup defiance
 * @class ActionApplication
 * @brief This application is installed on a node where actions are applied.
 */
class ActionApplication : public RlApplication
{
  public:
    ActionApplication();
    ~ActionApplication() override;

    /**
     * @brief Get the type ID.
     * @return the object TypeId.
     */
    static TypeId GetTypeId();

    /**
     * @brief Add a ChannelInterface to this ActionApplication over which it can communicate with
     * another AgentApplication.
     * @param remoteAppId the ID of the AgentApplication the interface is connected to.
     * @param interface the interface for communicating with specific AgentApplication.
     * @return the ID of the newly added interface among all interfaces of this
     * ActionApplication which are connected to the specified AgentApplication.
     */
    uint AddAgentInterface(uint32_t remoteAppId, Ptr<ChannelInterface> interface) override;

    /**
     * @brief Delete a ChannelInterface that is currently connected to a remote AgentApplication.
     * @param remoteAppId the ID of the AgentApplication the interface is connected to.
     * @param interfaceId the ID of the ChannelInterface to delete. This is the ID among all
     * interfaces of this ActionApplication which are connected to the specified
     * AgentApplication.
     */
    void DeleteAgentInterface(uint32_t remoteAppId, uint interfaceId) override;

    /**
     * @brief Executes whatever has to be done if an action is received from an AgentApplication.
     * @param remoteAppId the ID of the AgentApplication from which the action is received.
     * @param action the received action.
     */
    virtual void ExecuteAction(uint remoteAppId, Ptr<OpenGymDictContainer> action) = 0;

  protected:
    InterfaceMap m_interfaces; //!< All interfaces connected to this ActionApplication for receiving
                               //!< from AgentApplications.
};

} // namespace ns3

#endif /* ACTION_APPLICATION_H */
