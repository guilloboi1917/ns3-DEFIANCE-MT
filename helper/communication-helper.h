#ifndef COMMUNICATION_HELPER_H
#define COMMUNICATION_HELPER_H

#include "ns3/agent-application.h"
#include "ns3/channel-interface.h"
#include "ns3/ipv4-address.h"
#include "ns3/object-factory.h"
#include "ns3/observation-application.h"
#include "ns3/rl-application-container.h"
#include "ns3/rl-application.h"
#include "ns3/simple-channel-interface.h"
#include "ns3/socket-channel-interface.h"

namespace ns3
{

/**
 * @ingroup defiance
 * @brief Common parameters used for communication between RlApplications over a
 * ChannelInterface.
 */
struct CommunicationAttributes
{
    Time delay{0}; //!< delay for arriving messages if using the SimpleChannelInterface
    CommunicationAttributes() {};
    CommunicationAttributes(Time d)
        : delay(d) {};
    virtual ~CommunicationAttributes() = default;
};

/**
 * @ingroup defiance
 * @brief CommunicationAttributes needed for information exchange over a
 * SocketChannelInterface.
 */
struct SocketCommunicationAttributes : CommunicationAttributes
{
    Ipv4Address clientAddress; //!< Address of client's NetDevice
    Ipv4Address serverAddress; //!< Address of server's NetDevice, usually an AgentApplication

    SocketCommunicationAttributes(Ipv4Address client, Ipv4Address server)
        : clientAddress(client),
          serverAddress(server) {};

    SocketCommunicationAttributes(Ipv4Address client, Ipv4Address server, TypeId protocol)
        : clientAddress(client),
          serverAddress(server),
          protocol(protocol) {};

    TypeId protocol = UdpSocketFactory::GetTypeId(); //!< protocol to be used for the connection
};

/**
 * @ingroup defiance
 * @brief Hold information for constructing a ChannelInterface between two  RlApplications.
 * Depending on the \c attributes, either a SimpleChannelInterface or a  SocketChannelInterface
 * will be used.
 */
struct CommunicationPair
{
    RlApplicationId clientId;                  //!< RlApplicationId that identifies client App
    RlApplicationId serverId;                  //!< RlApplicationId that identifies server App
    const CommunicationAttributes& attributes; //!< All attributes needed to set up the
                                               //!< communication between client App and server App
};

/**
 * @ingroup defiance
 * @class CommunicationHelper
 * @brief Helper to setup communication between RlApplications
 */
class CommunicationHelper : public Object
{
  public:
    explicit CommunicationHelper(bool checkForSimpleInterfaces = true);

    /**
     * @brief Get the type ID.
     * @return the object TypeId.
     */
    static TypeId GetTypeId();

    /**
     * @brief Set the ObservationApps which are available in the scenario.
     * @param observationApps container of all ObservationApps of the scenario.
     * Currently, the order in the container is used to create the apps' IDs.
     */
    void SetObservationApps(RlApplicationContainer observationApps);
    /**
     * @brief Set the RewardApps which are available in the scenario.
     * @param rewardApps container of all RewardApps of the scenario.
     * Currently, the order in the container is used to create the apps' IDs.
     */
    void SetRewardApps(RlApplicationContainer rewardApps);
    /**
     * @brief Set the AgentApps which are available in the scenario.
     * @param agentApps container of all AgentApps of the scenario.
     * Currently, the order in the container is used to create the apps' IDs.
     */
    void SetAgentApps(RlApplicationContainer agentApps);
    /**
     * @brief Set the ActionApps which are available in the scenario.
     * @param actionApps container of all ActionApps of the scenario.
     * Currently, the order in the container is used to create the apps' IDs.
     */
    void SetActionApps(RlApplicationContainer actionApps);

    /**
     * @brief Get the corresponding RlApplication for this RlApplicationId by looking it up in the
     * right RlApplicationContainer.
     * @param id the ID to look up.
     * @return the corresponding RlApplication.
     */
    Ptr<RlApplication> GetApp(RlApplicationId id);

    /**
     * @brief Connect specified RlApplications.
     * @param communicationPairs includes a CommunicationPair for every connection to be made,
     * \c clientId corresponds to ID of the first RlApplication, \c serverId to that of
     * the second one.
     */
    void AddCommunication(const std::vector<CommunicationPair>& communicationPairs);

    /**
     * @brief Delete the communication connection between two RlApplications including the two
     * interfaces.
     * @param localAppId ID of the local RlApplication.
     * @param remoteAppId ID of the remote RlApplication connected to the local RlApplication.
     * @param localInterfaceId ID of the interface in the local RlApplication.
     * @param remoteInterfaceId ID of the interface in the remote RlApplication.
     */
    void DeleteCommunication(RlApplicationId localAppId,
                             RlApplicationId remoteAppId,
                             uint localInterfaceId = 0,
                             uint remoteInterfaceId = 0);

    /**
     * @brief Assigns a unique RlApplicationId to all RlApplications registered in this
     * helper. Should be called before communications are set, so that IDs can already be used
     * there.
     *
     * This assigns the IDs in increasing order starting at 0. To customize, override this function.
     */
    void SetIds();

    /**
     * @brief Calls \c Setup() on all RlApplications registered in this helper.
     * Should be called after all RlApplications were set.
     */
    void Configure();

  protected:
    /**
     * @brief Set up a communication relationship between two RlApplications over ChannelInterfaces.
     * @param clientApp first communication partner.
     * @param serverApp second communication partner (usually AgentApplication).
     * @param attributes reference to all attributes needed to set up the communication between
     * \c clientApp and \c serverApp.
     * @return a pair consisting of the ChannelInterface for the \c clientApp and the
     * ChannelInterface for the \c serverApp.
     */
    std::pair<Ptr<ChannelInterface>, Ptr<ChannelInterface>> Connect(
        Ptr<RlApplication> clientApp,
        Ptr<RlApplication> serverApp,
        const CommunicationAttributes& attributes) const;

  private:
    RlApplicationContainer
        m_observationApps; //!< stores all ObservationApplications available in the scenario
    RlApplicationContainer
        m_rewardApps;                   //!< stores all RewardApplications available in the scenario
    RlApplicationContainer m_agentApps; //!< stores all AgentApplications available in the scenario
    RlApplicationContainer
        m_actionApps; //!< stores all ActionApplications available in the scenario

    bool m_checkForSimpleInterfaces; //!< if \c true, RlApplications on the same Node will
                                     //!< automatically be connected via SimpleChannelInterfaces

    Time m_connectTime{0}; //!< time when the connections are established between interfaces.

    /**
     * Create two ChannelInterfaces between \c clientApp and \c serverApp with corresponding
     * \c attributes.
     * @return the newly created ChannelInterfaces.
     */
    std::pair<Ptr<ChannelInterface>, Ptr<ChannelInterface>> CreateChannelInterfaces(
        Ptr<RlApplication> clientApp,
        Ptr<RlApplication> serverApp,
        const CommunicationAttributes& attributes) const;
}; // class CommunicationHelper

} // namespace ns3

#endif /* COMMUNICATION_HELPER_H */
