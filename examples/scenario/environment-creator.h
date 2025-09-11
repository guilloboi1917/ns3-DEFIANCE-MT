#ifndef ENVIRONMENTCREATOR_H
#define ENVIRONMENTCREATOR_H

#include <ns3/lte-helper.h>
#include <ns3/point-to-point-epc-helper.h>

#include <cstdint>

namespace ns3
{

enum class ScenarioMode
{
    Pendulum,
    PendulumBaseStations,
    PendulumTrainingAgent,
    Handover,
    HandoverTrainingAgent,
    HandoverAgent,
    PRB,
    Undefined
};

/**
 * \ingroup defiance
 * \class EnvironmentCreator
 * \brief This class creates specific kinds of environment scenarios.
 */
class EnvironmentCreator
{
  public:
    EnvironmentCreator();

    /**
     * \brief Create \c numCarts cart nodes and place them at (0, 0). Schedule the start of their
     * physics simulation.
     */
    void CreateCarts(int numCarts, float trackLength);

    /**
     * \brief Create the scenario. This includes creating and placing eNodeBs, carts, installing
     * agents and setting up the IP communication between them.
     */
    void SetupPendulumScenario(uint32_t numCarts,
                               uint32_t numEnbs,
                               bool EnableTrainingServer); // Balancieren lernen 2 u. 3

    // Getter methods for the protected attributes

    /**
     * \brief Retrieve the cart nodes of the scenario.
     * \return NodeContainer with all cart nodes.
     */
    NodeContainer& GetCartNodes();

    /**
     * \brief Retrieve the eNodeB nodes of the scenario.
     * \return NodeContainer with all eNodeB nodes.
     */
    NodeContainer& GetEnbNodes();

    /**
     * \brief Retrieve the training agent nodes of the scenario.
     * \return NodeContainer with all training agent nodes.
     */
    NodeContainer& GetTrainingAgentNodes();

    /**
     * \brief Retrieve the inference agent nodes of the scenario.
     * \return NodeContainer with all inference agent nodes.
     */
    NodeContainer& GetInferenceAgentNodes();

    /**
     * \brief Retrieve the LTE helper of the scenario.
     * \return Ptr to the LTE helper.
     */
    Ptr<LteHelper> GetLteHelper();

    /**
     * \brief Retrieve the EPC helper of the scenario.
     * \return Ptr to the EPC helper.
     */
    Ptr<EpcHelper> GetEpcHelper();

  protected:
    /**
     * \brief Create and place the eNodeBs for the scenario.
     * \param numEnbs the number of eNodeBs to create.
     * \param coverageRadius the coverage radius of each eNodeB.
     */
    void CreateEnbs(int numEnbs, float coverageRadius);

    /**
     * \brief Place the eNodeBs randomly on a straight track. It is ensured that every point of the
     * track is within the coverage radius of at least one eNodeB.
     */
    void PlaceEnbsRandomly();

    float m_trackEnd;
    float m_coverageRadius;

    ScenarioMode m_mode;

    NodeContainer m_enbNodes;
    NodeContainer m_cartNodes;
    NodeContainer m_trainingAgentNodes;
    NodeContainer m_inferenceAgentNodes;

    Ptr<LteHelper> m_lteHelper;
    Ptr<PointToPointEpcHelper> m_epcHelper;
};
} // namespace ns3

#endif
