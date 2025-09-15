#include "ns3/agent-application.h"
#include "ns3/defiance-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TemplateAgentApplication");

class TemplateAgentApplication : public AgentApplication
{
  public:
    TemplateAgentApplication()
        : AgentApplication(),
          // register custom data structure for agent messages here
          m_agentDataStruct(10) {};

    ~TemplateAgentApplication() override = default;

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::TemplateAgentApplication")
                                .SetParent<AgentApplication>()
                                .SetGroupName("defiance")
                                .AddConstructor<TemplateAgentApplication>();
        return tid;
    }

    // needs to be implemented, observation was already stored in m_obsDataStruct
    void OnRecvObs(uint id) override
    {
        NS_LOG_INFO("Received Observation from " << id);
        // you can do something with received observations in m_obsDataStruct
        // auto latestObservation = m_obsDataStruct.GetNewestByID(0);
        // you can send a message to another agent
        // Ptr<OpenGymDictContainer> dictContainer = CreateObject<OpenGymDictContainer>();
        // SendToAgent(dictContainer, 1, 0);
    }

    // can be implemented to use inter-agent communication
    void OnRecvFromAgent(uint id, Ptr<OpenGymDictContainer> payload) override
    {
        // save agent message in m_agentDataStruct
        NS_LOG_FUNCTION(this << "Received message from agent " << id);
        m_agentDataStruct.Push(payload, id);
    }

    // needs to be implemented, reward was already stored in m_rewardDataStruct
    void OnRecvReward(uint id) override
    {
        NS_LOG_INFO("Received Reward from " << id);
    }

  protected:
    // if you need to store agent messages, define new HistoryContainer here
    HistoryContainer m_agentDataStruct;

  private:
    // needs to be implemented
    Ptr<OpenGymSpace> GetObservationSpace() override
    {
        return {};
    }

    // needs to be implemented
    Ptr<OpenGymSpace> GetActionSpace() override
    {
        return {};
    }
};

NS_OBJECT_ENSURE_REGISTERED(TemplateAgentApplication);
