#include "ns3/action-application.h"
#include "ns3/defiance-module.h"

class TemplateActionApplication : public ActionApplication
{
  public:
    TemplateActionApplication() {};
    ~TemplateActionApplication() {};

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::TemplateActionApplication")
                                .SetParent<ActionApplication>()
                                .SetGroupName("defiance")
                                .AddConstructor<TemplateActionApplication>();
        return tid;
    }

    // needs to be implemented
    void ExecuteAction(uint32_t remoteAppId, Ptr<OpenGymDictContainer> action) override
    {
        // implement your action execution here
        // auto cart = DynamicCast<PendulumCart>(GetNode());
        // auto act = DynamicCast<OpenGymTupleContainer>(action->Get("action"));
        // cart->SetAcceleration(DynamicCast<OpenGymBoxContainer<float>>(act->Get(1))->GetValue(0));
    }
};

NS_OBJECT_ENSURE_REGISTERED(TemplateActionApplication);
