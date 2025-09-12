#include "rl-application-helper.h"

#include "ns3/data-rate.h"
#include "ns3/names.h"
#include "ns3/string.h"

namespace ns3
{

RlApplicationHelper::RlApplicationHelper(TypeId typeId)
{
    SetTypeId(typeId);
}

RlApplicationHelper::RlApplicationHelper(const std::string& typeId)
{
    SetTypeId(typeId);
}

void
RlApplicationHelper::SetTypeId(TypeId typeId)
{
    m_factory.SetTypeId(typeId);
}

void
RlApplicationHelper::SetTypeId(const std::string& typeId)
{
    m_factory.SetTypeId(typeId);
}

void
RlApplicationHelper::SetAttribute(const std::string& name, const AttributeValue& value)
{
    m_factory.Set(name, value);
}

RlApplicationContainer
RlApplicationHelper::Install(Ptr<Node> node)
{
    return RlApplicationContainer(DoInstall(node));
}

RlApplicationContainer
RlApplicationHelper::Install(const std::string& nodeName)
{
    auto node = Names::Find<Node>(nodeName);
    NS_ABORT_MSG_IF(!node, "Node " << nodeName << " does not exist");
    return RlApplicationContainer(DoInstall(node));
}

RlApplicationContainer
RlApplicationHelper::Install(NodeContainer c)
{
    RlApplicationContainer apps;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        apps.Add(DoInstall(*i));
    }
    return apps;
}

Ptr<RlApplication>
RlApplicationHelper::DoInstall(Ptr<Node> node)
{
    NS_ABORT_MSG_IF(!node, "Node does not exist");
    auto app = m_factory.Create<RlApplication>();
    node->AddApplication(app);
    return app;
}

} // namespace ns3
