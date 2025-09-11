#include "device-manager.h"

#include <ns3/epc-helper.h>

#include <algorithm>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("DeviceManager");

DeviceManager::DeviceManager()
{
}

DeviceManager::~DeviceManager()
{
}

void
DeviceManager::SetUtNodes(NodeContainer nodes)
{
    m_UtNodes = nodes;
}

void
DeviceManager::SetBsNodes(NodeContainer nodes)
{
    m_BsNodes = nodes;
}

void
DeviceManager::SetUtDevices(NetDeviceContainer devices)
{
    m_UtDevices = devices;
}

void
DeviceManager::SetBsDevices(NetDeviceContainer devices)
{
    m_BsDevices = devices;
}

NodeContainer&
DeviceManager::GetUtNodes()
{
    return m_UtNodes;
}

NodeContainer&
DeviceManager::GetBsNodes()
{
    return m_BsNodes;
}

NetDeviceContainer&
DeviceManager::GetUtDevices()
{
    return m_UtDevices;
}

NetDeviceContainer&
DeviceManager::GetBsDevices()
{
    return m_BsDevices;
}

void
DeviceManager::AddUtSubset(std::string subsetName, std::vector<uint32_t> nodeIndices)
{
    m_UtSubsets[subsetName] = nodeIndices;
}

void
DeviceManager::AddUtSubset(std::string subsetName, NodeContainer nodes)
{
    std::vector<uint32_t> indices;
    for (auto it = nodes.Begin(); it != nodes.End(); it++)
    {
        auto element = std::find(m_UtNodes.Begin(), m_UtNodes.End(), *it);
        size_t index = std::distance(m_UtNodes.Begin(), element);
        NS_ASSERT_MSG(index != m_UtNodes.GetN(), "Node " << *it << " found in manager.");
        indices.emplace_back(index);
    }
    AddUtSubset(subsetName, indices);
}

std::vector<uint32_t>
DeviceManager::GetUtSubsetIndices(std::string subsetName)
{
    return m_UtSubsets[subsetName];
}

NodeContainer
DeviceManager::GetUtSubsetNodes(std::string subsetName)
{
    NodeContainer nodes;
    for (auto idx : m_UtSubsets[subsetName])
    {
        nodes.Add(m_UtNodes.Get(idx));
    }
    return nodes;
}

NetDeviceContainer
DeviceManager::GetUtSubsetDevices(std::string subsetName)
{
    NetDeviceContainer devices;
    for (auto idx : m_UtSubsets[subsetName])
    {
        devices.Add(m_UtDevices.Get(idx));
    }
    return devices;
}

void
DeviceManager::AddBsSubset(std::string subsetName, std::vector<uint32_t> nodeIndices)
{
    m_BsSubsets[subsetName] = nodeIndices;
}

void
DeviceManager::AddBsSubset(std::string subsetName, NodeContainer nodes)
{
    std::vector<uint32_t> indices;
    for (auto it = nodes.Begin(); it != nodes.End(); it++)
    {
        auto element = std::find(m_BsNodes.Begin(), m_BsNodes.End(), *it);
        size_t index = std::distance(m_BsNodes.Begin(), element);
        NS_ASSERT_MSG(index != m_BsNodes.GetN(), "Node " << *it << " found in manager.");
        indices.emplace_back(index);
    }
    AddBsSubset(subsetName, indices);
}

std::vector<uint32_t>
DeviceManager::GetBsSubsetIndices(std::string subsetName)
{
    return m_BsSubsets[subsetName];
}

NodeContainer
DeviceManager::GetBsSubsetNodes(std::string subsetName)
{
    NodeContainer nodes;
    for (auto idx : m_BsSubsets[subsetName])
    {
        nodes.Add(m_BsNodes.Get(idx));
    }
    return nodes;
}

NetDeviceContainer
DeviceManager::GetBsSubsetDevices(std::string subsetName)
{
    NetDeviceContainer devices;
    for (auto idx : m_BsSubsets[subsetName])
    {
        devices.Add(m_BsDevices.Get(idx));
    }
    return devices;
}

} // namespace ns3
