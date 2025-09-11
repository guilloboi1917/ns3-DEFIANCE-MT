#ifndef DEFIANCE_DEVICE_MANAGER_H
#define DEFIANCE_DEVICE_MANAGER_H

#include <ns3/net-device-container.h>
#include <ns3/node-container.h>

#include <map>

namespace ns3
{
class MobComEnv;

// Each class should be documented using Doxygen and have an \ingroup defiance directive

/**
 * \ingroup defiance
 * \class DeviceManager
 * \brief Contains the information about the nodes and devices of the simulation. Maintains subsets
 * ot these nodes and devices to work with.
 */
class DeviceManager
{
  public:
    DeviceManager();
    ~DeviceManager();

    // Setter methods for the node and device containers
    void SetUtNodes(NodeContainer nodes);
    void SetBsNodes(NodeContainer nodes);
    void SetUtDevices(NetDeviceContainer devices);
    void SetBsDevices(NetDeviceContainer devices);

    // Getters that return references and therefore allow to modify the internal state
    NodeContainer& GetUtNodes();
    NodeContainer& GetBsNodes();
    NetDeviceContainer& GetUtDevices();
    NetDeviceContainer& GetBsDevices();

    // management of semantic subsets
    // user terminals
    void AddUtSubset(std::string subsetName, std::vector<uint32_t> nodeIndices);
    void AddUtSubset(std::string subsetName, NodeContainer nodes);
    std::vector<uint32_t> GetUtSubsetIndices(std::string subsetName);
    NodeContainer GetUtSubsetNodes(std::string subsetName);
    NetDeviceContainer GetUtSubsetDevices(std::string subsetName);
    // base stations
    void AddBsSubset(std::string subsetName, std::vector<uint32_t> nodeIndices);
    void AddBsSubset(std::string subsetName, NodeContainer nodes);
    std::vector<uint32_t> GetBsSubsetIndices(std::string subsetName);
    NodeContainer GetBsSubsetNodes(std::string subsetName);
    NetDeviceContainer GetBsSubsetDevices(std::string subsetName);

  private:
    NodeContainer m_UtNodes;
    NodeContainer m_BsNodes;
    NetDeviceContainer m_UtDevices;
    NetDeviceContainer m_BsDevices;
    std::map<std::string, std::vector<uint32_t>>
        m_UtSubsets; // named subsets containing user terminal node / device indices
                     // Assumption: each node contains one device
    std::map<std::string, std::vector<uint32_t>>
        m_BsSubsets; // named subsets containing base station node / device indices
                     // Assumption: each node contains one device
};                   // class DeviceManager
} // namespace ns3

#endif /* DEFIANCE__DEVICE_MANAGER_H */
