#ifndef PC_ENVIRONMENT_H
#define PC_ENVIRONMENT_H

#include "base-environment.h"
#include "device-manager.h"

#include "ns3/ai-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/point-to-point-module.h"

#include <string>

namespace ns3
{

/**
 * @ingroup defiance
 *
 * @brief Single agent environment for training power control with reinforcement learning.
 */
class PowerControlMobComEnv : public MobComEnv
{
  public:
    using MobComEnv::MobComEnv;
    // gym interface
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;
    std::map<std::string, std::string> GetExtraInfo() override;

    // ns3 related code
    void SetupScenario() override;
    void SetScenarioAttributes() override;
    void CreateTopology() override;
    void AddTraffic() override;
    void RegisterCallbacks() override;

    // class specific helper methods and attributes
    void CreateBsLine(uint16_t numBSs, uint64_t lineLength);
    void CreateRandomFixedTopology(uint16_t numUEs, uint16_t numBSs, uint64_t lineLength);
    void CreateSimpleTopology(Vector position, Vector velocity);
    void CreateMultiBsTopology();

    void ParseCliArgs(int argc, char* argv[]);
    // static bool SetValue(const std::string & val);

    std::vector<uint32_t> GetObservationShape();

    std::map<std::string, std::string> sinrVecToMap();

    std::vector<std::vector<float>> sinrs;
    std::vector<float> powers;
    std::vector<float> powersMeasured;
    int cbCalls = 0;

    double arg_speed = 0;
    std::string arg_topology = "simple";
    double arg_noiseFigure = 9;
    std::string arg_propagationModel = "ns3::FriisPropagationLossModel";
    std::string arg_handoverAlgorithm = "ns3::NoOpHandoverAlgorithm";

  private:
    float sinrThreshold;
};

} // namespace ns3
#endif /* PC_ENVIRONMENT_H */
