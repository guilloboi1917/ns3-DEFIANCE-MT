#include "pc-environment.h"

#include "ns3/lte-module.h"

#include <numeric>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("PcSimulation");

void
updateSinr(PowerControlMobComEnv* env,
           uint16_t ueId,
           uint16_t cellId,
           uint16_t rnti,
           double rsrp,
           double sinr,
           uint8_t ccId)
{
    env->sinrs[cellId - 1][ueId] = 10 * std::log10((float)sinr);
    env->cbCalls++;

    // also measure the currently set tx power
    int i = 0;
    auto& bsDevices = env->GetDeviceManager()->GetBsDevices();
    for (auto it = bsDevices.Begin(); it != bsDevices.End(); it++)
    {
        env->powersMeasured[i] = DynamicCast<LteEnbNetDevice>(*it)->GetPhy()->GetTxPower();
        i++;
    }
}

void
PowerControlMobComEnv::SetupScenario()
{
    NS_LOG_FUNCTION(this);
    MobComEnv::SetupScenario();
    sinrs.resize(m_deviceManager->GetBsNodes().GetN());
    for (auto& ues : sinrs)
    {
        ues.resize(m_deviceManager->GetUtNodes().GetN());
    }
    // assumes that sinr is not given in dB
    sinrThreshold = 50; // https://www.lte-anbieter.info/technik/sinr.php

    powers.resize(m_deviceManager->GetBsDevices().GetN());
    powersMeasured.resize(m_deviceManager->GetBsDevices().GetN());
}

void
PowerControlMobComEnv::SetScenarioAttributes()
{
    GetLteHelper()->SetPathlossModelType(TypeId::LookupByName(arg_propagationModel));
    GetLteHelper()->SetHandoverAlgorithmType(arg_handoverAlgorithm);
    Config::SetDefault("ns3::LteUePhy::NoiseFigure", DoubleValue(arg_noiseFigure));
    // Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));
}

void
PowerControlMobComEnv::RegisterCallbacks()
{
    NS_LOG_FUNCTION(this);
    auto& utDevices = GetDeviceManager()->GetUtDevices();
    // TODO: remove EnableDlTxPhyTraces without introducing this issue: free(): invalid next size
    // (fast)
    uint16_t ueId = 0;
    for (auto it = utDevices.Begin(); it != utDevices.End(); it++)
    {
        auto map = DynamicCast<LteUeNetDevice>(*it)->GetCcMap();

        for (auto& mapIt : map)
        {
            DynamicCast<ComponentCarrierUe>(mapIt.second)
                ->GetPhy()
                ->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr",
                                             MakeBoundCallback(&updateSinr, this, ueId));
        }
        ueId++;
    }

    // Config::Connect(
    //     "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
    //     MakeBoundCallback(&updateSinr2, this));
}

std::vector<uint32_t>
PowerControlMobComEnv::GetObservationShape()
{
    NS_LOG_FUNCTION(this);
    return {static_cast<unsigned int>(GetDeviceManager()->GetUtDevices().GetN() *
                                      GetDeviceManager()->GetBsDevices().GetN())};
}

Ptr<OpenGymSpace>
PowerControlMobComEnv::GetObservationSpace()
{
    NS_LOG_FUNCTION(this);

    std::string dtype = TypeNameGet<float>();
    // assumes that sinr is given in dB
    auto box = CreateObject<OpenGymBoxSpace>(-100, 100.0, GetObservationShape(), dtype);
    return box;
}

Ptr<OpenGymDataContainer>
PowerControlMobComEnv::GetObservation()
{
    NS_LOG_FUNCTION(this);
    auto box = CreateObject<OpenGymBoxContainer<float>>(GetObservationShape());

    for (auto& cells : sinrs)
    {
        for (float uesinr : cells)
        {
            NS_LOG_INFO("Return Observation " << uesinr);
            box->AddValue(uesinr);
        }
    }

    return box;
}

Ptr<OpenGymSpace>
PowerControlMobComEnv::GetActionSpace()
{
    NS_LOG_FUNCTION(this);
    std::vector<uint32_t> shape = {
        static_cast<unsigned int>(GetDeviceManager()->GetBsDevices().GetN())};
    std::string dtype = TypeNameGet<float>();
    // assumes that power is between 0 and 46 dbm
    auto box = CreateObject<OpenGymBoxSpace>(0.0, 46.0, shape, dtype);
    return box;
}

bool
PowerControlMobComEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    NS_LOG_FUNCTION(this << action);
    auto box = action->GetObject<OpenGymBoxContainer<float>>();

    int bsId = 0;
    auto& bsDevices = m_deviceManager->GetBsDevices();
    for (auto it = bsDevices.Begin(); it != bsDevices.End(); it++)
    {
        float powerdbm = box->GetValue(bsId);
        DynamicCast<LteEnbNetDevice>(*it)->GetPhy()->SetTxPower(powerdbm);
        NS_LOG_INFO("Set Power (in dbm) for bs " << bsId << " to " << powerdbm << " dBm)");
        powers[bsId] = (float)powerdbm;
        bsId++;
    }

    return true;
}

float
PowerControlMobComEnv::GetReward()
{ // negative difference for all ues
  // under the threshold else inverse
  // of sum of power
    NS_LOG_FUNCTION(this);

    float negativeReward = 0;
    float powerSum = std::accumulate(powers.begin(), powers.end(), 0);
    for (auto& cell : sinrs)
    {
        for (auto ueSinr : cell)
        {
            if (ueSinr < sinrThreshold)
            {
                negativeReward += ueSinr - sinrThreshold;
            }
        }
    }
    float reward = 0.0;
    if (negativeReward < 0.0)
    {
        reward = negativeReward;
    }
    else
    {
        reward = 1.0 / powerSum;
    }
    NS_LOG_INFO("Return Reward " << reward);
    return reward;
}

std::map<std::string, std::string>
PowerControlMobComEnv::GetExtraInfo()
{
    NS_LOG_FUNCTION(this);
    auto& utDevices = GetDeviceManager()->GetUtDevices();

    std::map<std::string, std::string> extraInfo = sinrVecToMap();

    std::vector<double> distances;
    for (uint32_t i = 0; i < utDevices.GetN(); i++)
    {
        double distance = 0.0;
        auto utDevice = utDevices.Get(i);
        auto enbDevice = DynamicCast<LteEnbNetDevice>(GetDeviceManager()->GetBsDevices().Get(0));
        if (enbDevice)
        {
            auto targetNode = enbDevice->GetNode();
            auto mm = utDevice->GetNode()->GetObject<MobilityModel>();
            distance = mm->GetDistanceFrom(targetNode->GetObject<MobilityModel>());
        }
        else
        {
            distance = -1.0;
        }
        extraInfo["distance_" + std::to_string(i)] = std::to_string(distance);
    }
    extraInfo["cbCalls"] = std::to_string(cbCalls);
    for (std::vector<float>::size_type i = 0; i < powersMeasured.size(); i++)
    {
        extraInfo["power_bs_" + std::to_string(i)] = std::to_string(powersMeasured[i]);
    }
    extraInfo["time"] = std::to_string(Simulator::Now().GetMilliSeconds());

    return extraInfo;
}

std::map<std::string, std::string>
PowerControlMobComEnv::sinrVecToMap()
{
    std::map<std::string, std::string> m;
    for (std::vector<std::vector<float>>::size_type bs = 0; bs < sinrs.size(); bs++)
    {
        for (std::vector<float>::size_type ue = 0; ue < sinrs[bs].size(); ue++)
        {
            m["sinr_bs" + std::to_string(bs) + "ue" + std::to_string(ue)] =
                std::to_string(sinrs[bs][ue]);
            sinrs[bs][ue] = 0;
        }
    }
    return m;
}

// cli arguments can also be set using a callback
// bool PowerControlMobComEnv::SetValue(const std::string & val)
// {
//     m_attributes["speed"] = std::make_any<std::string>(val);
//     return true;
// }

void
PowerControlMobComEnv::ParseCliArgs(int argc, char* argv[])
{
    NS_LOG_FUNCTION(this);
    CommandLine cmd(__FILE__);
    // cmd.AddValue("speed", "Speed of the UE", MakeCallback(PowerControlMobComEnv::SetValue));
    cmd.AddValue("speed", "Speed of the UE", this->arg_speed);
    cmd.AddValue("topology", "Topology to use", this->arg_topology);
    cmd.AddValue("noiseFigure", "Noise figure of the UE", this->arg_noiseFigure);
    cmd.AddValue("propagationModel", "Propagation model to use", this->arg_propagationModel);
    cmd.AddValue("handoverAlgorithm", "Handover algorithm to use", this->arg_handoverAlgorithm);
    cmd.Parse(argc, argv);
}

void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi
              << ": previously connected to CellId " << cellId << " with RNTI " << rnti
              << ", doing handover to CellId " << targetCellId << std::endl;
}

void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi
              << ": successful handover to CellId " << cellId << " with RNTI " << rnti << std::endl;
}

void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellId,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellId
              << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId "
              << targetCellId << std::endl;
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellId
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
    using namespace ns3;

    CommandLine cmd(__FILE__);
    Ptr<PowerControlMobComEnv> environment =
        CreateObject<PowerControlMobComEnv>(Seconds(200), MilliSeconds(5), MilliSeconds(0));
    environment->ParseCliArgs(argc, argv);
    environment->SetupScenario();
    environment->GetLteHelper()->AddX2Interface(environment->GetDeviceManager()->GetBsNodes());

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkUe));
    Simulator::Stop(environment->GetSimulationDuration());
    Simulator::Run();
    environment->NotifySimulationEnd();

    return 0;
}
