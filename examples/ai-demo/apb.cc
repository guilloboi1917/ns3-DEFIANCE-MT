/*
 * Copyright (c) 2023 Huazhong University of Science and Technology
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 *
 *
 * Author:  Muyuan Shen <muyuan_shen@hust.edu.cn>
 */

#include "ns3/ai-module.h"
#include "ns3/core-module.h"

#include <chrono>
#include <iostream>
#include <random>

#define NUM_ENV 10000

namespace ns3
{

class ApbEnv : public OpenGymEnv
{
  public:
    ApbEnv();
    ~ApbEnv() override;
    static TypeId GetTypeId();
    void DoDispose() override;

    int GetAPlusB();

    // OpenGym interfaces:
    Ptr<OpenGymSpace> GetActionSpace() override;
    Ptr<OpenGymSpace> GetObservationSpace() override;
    bool GetGameOver() override;
    Ptr<OpenGymDataContainer> GetObservation() override;
    float GetReward() override;
    std::map<std::string, std::string> GetExtraInfo() override;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override;

    int m_a;
    int m_b;

  private:
    int m_sum;
};

ApbEnv::ApbEnv()
{
    SetOpenGymInterface(OpenGymInterface::Get());
}

ApbEnv::~ApbEnv()
{
}

TypeId
ApbEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ApbEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

void
ApbEnv::DoDispose()
{
}

int
ApbEnv::GetAPlusB()
{
    Notify();
    return m_sum;
}

Ptr<OpenGymSpace>
ApbEnv::GetActionSpace()
{
    std::vector<uint32_t> shape = {1};
    std::string dtype = TypeNameGet<float>();
    auto box = CreateObject<OpenGymBoxSpace>(0, 20, shape, dtype);
    return box;
}

Ptr<OpenGymSpace>
ApbEnv::GetObservationSpace()
{
    std::vector<uint32_t> shape = {2};
    std::string dtype = TypeNameGet<float>();
    auto box = CreateObject<OpenGymBoxSpace>(0, 10, shape, dtype);
    return box;
}

bool
ApbEnv::GetGameOver()
{
    return false;
}

Ptr<OpenGymDataContainer>
ApbEnv::GetObservation()
{
    std::vector<uint32_t> shape = {2};
    auto box = CreateObject<OpenGymBoxContainer<float>>(shape);

    box->AddValue(m_a);
    box->AddValue(m_b);

    return box;
}

float
ApbEnv::GetReward()
{
    int neg = -abs(m_sum - m_a - m_b);
    return neg * 10;
}

std::map<std::string, std::string>
ApbEnv::GetExtraInfo()
{
    return {};
}

bool
ApbEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
    auto box = DynamicCast<OpenGymBoxContainer<float>>(action);
    m_sum = box->GetValue(0);
    return true;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
    using namespace ns3;

    auto apb = CreateObject<ApbEnv>();

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> distrib(1, 10);

    int sum;

    for (int i = 0; i < NUM_ENV; ++i)
    {
        apb->m_a = distrib(gen);
        apb->m_b = distrib(gen);
        std::cout << "set: " << apb->m_a << "," << apb->m_b << ";";
        std::cout << "\n";

        sum = apb->GetAPlusB();

        std::cout << "get: " << sum << ";";
        std::cout << "\n";
    }

    apb->NotifySimulationEnd();

    return 0;
}
