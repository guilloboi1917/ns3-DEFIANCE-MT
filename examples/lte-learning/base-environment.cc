#include "base-environment.h"

#include <ns3/ns3-ai-gym-interface.h>
#include <ns3/point-to-point-epc-helper.h>

namespace ns3
{
NS_LOG_COMPONENT_DEFINE("lte-base-environment");

TypeId
MobComEnv::GetTypeId()
{
    static TypeId tid = TypeId("ns3::MobComEnv").SetParent<OpenGymEnv>().SetGroupName("OpenGym");
    return tid;
}

MobComEnv::MobComEnv()
    : MobComEnv(Seconds(200), MilliSeconds(5), MilliSeconds(300))
{
}

MobComEnv::MobComEnv(Time simulationDuration, Time notificationRate, Time notificationStart)
    : m_simulationDuration(simulationDuration),
      m_notificationRate(notificationRate),
      m_notificationStart(notificationStart)
{
    m_deviceManager = new DeviceManager();
    SetOpenGymInterface(OpenGymInterface::Get());
}

// from here follows code that is not required for the gym interface but actually implements the
// environment dynamics

void
MobComEnv::SetupScenario()
{
    NS_LOG_FUNCTION(this);
    // create the helper objects required for every lte simulation
    m_lteHelper = CreateObject<LteHelper>();
    m_epcHelper = CreateObject<PointToPointEpcHelper>();
    m_lteHelper->SetEpcHelper(m_epcHelper);
    SetScenarioAttributes();

    // set scheduling logic for communication between c++ and python
    ScheduleNotificationEvents();

    // sets the overall network topology
    CreateTopology();

    // create the internet
    AddTraffic();

    // register callbacks for tracing
    // m_lteHelper->EnableDlPhyTraces(); // TODO:move this into PcSimulation
    RegisterCallbacks();
}

void
MobComEnv::ScheduleNotificationEvents()
{
    NS_LOG_FUNCTION(this);
    Time cur = m_notificationStart;
    while (cur < m_simulationDuration)
    {
        Simulator::Schedule(cur, &MobComEnv::Notify, this);
        cur += m_notificationRate;
    }
}

void
MobComEnv::AttachUEsToEnbs()
{
    NS_LOG_FUNCTION(this);
    GetLteHelper()->AttachToClosestEnb(m_deviceManager->GetUtDevices(),
                                       m_deviceManager->GetBsDevices());
}

Ptr<LteHelper>
MobComEnv::GetLteHelper()
{
    return m_lteHelper;
}

Ptr<EpcHelper>
MobComEnv::GetEpcHelper()
{
    return m_epcHelper;
}

DeviceManager*
MobComEnv::GetDeviceManager()
{
    return m_deviceManager;
}

Time
MobComEnv::GetSimulationDuration()
{
    return m_simulationDuration;
}

} // namespace ns3
