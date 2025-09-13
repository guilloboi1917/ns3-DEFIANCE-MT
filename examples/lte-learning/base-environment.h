#ifndef BASE_ENVIRONMENT_H
#define BASE_ENVIRONMENT_H

#include "device-manager.h"

#include "ns3/container.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/ns3-ai-gym-env.h"

namespace ns3
{

class DeviceManager;

// NS_LOG_COMPONENT_DEFINE("lte-base-enviroment");

/**
 * @ingroup defiance
 * @class MobComEnv
 * @brief Environment specifically designed to train mobile communication related tasks.
 */
class MobComEnv : public OpenGymEnv
{
  public:
    MobComEnv();

    /**
     * @brief Constructor for the MobComEnv class.
     * @param simulationDuration the maximum of the simulation.
     * @param notificationRate the rate at which \c Notify() should be called to send the
     * observations to python and get the agent's actions.
     * @param notificationStart the time at which the first notification should be sent.
     */
    MobComEnv(Time simulationDuration, Time notificationRate, Time notificationStart);

    /**
     * @brief Get the type ID.
     * @return the object TypeId.
     */
    static TypeId GetTypeId();

    void DoDispose() override
    {
    }

    // abstract OpenGym interfaces
    Ptr<OpenGymSpace> GetActionSpace() override = 0;
    Ptr<OpenGymSpace> GetObservationSpace() override = 0;
    Ptr<OpenGymDataContainer> GetObservation() override = 0;
    float GetReward() override = 0;
    bool ExecuteActions(Ptr<OpenGymDataContainer> action) override = 0;

    /**
     * @brief Always return false here, as mobile environments are continous. This becomes the
     * terminated field in Gymnasium. Use a `TimeLimit` object to truncate the environment after a
     * given amount of steps.
     */
    bool GetGameOver() override
    {
        return false;
    }

    std::map<std::string, std::string> GetExtraInfo() override
    {
        return {};
    };

    /**
     * @return the total simulation duration.
     */
    Time GetSimulationDuration();

    // helper objects for orchestration

    /**
     * @return a pointer to the nr helper.
     */
    Ptr<LteHelper> GetLteHelper();

    /**
     * @return a pointer to the epc helper.
     */
    Ptr<EpcHelper> GetEpcHelper();

    /**
     * @return a pointer to the device manager.
     */
    DeviceManager* GetDeviceManager();

  protected:
    // networking related methods

    /**
     * @brief Template method for scenario creation. Customize it by overriding
     * \c SetScenarioAttributes(), \c ScheduleNotificationEvents(), \c CreateTopology(),
     * \c AddTraffic() and \c RegisterCallbacks().
     */
    virtual void SetupScenario();

    /**
     * @brief Override this method in the child class to set global attributes, or attributes
     * specific to the LteHelper or EpcHelper. In case you do not override this method, default
     * values will be used. Example for setting the Propagation Model:
     * GetLteHelper()->SetPathlossModelType(TypeId::LookupByName("ns3::RangePropagationLossModel"));
     * GetLteHelper()->SetPathlossModelAttribute("MaxRange", DoubleValue(2000));
     */
    virtual void SetScenarioAttributes() {};

    /**
     * @brief Fill containers for UEs and base stations.
     * The specific topology has to be specified in the child class.
     */
    virtual void CreateTopology() = 0;

    /**
     * @brief Overwrite to add internet traffic to the scenario.
     * Include 1. Routing and IPs, 2. Attaching UEs and eNbs, and 3. Registering applications.
     * \sa static-environment.cc
     */
    virtual void AddTraffic() = 0;

    /**
     * @brief Register the necessary callbacks for tracing.
     */
    virtual void RegisterCallbacks()
    {
    }

    /**
     * @brief Attach user equipment to base stations. By default, each UE is attached to the closest
     * base station.
     */
    void AttachUEsToEnbs();

    /**
     * @brief Schedule the \c Notify() events according to \c m_notificationRate
     */
    void ScheduleNotificationEvents();

    Ptr<LteHelper> m_lteHelper;
    Ptr<EpcHelper> m_epcHelper;
    DeviceManager* m_deviceManager;

    const Time m_simulationDuration; //!< Maximum duration of the simulation Default = Seconds(200);
    const Time m_notificationRate;   //!< The rate, at wich \c Notify() should be called to send the
                                     //!< observations to python and get the agent's actions.
                                     //!< Default = MilliSeconds(5)
    const Time m_notificationStart;  //!< The time that \c Notify() should be called for the first
                                     //!< time. Default = MilliSeconds(300)
};
} // namespace ns3

#endif /* BASE_ENVIRONMENT_H */
