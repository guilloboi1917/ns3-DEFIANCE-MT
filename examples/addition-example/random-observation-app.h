#ifndef NS3_RANDOM_OBSERVATION_APP_H
#define NS3_RANDOM_OBSERVATION_APP_H

#include <ns3/observation-application.h>
#include <ns3/random-variable-stream.h>

namespace ns3
{

class RandomObservationApp : public ObservationApplication
{
  public:
    RandomObservationApp();
    static TypeId GetTypeId();
    void RegisterCallbacks() override;

    /**
     * Assigns stream number to UniformRandomVariable used to
     * generate observations.
     *
     * \param stream the stream number to assign
     */
    int64_t AssignStreams(int64_t stream) override;

  private:
    /// random variable stream for random observation.
    Ptr<UniformRandomVariable> m_uv;

    /**
     * Creates observation in form of a random number.
     */
    void Observe();
};
} // namespace ns3
#endif // NS3_RANDOM_OBSERVATION_APP_H
