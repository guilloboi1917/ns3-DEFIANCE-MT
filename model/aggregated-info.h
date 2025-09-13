#ifndef AGGREGATED_INFO_H
#define AGGREGATED_INFO_H

#include <cstdlib>

namespace ns3
{

/**
 * @ingroup defiance
 * @class AggregatedInfo
 * @brief A helper class to store aggregated information about a single observation/reward. The
 * aggregated information includes the minimum, maximum, average and sum of the observation/reward
 * values.
 */
class AggregatedInfo
{
  public:
    AggregatedInfo();

    void UpdateStatistics(AggregatedInfo update);

    void UpdateStatistics(float update);

    float GetMin() const;

    float GetMax() const;

    float GetAvg() const;

    void UpdateMin(float value);

    void UpdateMax(float value);

    void UpdateAverage(float value);

    virtual void CustomUpdate(float value, uint index, uint id);

  private:
    float m_min;
    float m_max;
    float m_avg;
    float m_sum;
    float m_counter;

    void IncrementSum(float value);
};
} // namespace ns3
#endif
