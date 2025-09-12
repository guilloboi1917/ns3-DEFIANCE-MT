#include "aggregated-info.h"

#include "ns3/log.h"

#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AggregatedInfo");

AggregatedInfo::AggregatedInfo()
    : m_min(std::numeric_limits<float>::max()),
      m_max(std::numeric_limits<float>::min()),
      m_avg(0),
      m_sum(0),
      m_counter(0) {};

float
AggregatedInfo::GetMin() const
{
    return m_min;
};

float
AggregatedInfo::GetMax() const
{
    return m_max;
};

float
AggregatedInfo::GetAvg() const
{
    return m_avg;
};

void
AggregatedInfo::UpdateMin(float value)
{
    if (m_min > value)
    {
        m_min = value;
    }
};

void
AggregatedInfo::UpdateMax(float value)
{
    if (m_max < value)
    {
        m_max = value;
    }
};

void
AggregatedInfo::UpdateAverage(float value)
{
    m_counter += 1;
    IncrementSum(value);
    m_avg = m_sum / m_counter;
};

void
AggregatedInfo::UpdateStatistics(AggregatedInfo update)
{
    UpdateMax(update.GetMax());
    UpdateMin(update.GetMin());
    UpdateAverage(update.GetAvg());
}

void
AggregatedInfo::UpdateStatistics(float update)
{
    UpdateMax(update);
    UpdateMin(update);
    UpdateAverage(update);
}

void
AggregatedInfo::IncrementSum(float value)
{
    m_sum += value;
};

void
AggregatedInfo::CustomUpdate(float value, uint index, uint id)
{
}
