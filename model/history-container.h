#ifndef HISTORY_CONTAINER_H
#define HISTORY_CONTAINER_H

#include "aggregated-info.h"

#include "ns3/ai-module.h"
#include "ns3/core-module.h"

#include <cassert>
#include <chrono>
#include <climits>
#include <cstdint>
#include <iostream>
#include <sys/types.h>
#include <vector>

namespace ns3
{

/**
 * @ingroup defiance
 * @class TimestampedData
 * @brief A helper class to store OpenGymDictContainer data with a timestamp.
 * The timestamp is recorded both at simulation time and system time. One can choose whether to
 * track the ns3 simulation time, because it adds runtime overhead (for 100k obs ~ 0.1s).
 */
class TimestampedData
{
  public:
    Ptr<OpenGymDictContainer> data;
    std::chrono::system_clock::time_point timestamp;
    int64_t ns3timestamp;

    /**
     * @brief Creates a new TimestampedData object.
     *
     * @param data contains the dictionary with the observation/reward data as key-value pairs of
     * strings and OpenGymBoxContainers.
     * @param trackNs3Time boolean to decide whether the ns3 simulation time should be tracked.
     */
    TimestampedData(Ptr<OpenGymDictContainer> data, bool trackNs3Time);
};

/**
 * @ingroup defiance
 * @class TimestampedDataDeque
 * @brief A class to store observation/reward data with timestamps in a deque. The class provides
 * basic methods for manipulation of the deque.
 */
class TimestampedDataDeque
{
  public:
    /**
     * @brief Add a new data entry to the deque.
     */
    void Push(TimestampedData value);

    /**
     * @brief Remove the \c count newest data entries from the deque.
     */
    void PopNewest(uint count = 1);

    /**
     * @brief Remove the \c count oldest data entries from the deque.
     */
    void PopOldest(uint count = 1);

    /**
     * @brief Get the \c count oldest data entries from the deque.
     * @return a vector of data entries.
     */
    std::vector<TimestampedData*> GetOldest(uint count = 1);

    /**
     * @brief Get the \c count newest data entries from the deque.
     * @return a vector of data entries.
     */
    std::vector<TimestampedData*> GetNewest(uint count = 1);

    /**
     * @brief Get all data entries from the deque.
     * @return a vector of data entries.
     */
    std::vector<TimestampedData*> GetAll();

    /**
     * @brief Clear the deque.
     */
    void Clear();

    /**
     * @return the size of the deque.
     */
    uint Size() const;

  private:
    std::deque<TimestampedData> m_deque;
};

/**
 * @ingroup defiance
 * @class HistoryContainer
 * @brief The main datastructure to store observation/reward data from different observation/reward
 * spaces.
 * It stores different deques for each observation/reward space \c m_deques and a history deque
 * containing all data \c m_historydeque. The number of last data to store each observation/reward
 * space deque is defined by \c m_historyLimit and the number of observation/reward spaces is
 * defined by \c m_dequeNumber. The class provides methods to manage the data.
 */
class HistoryContainer
{
  public:
    /**
     * @brief Creates a new \c HistoryContainer object.
     * @param m_historyLimit the number data entries to store in each observation/reward space deque
     * before overwriting the oldest data.
     * @param trackNs3Time boolean to decide whether the ns3 simulation time should be tracked with
     * every data entry.
     */
    HistoryContainer(uint historyLimit, bool trackNs3Time = false);
    ~HistoryContainer();

    /**
     * @brief Retrieve the number of different histories in the container.
     * @return number of history deques.
     */
    uint GetHistoryCount() const;

    /**
     * @brief Push data into a certain history deque. If no history with the given ID exists, a new
     * history is created.
     * @param data the data to push.
     * @param id the ID of the history deque data should be pushed to.
     */
    void Push(Ptr<OpenGymDictContainer> data, uint id);

    /**
     * @brief Aggregate the latest \c n data entries of the specified history deque.
     * @param n the number of data entries to aggregate.
     * @param id the ID of the history deque.
     * @return a map with the keys of the dictionary and the aggregated information for each key.
     */
    std::map<std::string, AggregatedInfo> AggregateNewest(uint id, uint n = 1);

    /**
     * @brief Aggregate the latest data entry of all history deques.
     * @return the newest data entry.
     */
    TimestampedData* GetNewestOfCombinedHistory();

    /**
     * @brief Retrieve the latest \c n data entries of all history deques.
     * @param n the number of data entries to return.
     * @return a vector with the newest \c n data entries.
     */
    std::vector<TimestampedData*> GetNewestOfCombinedHistory(uint n);

    /**
     * @brief Retrieve the latest \c n collected data entries of one history deque.
     * @param n the number of data entries to return.
     * @param id the ID of the history deque.
     * @return a vector with the newest \c n data entries.
     */
    std::vector<TimestampedData*> GetNewestByID(uint id, uint n);

    /**
     * @brief Retrieve the latest data entry of a history deque.
     * @param id the ID of the history deque.
     * @return the newest data entry of the history deque.
     */
    TimestampedData* GetNewestByID(uint id);

    /**
     * @brief Retrieve the size of a certain history deque.
     * @param id the ID of the history deque.
     * @return the size of the specified history deque.
     */
    uint GetSize(uint id);

    /**
     * @brief Retrieve the size of the combined history deque. The size can be larger than the sum
     * of the sizes of the individual history deques, because the combined history deque contains
     * at most \c m_historyLength * \c m_historyCount data entries. Thus, if some history deques
     * receive data entries more frequently than others, the combined history deque can contain
     * more entries of that deque than \c m_historyLength.
     * @return the size of the combined history deque.
     */
    uint GetSizeOfHistory();

    /**
     * @brief Print type and size information of each history deque.
     * @param where the stream to print the data entries to.
     */
    void Print(std::ostream& where);

    /**
     * @brief Print all stored data entries of a certain history deque.
     * @param where the stream to print the data entries to
     * @param id the ID of the history deque.
     * @param type the space type of the data entry. Currently only \c ns3_ai_gym::Box is supported
     * with data of type \c uint, \c float, \c uint32_t or \c double.
     */
    void PrintHistory(std::ostream& where, uint id, ns3_ai_gym::SpaceType type = ns3_ai_gym::Box);

    /**
     * @brief Delete an existing history if it exists.
     * @param id th ID of the history deque that should be deleted.
     */
    void DeleteHistory(uint id);

    /**
     * @brief Check whether a certain history deque exists without an assertion.
     * @param id the ID of the history deque.
     * @return \c true if the history exists, \c false otherwise.
     */
    bool HistoryExists(uint id);

  private:
    uint m_historyCount;  //!< Amount of history deques
    uint m_historyLength; //!< Length of each history deque
    bool m_trackNs3Time;  //!< whether the ns3 sim time is tracked for all history deques

    TimestampedDataDeque m_combinedHistory;           //!< History deque containing all recent data
    std::map<uint, TimestampedDataDeque> m_histories; //!< Different deques for each history deque

    /**
     * @brief Assert that the history deque with the given ID exists. Throw an NS_ASSERT_MSG() if
     * the history does not exist.
     * @param id the ID of the history.
     * @return \c true if the history exists, \c false otherwise.
     */
    bool AssertHistoryExists(uint id);

    /**
     * @brief Aggregates OpenGymBoxContainers and returns the aggregated information for each box.
     * @param dict contains data as key-value pairs of strings and OpenGymBoxContainers to aggregate
     * @return a map with the dictionary keys and the aggregated information for each key
     */
    std::map<std::string, AggregatedInfo> GetInfoFromDict(Ptr<OpenGymDictContainer> dict);
};
} // namespace ns3
#endif
