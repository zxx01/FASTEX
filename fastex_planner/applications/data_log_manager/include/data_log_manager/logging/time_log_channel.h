/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 22:15:00
 * @LastEditTime: 2026-05-31 22:18:54
 * @Description:
 */

#ifndef _TIME_LOG_CHANNEL_H_
#define _TIME_LOG_CHANNEL_H_

#include <memory>
#include <string>

#include <ros/ros.h>

#include "data_log_manager/types/log_data_type.h"
#include "fastex_msgs/DataLog.h"
#include "fastex_msgs/IterationTime.h"

namespace data_log_manager
{
/**
 * @brief Encapsulates one runtime time-log stream and its optional iteration-time publisher.
 */
class TimeLogChannel
{
  public:
    using SharedPtr = std::shared_ptr<TimeLogChannel>;
    using UniquePtr = std::unique_ptr<TimeLogChannel>;

    TimeLogChannel();
    ~TimeLogChannel() = default;

    void initialize(const std::string& dir, const std::string& file_name, bool enable_file_logging,
                    bool publish_iteration_time = false,
                    const ros::Publisher& iteration_time_pub = ros::Publisher());

    bool processLog(const fastex_msgs::DataLogConstPtr& msg, const std::string& frame_id = "world");

    const TimeLogData& data() const;

  private:
    bool enable_file_logging_;
    bool publish_iteration_time_;
    TimeLogData log_data_;
    ros::Publisher iteration_time_pub_;
};

} // namespace data_log_manager

#endif // _TIME_LOG_CHANNEL_H_
