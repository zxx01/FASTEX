/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 22:15:00
 * @LastEditTime: 2026-05-31 22:19:57
 * @Description:
 */

#include "data_log_manager/logging/time_log_channel.h"

namespace data_log_manager
{
TimeLogChannel::TimeLogChannel()
    : enable_file_logging_(false), publish_iteration_time_(false), iteration_time_pub_()
{
}

void TimeLogChannel::initialize(const std::string& dir, const std::string& file_name,
                                bool enable_file_logging, bool publish_iteration_time,
                                const ros::Publisher& iteration_time_pub)
{
    enable_file_logging_ = enable_file_logging;
    publish_iteration_time_ = publish_iteration_time;
    iteration_time_pub_ = iteration_time_pub;

    if (enable_file_logging_)
        log_data_.initializeLog(dir, file_name);
}

bool TimeLogChannel::processLog(const fastex_msgs::DataLogConstPtr& msg,
                                const std::string& frame_id)
{
    if (!log_data_.updateAndWrite(msg->iteration_num, msg->start_time, msg->end_time,
                                  enable_file_logging_))
        return false;

    if (publish_iteration_time_ && iteration_time_pub_)
    {
        fastex_msgs::IterationTime iteration_time;
        iteration_time.header.stamp = ros::Time::now();
        iteration_time.header.frame_id = frame_id;
        iteration_time.iterationTime = log_data_.duration_;
        iteration_time.timeConsumed =
            std::stod(log_data_.end_time_) - std::stod(log_data_.first_start_time_);
        iteration_time_pub_.publish(iteration_time);
    }

    return true;
}

const TimeLogData& TimeLogChannel::data() const { return log_data_; }

} // namespace data_log_manager
