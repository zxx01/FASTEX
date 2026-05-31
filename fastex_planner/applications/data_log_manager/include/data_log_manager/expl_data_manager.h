/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-14 20:35:41
 * @LastEditTime: 2026-05-31 22:21:08
 * @Description:
 */

#ifndef _EXPL_DATA_MANAGER_H_
#define _EXPL_DATA_MANAGER_H_

#include <ros/ros.h>

#include "std_srvs/Empty.h"
#include <nav_msgs/Odometry.h>

#include "data_log_manager/core/exploration_session_tracker.h"
#include "data_log_manager/logging/time_log_channel.h"
#include "data_log_manager/types/log_data_type.h"
#include "fastex_msgs/DataLog.h"
#include "fastex_msgs/IterationTime.h"
#include "fastex_msgs/frontierStatistics.h"

namespace data_log_manager
{
class ExplorationDataManager
{
  public:
    using SharedPtr = std::shared_ptr<ExplorationDataManager>;
    using UniquePtr = std::unique_ptr<ExplorationDataManager>;

    ExplorationDataManager() {};
    ~ExplorationDataManager() {};

    void initialize(ros::NodeHandle& nh);

  private:
    bool enable_file_logging_;
    ExplorationSessionTracker::UniquePtr session_tracker_;

    std::unique_ptr<ExplorationDataLog> expl_data_log_;
    TimeLogChannel frontier_log_channel_, dynamic_expanding_grid_log_channel_, roadmap_log_channel_,
        explore_preprocess_log_channel_, expl_motion_log_channel_, expl_iteration_time_log_channel_;

    ros::Timer exploration_data_timer_;
    ros::Publisher exploration_data_pub_, explore_iteration_time_pub_, explore_preprocess_time_pub_,
        explore_motion_time_pub_;
    ros::Subscriber odom_sub_, map_info_sub_, explore_planner_time_sub_, frontier_log_sub_,
        dynamic_expanding_grid_log_sub_, roadmap_log_sub_, expl_preprocess_time_sub_,
        expl_motion_log_sub_, expl_iteration_time_sub_;
    ros::ServiceServer init_server_, finish_server_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mapInfoCallback(const fastex_msgs::frontierStatisticsConstPtr& msg);
    void frontierLogCallback(const fastex_msgs::DataLogConstPtr& msg);
    void dynamicExpandingGridLogCallback(const fastex_msgs::DataLogConstPtr& msg);
    void roadMapLogCallback(const fastex_msgs::DataLogConstPtr& msg);
    void explorationPreprocessTimeCallback(const fastex_msgs::DataLogConstPtr& msg);
    void explMotionLogCallback(const fastex_msgs::DataLogConstPtr& msg);
    void explIterationTimeCallback(const fastex_msgs::DataLogConstPtr& msg);

    bool explorationInitCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool explorationFinishCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void handleTimeLogCallback(const fastex_msgs::DataLogConstPtr& msg, TimeLogChannel& channel);
    void explorationDataTimerCallback(const ros::TimerEvent& event);
};

} // namespace data_log_manager

#endif // _EXPL_DATA_MANAGER_H_
