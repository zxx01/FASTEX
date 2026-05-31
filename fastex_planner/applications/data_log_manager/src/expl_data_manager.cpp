/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-14 20:36:02
 * @LastEditTime: 2026-05-31 22:21:26
 * @Description:
 */
#include <ros/package.h>

#include "data_log_manager/expl_data_manager.h"
#include "fastex_msgs/ExploredVolumeTravedDistTime.h"

#include "time_utils/time_utils.h"

namespace data_log_manager
{
void ExplorationDataManager::initialize(ros::NodeHandle& nh)
{
    nh.param("enable_file_logging", enable_file_logging_, true);
    session_tracker_ = std::make_unique<ExplorationSessionTracker>();

    exploration_data_timer_ = nh.createTimer(
        ros::Duration(1.0), &ExplorationDataManager::explorationDataTimerCallback, this);

    // Publish the exploration data to visualization.
    exploration_data_pub_ =
        nh.advertise<fastex_msgs::ExploredVolumeTravedDistTime>("exploration_data", 2);

    // Subscribe the odometry, map info and explore planner time to calculate the exploration data
    // to be published.
    odom_sub_ = nh.subscribe("odometry", 5, &ExplorationDataManager::odomCallback, this);
    map_info_sub_ = nh.subscribe("map_info", 5, &ExplorationDataManager::mapInfoCallback, this);

    // Service server to receive the start and finish time of the exploration.
    init_server_ = nh.advertiseService("explorer_start",
                                       &ExplorationDataManager::explorationInitCallback, this);
    finish_server_ = nh.advertiseService("explorer_finish",
                                         &ExplorationDataManager::explorationFinishCallback, this);

    /*-------------------------------------Log And Vis---------------------------------------*/

    // Load log directory from ROS parameter
    std::string logs_dir_path;
    if (!nh.param<std::string>("logs_dir_path", logs_dir_path,
                               ros::package::getPath("data_log_manager") +
                                   "/exploration_data_files"))
    {
        ROS_WARN("Failed to load logs_dir_path from ROS parameter server, use default path: %s",
                 logs_dir_path.c_str());
    }
    logs_dir_path = logs_dir_path + "/" + time_utils::Timer::getCurrentTimeString();

    expl_data_log_ = std::make_unique<ExplorationDataLog>();

    // Publishers
    explore_iteration_time_pub_ =
        nh.advertise<fastex_msgs::IterationTime>("explore_iteration_time", 2);
    explore_preprocess_time_pub_ =
        nh.advertise<fastex_msgs::IterationTime>("explore_preprocess_time", 2);
    explore_motion_time_pub_ = nh.advertise<fastex_msgs::IterationTime>("explore_motion_time", 2);

    frontier_log_channel_.initialize(logs_dir_path + "/ExplorationMapProcess",
                                     "frontier_vp_process_time.txt", enable_file_logging_);
    dynamic_expanding_grid_log_channel_.initialize(logs_dir_path + "/ExplorationMapProcess",
                                                   "dhgrid_process_time.txt", enable_file_logging_);
    roadmap_log_channel_.initialize(logs_dir_path + "/ExplorationMapProcess",
                                    "roadmap_process_time.txt", enable_file_logging_);
    explore_preprocess_log_channel_.initialize(logs_dir_path + "/ExplorationMapProcess",
                                               "explore_preprocess_time.txt", enable_file_logging_,
                                               true, explore_preprocess_time_pub_);
    expl_motion_log_channel_.initialize(logs_dir_path + "/ExplorationMotion",
                                        "expl_motion_time.txt", enable_file_logging_, true,
                                        explore_motion_time_pub_);
    expl_iteration_time_log_channel_.initialize(logs_dir_path + "/ExplorationTotalData",
                                                "expl_iteration_time.txt", enable_file_logging_,
                                                true, explore_iteration_time_pub_);

    // Set up file logs only when requested. The node can still publish visualization topics
    // without writing files.
    if (enable_file_logging_)
        expl_data_log_->initializeLog(logs_dir_path + "/ExplorationTotalData",
                                      "exploration_data.txt");

    frontier_log_sub_ =
        nh.subscribe("frontier_log", 10, &ExplorationDataManager::frontierLogCallback, this);
    dynamic_expanding_grid_log_sub_ =
        nh.subscribe("dynamic_expanding_grid_log", 10,
                     &ExplorationDataManager::dynamicExpandingGridLogCallback, this);
    roadmap_log_sub_ =
        nh.subscribe("road_map_log", 10, &ExplorationDataManager::roadMapLogCallback, this);

    expl_preprocess_time_sub_ =
        nh.subscribe("expl_preprocess_time", 10,
                     &ExplorationDataManager::explorationPreprocessTimeCallback, this);
    expl_motion_log_sub_ =
        nh.subscribe("expl_motion_log", 10, &ExplorationDataManager::explMotionLogCallback, this);
    expl_iteration_time_sub_ = nh.subscribe(
        "expl_iteration_time", 10, &ExplorationDataManager::explIterationTimeCallback, this);
}

void ExplorationDataManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    session_tracker_->updateOdom(msg);
}

void ExplorationDataManager::mapInfoCallback(const fastex_msgs::frontierStatisticsConstPtr& msg)
{
    session_tracker_->updateMapStats(msg);
}

bool ExplorationDataManager::explorationInitCallback(std_srvs::Empty::Request& req,
                                                     std_srvs::Empty::Response& res)
{
    session_tracker_->startSession(time_utils::Timer::getTimeNow("us") / 1e6);
    return true;
}

bool ExplorationDataManager::explorationFinishCallback(std_srvs::Empty::Request& req,
                                                       std_srvs::Empty::Response& res)
{
    session_tracker_->finishSession(time_utils::Timer::getTimeNow("us") / 1e6);
    return true;
}

void ExplorationDataManager::handleTimeLogCallback(const fastex_msgs::DataLogConstPtr& msg,
                                                   TimeLogChannel& channel)
{
    if (!session_tracker_->isActive())
        return;

    channel.processLog(msg);
}

void ExplorationDataManager::explorationDataTimerCallback(const ros::TimerEvent& event)
{
    if (!session_tracker_->isActive())
        return;

    const auto summary = session_tracker_->buildSummary(time_utils::Timer::getTimeNow("us") / 1e6);

    fastex_msgs::ExploredVolumeTravedDistTime msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.exploredVoxelNum = summary.known_cell_num;
    msg.exploredVolume = summary.known_space_volume;
    msg.travelDist = summary.cumulative_distance;
    msg.timeConsumed = summary.cumulative_time;
    msg.odom_x = summary.last_position(0);
    msg.odom_y = summary.last_position(1);
    msg.odom_z = summary.last_position(2);
    exploration_data_pub_.publish(msg);

    expl_data_log_->updateAndWrite(msg.timeConsumed, msg.travelDist, msg.exploredVoxelNum,
                                   msg.exploredVolume, msg.odom_x, msg.odom_y, msg.odom_z,
                                   enable_file_logging_);
}

void ExplorationDataManager::frontierLogCallback(const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, frontier_log_channel_);
}

void ExplorationDataManager::dynamicExpandingGridLogCallback(
    const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, dynamic_expanding_grid_log_channel_);
}

void ExplorationDataManager::roadMapLogCallback(const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, roadmap_log_channel_);
}

void ExplorationDataManager::explorationPreprocessTimeCallback(
    const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, explore_preprocess_log_channel_);
}

void ExplorationDataManager::explMotionLogCallback(const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, expl_motion_log_channel_);
}

void ExplorationDataManager::explIterationTimeCallback(const fastex_msgs::DataLogConstPtr& msg)
{
    handleTimeLogCallback(msg, expl_iteration_time_log_channel_);
}

} // namespace data_log_manager
