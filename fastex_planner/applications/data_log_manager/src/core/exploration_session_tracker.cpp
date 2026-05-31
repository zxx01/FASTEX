/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 22:00:03
 * @LastEditTime: 2026-05-31 22:03:30
 * @Description:
 */

#include "data_log_manager/core/exploration_session_tracker.h"

#include <cmath>

namespace data_log_manager
{
ExplorationSessionTracker::ExplorationSessionTracker() { reset(); }

void ExplorationSessionTracker::reset()
{
    explore_start_ = false;
    explore_finish_ = false;
    start_time_ = 0.0;
    finish_time_ = 0.0;

    total_distance_ = 0.0;
    last_position_ = eigen_utils::Vec3d::Zero();

    known_cell_num_ = 0;
    known_map_resolution_ = 0.0;
    known_space_volume_ = 0.0;

    motion_iteration_num_ = 0;
    map_iteration_num_ = 0;
}

void ExplorationSessionTracker::startSession(double start_time)
{
    start_time_ = start_time;
    explore_start_ = true;
}

void ExplorationSessionTracker::finishSession(double finish_time)
{
    finish_time_ = finish_time;
    explore_finish_ = true;
}

bool ExplorationSessionTracker::hasStarted() const { return explore_start_; }

bool ExplorationSessionTracker::hasFinished() const { return explore_finish_; }

bool ExplorationSessionTracker::isActive() const { return explore_start_ && !explore_finish_; }

void ExplorationSessionTracker::updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    eigen_utils::Vec3d cur_position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
    if (isActive())
        total_distance_ += (cur_position - last_position_).norm();
    last_position_ = cur_position;
}

void ExplorationSessionTracker::updateMapStats(const fastex_msgs::frontierStatisticsConstPtr& msg)
{
    if (!isActive())
        return;

    known_cell_num_ = msg->known_cell_num;
    known_map_resolution_ = msg->resolution;
    known_space_volume_ = known_cell_num_ * std::pow(known_map_resolution_, 3);
}

ExplorationSessionSummary ExplorationSessionTracker::buildSummary(double current_time) const
{
    return ExplorationSessionSummary{current_time - start_time_, total_distance_, known_cell_num_,
                                     known_space_volume_, last_position_};
}

} // namespace data_log_manager
