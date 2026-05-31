/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 22:00:03
 * @LastEditTime: 2026-05-31 22:02:35
 * @Description:
 */

#ifndef _EXPLORATION_SESSION_TRACKER_H_
#define _EXPLORATION_SESSION_TRACKER_H_

#include <memory>

#include <nav_msgs/Odometry.h>

#include "common_utils/eigen_utils.h"
#include "fastex_msgs/frontierStatistics.h"

namespace data_log_manager
{
/**
 * @brief Snapshot of the current exploration session metrics used for publishing.
 */
struct ExplorationSessionSummary
{
    double cumulative_time;
    double cumulative_distance;
    size_t known_cell_num;
    double known_space_volume;
    eigen_utils::Vec3d last_position;
};

/**
 * @brief Tracks exploration lifecycle state and cumulative runtime metrics.
 */
class ExplorationSessionTracker
{
  public:
    using SharedPtr = std::shared_ptr<ExplorationSessionTracker>;
    using UniquePtr = std::unique_ptr<ExplorationSessionTracker>;

    ExplorationSessionTracker();
    ~ExplorationSessionTracker() = default;

    void reset();
    void startSession(double start_time);
    void finishSession(double finish_time);

    bool hasStarted() const;
    bool hasFinished() const;
    bool isActive() const;

    void updateOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void updateMapStats(const fastex_msgs::frontierStatisticsConstPtr& msg);

    ExplorationSessionSummary buildSummary(double current_time) const;

  private:
    bool explore_start_;
    bool explore_finish_;
    double start_time_;
    double finish_time_;

    double total_distance_;
    eigen_utils::Vec3d last_position_;

    size_t known_cell_num_;
    double known_map_resolution_;
    double known_space_volume_;

    int motion_iteration_num_;
    int map_iteration_num_;
};

} // namespace data_log_manager

#endif // _EXPLORATION_SESSION_TRACKER_H_
