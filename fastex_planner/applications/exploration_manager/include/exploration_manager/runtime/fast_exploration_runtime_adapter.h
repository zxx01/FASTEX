/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 16:02:30
 * @LastEditTime: 2026-05-31 17:47:10
 * @Description:
 */

#ifndef _FAST_EXPLORATION_RUNTIME_ADAPTER_H_
#define _FAST_EXPLORATION_RUNTIME_ADAPTER_H_

#include "common_utils/eigen_utils.h"

#include <memory>

#include <ros/ros.h>

#include "bspline/Bspline.h"
#include "plan_manage/planner_manager.h"
#include "traj_utils/planning_visualization.h"

namespace fastex_explorer
{
class FastexExplorationManager;

/**
 * @brief Adapter that centralizes FSM runtime side effects.
 *
 * This layer owns the blocking ROS service clients and the asynchronous
 * visualization trigger so that the FSM can focus on state transitions and
 * timing logic.
 */
class FastExplorationRuntimeAdapter
{
  public:
    using UniquePtr = std::unique_ptr<FastExplorationRuntimeAdapter>;

    FastExplorationRuntimeAdapter() = default;
    ~FastExplorationRuntimeAdapter() = default;

    /**
     * @brief Initialize service clients and visualization dependencies.
     *
     * @param nh ROS node handle used to create service clients.
     * @param fastex_expl_manager Shared exploration manager used for map/planning visualization.
     * @param planner_manager Shared planner manager used for trajectory visualization.
     * @param visualization Shared visualization helper used to publish markers.
     */
    void init(ros::NodeHandle& nh,
              const std::shared_ptr<FastexExplorationManager>& fastex_expl_manager,
              const std::shared_ptr<fast_planner::FastPlannerManager>& planner_manager,
              const std::shared_ptr<fast_planner::PlanningVisualization>& visualization);

    /**
     * @brief Send the latest B-spline trajectory to the downstream execution service.
     *
     * @param bspline Trajectory message prepared by the exploration planner.
     * @return true if the service call succeeded.
     * @return false otherwise.
     */
    bool pushBsplineToServer(const bspline::Bspline& bspline);

    /**
     * @brief Notify the logging subsystem that exploration has started.
     *
     * @return true if the service call succeeded.
     * @return false otherwise.
     */
    bool notifyExplorationStart();

    /**
     * @brief Notify the logging subsystem that exploration has finished.
     *
     * @return true if the service call succeeded.
     * @return false otherwise.
     */
    bool notifyExplorationFinish();

    /**
     * @brief Publish the task-complete signal when exploration reaches the finish state.
     */
    void publishTaskComplete() const;

    /**
     * @brief Publish the current odometry position as a trajectory marker sample.
     *
     * @param odom_pos Current robot position.
     */
    void publishTrajectorySample(const eigen_utils::Vec3d& odom_pos);

    /**
     * @brief Trigger asynchronous planning and trajectory visualization updates.
     */
    void triggerVisualizationAsync() const;

  private:
    /// Shared exploration manager used for preprocessing and planning markers.
    std::shared_ptr<FastexExplorationManager> fastex_expl_manager_;
    /// Shared planner manager used to access the current executed trajectory.
    std::shared_ptr<fast_planner::FastPlannerManager> planner_manager_;
    /// Shared visualization helper used for runtime markers.
    std::shared_ptr<fast_planner::PlanningVisualization> visualization_;

    /// Service client for forwarding the generated B-spline trajectory.
    ros::ServiceClient bspline_client_;
    /// Service client for notifying exploration start.
    ros::ServiceClient expl_data_start_client_;
    /// Service client for notifying exploration finish.
    ros::ServiceClient expl_data_finish_client_;
    /// Publisher for the task completion signal.
    ros::Publisher task_complete_pub_;
    /// Publisher for the sampled executed trajectory marker.
    ros::Publisher traj_pub_;
    /// Monotonic marker id used for trajectory sample publishing.
    int traj_marker_id_{1};
};
} // namespace fastex_explorer

#endif
