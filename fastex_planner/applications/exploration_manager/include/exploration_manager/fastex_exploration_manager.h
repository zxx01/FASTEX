/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-28 21:26:16
 * @LastEditTime: 2026-05-05 00:00:54
 * @Description:
 */
#ifndef _FASTEX_EXPLORATION_MANAGER_H_
#define _FASTEX_EXPLORATION_MANAGER_H_

#include <memory>
#include <optional>
#include <ros/ros.h>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "exploration_manager/expl_data.h"
#include "exploration_manager/planners/global_coverage_planner.h"
#include "exploration_manager/planners/lkh_interface.h"
#include "exploration_manager/planners/local_frontier_planner.h"
#include "exploration_manager/planning_types.h"
#include "map_process/core/map_process.h"
#include "map_process/searcher/path_searcher.h"
#include "plan_env/edt_environment.h"
#include "plan_manage/planner_manager.h"
#include "traj_utils/planning_visualization.h"

namespace fastex_explorer
{
using fast_planner::EDTEnvironment;
using fast_planner::FastPlannerManager;
using fast_planner::SDFMap;

/**
 * @brief High-level planning state used by the exploration FSM.
 */
enum class PLAN_STATE : int8_t
{
    /// No active planning request.
    IDLE,
    /// Trigger a fresh target selection from global exploration context.
    GLOBAL_DECISION,
    /// Reuse the current global tour and make a local target update.
    LOCAL_DECISION,
    /// Replan only the trajectory toward the current target.
    TRAJECTORY_PLANNING,
};

/**
 * @brief High-level exploration phase.
 */
enum class EXPL_PHASE : int8_t
{
    /// Normal exploration mode.
    EXPL,
    /// Return-to-home mode after frontier exhaustion.
    HOME
};

/**
 * @brief Top-level coordinator for FASTEX exploration planning.
 *
 * The manager assembles preprocessing modules and planner helpers, reacts to
 * FSM requests, and keeps the runtime exploration data synchronized with the
 * currently selected planning phase.
 */
class FastexExplorationManager
{
  public:
    using SharedPtr = std::shared_ptr<FastexExplorationManager>;
    using UniquePtr = std::unique_ptr<FastexExplorationManager>;

    FastexExplorationManager() {};
    ~FastexExplorationManager() {};

    /**
     * @brief Initialize planner modules, preprocessing modules, and runtime publishers.
     */
    void initialize(ros::NodeHandle& nh);

    /**
     * @brief Update the current robot position tracked by the manager and preprocessing stack.
     */
    void setCurrentPosition(const eigen_utils::Vec3d& pos);
    /**
     * @brief Switch the high-level exploration phase.
     */
    void transiteExplorationPhase(const EXPL_PHASE& phase);
    /**
     * @brief Get the current high-level exploration phase.
     */
    EXPL_PHASE getExplorationPhase();

    /**
     * @brief Refresh preprocessing outputs required by the next planning cycle.
     */
    void updateExplorationPreprocessData(const eigen_utils::Vec3d& pos);

    /**
     * @brief Run the requested planning stage and update exploration outputs.
     *
     * @param pos Current position.
     * @param vel Current velocity.
     * @param acc Current acceleration.
     * @param yaw Current yaw state.
     * @param plan_state High-level planning request issued by the FSM.
     * @return PLAN_RESULT Planning outcome.
     */
    PLAN_RESULT planExploreMotion(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& vel,
                                  const eigen_utils::Vec3d& acc, const eigen_utils::Vec3d& yaw,
                                  const PLAN_STATE& plan_state);

    /**
     * @brief Publish current preprocessing and planning visualization artifacts.
     */
    void visualizePlanningData();

    /// Shared exploration runtime data updated across planning cycles.
    std::shared_ptr<ExplorationData> ed_;
    /// Shared exploration parameters loaded from ROS.
    std::shared_ptr<ExplorationParams> ep_;
    /// Shared downstream trajectory planner manager.
    std::shared_ptr<FastPlannerManager> planner_manager_;

    /// Preprocessing stack used to maintain frontier, grid, roadmap, and history data.
    map_process::MapProcess::UniquePtr map_process_;
    /// Shared path search helper reused by home planning and preprocessing modules.
    map_process::PathSearcher::SharedPtr path_searcher_;

  private:
    std::shared_ptr<EDTEnvironment> edt_environment_;
    std::shared_ptr<SDFMap> sdf_map_;

    /// Helper responsible for global coverage tour generation.
    GlobalCoveragePlanner::UniquePtr global_coverage_planner_;
    /// Shared LKH adapter used by the planner helpers.
    LkhInterface::UniquePtr lkh_interface_;
    /// Helper responsible for local frontier ordering and refinement.
    LocalFrontierPlanner::UniquePtr local_frontier_planner_;
    std::shared_ptr<fast_planner::PlanningVisualization> visualization_;

    /// Cached index used when truncating the current tour into an intermediate target.
    int last_tour_id_;
    /// Latest robot position seen by the manager.
    eigen_utils::Vec3d cur_pos_;
    /// Current high-level exploration phase.
    EXPL_PHASE expl_phase_;
    /// Preferred path search mode used for return-home planning.
    map_process::PATH_SEARCH_TYPE home_path_search_type_;

    /// Whether the next global planning cycle should use a full replan.
    bool refresh_global_;

    ros::Publisher incremental_global_planning_pub_, expl_preprocess_time_pub_,
        expl_motion_log_pub_;

    /**
     * @brief Load exploration-specific ROS parameters.
     */
    void loadParamsFromROS(ros::NodeHandle& nh);

    /**
     * @brief Ensure the current planning position is represented in the whole-state roadmap.
     */
    void addCurrentPositionToRoadMap(const eigen_utils::Vec3d& pos);

    /**
     * @brief Select the next exploration target position and yaw.
     */
    PLAN_RESULT planExploreTargetPoint(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& vel,
                                       const eigen_utils::Vec3d& yaw,
                                       eigen_utils::Vec3d& target_pos, double& target_yaw,
                                       const PLAN_STATE& plan_state);
    /**
     * @brief Generate a trajectory toward the selected exploration target.
     */
    PLAN_RESULT planExploreTrajectory(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& vel,
                                      const eigen_utils::Vec3d& acc, const eigen_utils::Vec3d& yaw,
                                      const eigen_utils::Vec3d& target_pos,
                                      const double& target_yaw);

    /**
     * @brief Plan a path back to the configured home position.
     */
    PLAN_RESULT planHomePath(const eigen_utils::Vec3d& pos, eigen_utils::Vec3d& home_pos,
                             eigen_utils::Vec_Vec3d& path);

    /**
     * @brief Refresh global top viewpoints for all active frontier clusters.
     */
    void updateGlobalTopViewpoints();
    /**
     * @brief Publish debug markers for incremental global replanning partitions.
     */
    void visualizeIncrementalGlobalPlanning(
        const GlobalCoveragePlanner::IncrementalVisualizationData& vis_data);

    /**
     * @brief Truncate a polyline tour into the next intermediate target pose.
     */
    void extractIntemediateTarget(eigen_utils::Vec_Vec3d& path,
                                  const eigen_utils::Vec3d& default_target_pos,
                                  const double dist_thresh, const bool reset_start_idx,
                                  eigen_utils::Vec3d& target_pos, double& target_yaw);

    /**
     * @brief Build a sphere marker from a list of positions.
     */
    visualization_msgs::Marker drawSpheres(const eigen_utils::Vec_Vec3d& list, const double& scale,
                                           const eigen_utils::Vec4f& color, const std::string& ns,
                                           const int& id);
    /**
     * @brief Build a line-list marker from paired position lists.
     */
    visualization_msgs::Marker drawLines(const eigen_utils::Vec_Vec3d& list1,
                                         const eigen_utils::Vec_Vec3d& list2, const double& scale,
                                         const eigen_utils::Vec4f& color, const string& ns,
                                         const int& id);
};

} // namespace fastex_explorer

#endif
