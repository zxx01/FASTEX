/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 17:21:12
 * @LastEditTime: 2026-05-31 17:27:01
 * @Description:
 */

#ifndef _LOCAL_FRONTIER_PLANNER_H_
#define _LOCAL_FRONTIER_PLANNER_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "common_utils/eigen_utils.h"
#include "exploration_manager/planners/lkh_interface.h"
#include "exploration_manager/planners/local_viewpoint_refiner.h"
#include "exploration_manager/planning_types.h"
#include "map_process/frontier/frontier_manager.h"
#include "map_process/grid/dynamic_expanding_grid.h"
#include "map_process/searcher/path_searcher.h"

namespace fastex_explorer
{
/**
 * @brief Aggregated result of local frontier planning.
 *
 * This keeps the manager-side orchestration lightweight by packaging the
 * selected cluster order, generated tour, and optional refined tour together.
 */
struct LocalFrontierPlan
{
    /// Ordered frontier cluster ids selected for the next local visit sequence.
    std::vector<int> refine_cluster_indices;
    /// Raw local cluster tour before optional viewpoint refinement.
    eigen_utils::Vec_Vec3d cluster_tour;
    /// Refined tour generated from viewpoint candidates when refinement succeeds.
    eigen_utils::Vec_Vec3d refined_tour;
    /// Immediate target used by the manager to truncate the global tour.
    eigen_utils::Vec3d default_target_pos;
    /// Whether @ref refined_tour should replace @ref cluster_tour.
    bool used_refined_tour{false};
};

/**
 * @brief Planner responsible for local frontier-cluster sequencing and refinement.
 *
 * It encapsulates local ATSP/SOP construction, LKH invocation, fine/coarse path
 * stitching between viewpoints, and short-range viewpoint refinement.
 */
class LocalFrontierPlanner
{
  public:
    using UniquePtr = std::unique_ptr<LocalFrontierPlanner>;

    /**
     * @brief Construct the local frontier planner.
     *
     * @param frontier_manager Frontier source used to fetch clustered viewpoints.
     * @param dynamic_expanding_grid Grid model used for SOP matrices and cached local paths.
     * @param path_searcher Path search utility used for local path stitching.
     * @param lkh_interface Shared LKH wrapper used for ATSP/SOP solving.
     * @param straight_max_dist Interpolation spacing used during path smoothing.
     * @param tsp_dir Directory where temporary LKH problem files are written.
     * @param drone_id Active robot id used to derive unique LKH file names.
     */
    LocalFrontierPlanner(const map_process::FrontierManager::SharedPtr& frontier_manager,
                         const map_process::DynamicExpandingGrid::SharedPtr& dynamic_expanding_grid,
                         const map_process::PathSearcher::SharedPtr& path_searcher,
                         LkhInterface& lkh_interface, const double straight_max_dist,
                         const std::string& tsp_dir, const int drone_id);
    ~LocalFrontierPlanner() = default;

    /**
     * @brief Plan the local frontier visit order for the next global grid decision.
     *
     * Depending on the number of remaining global grids, this uses either a
     * simple local ATSP or an SOP constrained by the upcoming global grid order.
     * When the first target is sufficiently close, an additional local viewpoint
     * refinement stage is attempted.
     *
     * @param cur_pos Current vehicle position.
     * @param cur_vel Current vehicle velocity.
     * @param cur_yaw Current vehicle yaw state.
     * @param cluster_indices Active frontier cluster ids inside the first global grid.
     * @param optimal_grid_indices Current global grid order.
     * @param top_viewpoints Cluster representative viewpoints indexed by cluster id.
     * @param plan Output local planning result bundle.
     * @return PLAN_RESULT Planning outcome.
     */
    PLAN_RESULT
    planFrontierTour(const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
                     const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
                     const std::vector<int>& optimal_grid_indices,
                     const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
                     LocalFrontierPlan& plan) const;

  private:
    /**
     * @brief Solve a local ATSP over frontier clusters in the first global grid.
     */
    PLAN_RESULT planLocalFrontierClustersTour(
        const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
        const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
        const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
        const eigen_utils::Vec_Vec3d& grid_pos, std::vector<int>& indices,
        eigen_utils::Vec_Vec3d& local_cluster_tour) const;
    /**
     * @brief Solve a local SOP constrained by the planned order of global grids.
     */
    PLAN_RESULT planLocalFrontierClustersTourBySOP(
        const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
        const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
        const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
        const std::vector<int>& optimal_grid_indices, std::vector<int>& refine_cluster_indices,
        eigen_utils::Vec_Vec3d& optimal_local_tours, eigen_utils::Vec3d& default_target_pos) const;
    /**
     * @brief Refine the early part of a local cluster tour using dense viewpoint candidates.
     *
     * @param cur_pos Current vehicle position.
     * @param cur_vel Current vehicle velocity.
     * @param cur_yaw Current yaw state.
     * @param refine_cluster_indices Ordered cluster ids selected for refinement.
     * @param default_target_pos Fallback immediate target before refinement.
     * @param global_tour In-place tour updated when refinement succeeds.
     * @param refined_tour Output refined tour polyline.
     * @param refined_target_pos Output refined immediate target.
     * @param refined Whether refinement succeeded and should replace the raw local tour.
     * @return PLAN_RESULT Refinement outcome.
     */
    PLAN_RESULT refineLocalTour(const eigen_utils::Vec3d& cur_pos,
                                const eigen_utils::Vec3d& cur_vel,
                                const eigen_utils::Vec3d& cur_yaw,
                                const std::vector<int>& refine_cluster_indices,
                                const eigen_utils::Vec3d& default_target_pos,
                                eigen_utils::Vec_Vec3d& global_tour,
                                eigen_utils::Vec_Vec3d& refined_tour,
                                eigen_utils::Vec3d& refined_target_pos, bool& refined) const;
    /**
     * @brief Build the ATSP cost matrix used for local frontier-cluster ordering.
     */
    void computeFinalLocalClustersCostMatrix(
        const eigen_utils::Vec_Vec3d& cur_pos, const eigen_utils::Vec_Vec3d& cur_vel,
        const eigen_utils::Vec_Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
        const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
        const eigen_utils::Vec_Vec3d& grid_pos, Eigen::MatrixXd& final_cost_matrix) const;

    /// Frontier source used to fetch cluster viewpoints for local planning.
    map_process::FrontierManager::SharedPtr frontier_manager_;
    /// Grid model used to build SOP problems and query cached local paths.
    map_process::DynamicExpandingGrid::SharedPtr dynamic_expanding_grid_;
    /// Path search helper used for local path stitching and smoothing.
    map_process::PathSearcher::SharedPtr path_searcher_;
    /// Shared LKH wrapper used for ATSP/SOP solving.
    LkhInterface& lkh_interface_;
    /// Dense local viewpoint refiner used for short-range local improvements.
    LocalViewpointRefiner::UniquePtr local_viewpoint_refiner_;

    /// Interpolation spacing used when smoothing local paths.
    double straight_max_dist_;
    /// Temporary LKH workspace.
    std::string tsp_dir_;
    /// Active robot id for namespacing LKH problems.
    int drone_id_;
};
} // namespace fastex_explorer

#endif // _LOCAL_FRONTIER_PLANNER_H_
