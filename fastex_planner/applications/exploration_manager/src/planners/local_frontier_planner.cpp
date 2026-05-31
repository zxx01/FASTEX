/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 17:21:12
 * @LastEditTime: 2026-05-31 17:27:21
 * @Description:
 */

#include <limits>

#include <ros/ros.h>

#include "exploration_manager/planners/global_coverage_planner.h"
#include "exploration_manager/planners/local_frontier_planner.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"

namespace fastex_explorer
{
namespace
{
const struct LocalFrontierPlannerTimerLogFlags
{
    bool compute_local_cluster_cost_matrix{false};
    bool solve_local_cluster_tsplkh{false};
    bool compute_local_sop_cost_matrix{false};
    bool refine_local_tour{false};
} kTimerLogFlags;
} // namespace

LocalFrontierPlanner::LocalFrontierPlanner(
    const map_process::FrontierManager::SharedPtr& frontier_manager,
    const map_process::DynamicExpandingGrid::SharedPtr& dynamic_expanding_grid,
    const map_process::PathSearcher::SharedPtr& path_searcher, LkhInterface& lkh_interface,
    const double straight_max_dist, const std::string& tsp_dir, const int drone_id)
    : frontier_manager_(frontier_manager), dynamic_expanding_grid_(dynamic_expanding_grid),
      path_searcher_(path_searcher), lkh_interface_(lkh_interface),
      local_viewpoint_refiner_(
          std::make_unique<LocalViewpointRefiner>(path_searcher, straight_max_dist)),
      straight_max_dist_(straight_max_dist), tsp_dir_(tsp_dir), drone_id_(drone_id)
{
}

PLAN_RESULT LocalFrontierPlanner::planFrontierTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
    const std::vector<int>& optimal_grid_indices,
    const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
    LocalFrontierPlan& plan) const
{
    plan = LocalFrontierPlan{};

    PLAN_RESULT res = PLAN_RESULT::SUCCEED;
    if (optimal_grid_indices.size() == 1)
    {
        res = planLocalFrontierClustersTour(cur_pos, cur_vel, cur_yaw, cluster_indices,
                                            top_viewpoints, {}, plan.refine_cluster_indices,
                                            plan.cluster_tour);
        if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
            return res;

        plan.default_target_pos = top_viewpoints[plan.refine_cluster_indices[0]].first;
    }
    else
    {
        res = planLocalFrontierClustersTourBySOP(
            cur_pos, cur_vel, cur_yaw, cluster_indices, top_viewpoints, optimal_grid_indices,
            plan.refine_cluster_indices, plan.cluster_tour, plan.default_target_pos);
        if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
            return res;
    }

    double first_target_dist = 0.0;
    for (size_t i = 0; i + 1 < plan.cluster_tour.size(); ++i)
    {
        first_target_dist += (plan.cluster_tour[i + 1] - plan.cluster_tour[i]).norm();
        if ((plan.cluster_tour[i + 1] - plan.default_target_pos).norm() < 0.01 ||
            (plan.cluster_tour[i] - plan.default_target_pos).norm() < 0.01 ||
            first_target_dist > 20.0)
            break;
    }

    if (first_target_dist >= 20.0)
        return PLAN_RESULT::SUCCEED;

    eigen_utils::Vec3d refined_target_pos = plan.default_target_pos;
    bool refined = false;
    res = refineLocalTour(cur_pos, cur_vel, cur_yaw, plan.refine_cluster_indices,
                          plan.default_target_pos, plan.cluster_tour, plan.refined_tour,
                          refined_target_pos, refined);
    if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
        return res;

    if (refined)
    {
        plan.default_target_pos = refined_target_pos;
        plan.used_refined_tour = true;
    }

    return PLAN_RESULT::SUCCEED;
}

PLAN_RESULT LocalFrontierPlanner::planLocalFrontierClustersTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
    const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
    const eigen_utils::Vec_Vec3d& grid_pos, std::vector<int>& indices,
    eigen_utils::Vec_Vec3d& local_cluster_tour) const
{
    indices.clear();

    time_utils::Timer timer("Compute Local Cluster CostMatrix");
    timer.start();

    Eigen::MatrixXd cost_matrix;
    computeFinalLocalClustersCostMatrix({cur_pos}, {cur_vel}, {cur_yaw}, cluster_indices,
                                        top_viewpoints, grid_pos, cost_matrix);
    timer.stop(kTimerLogFlags.compute_local_cluster_cost_matrix, "ms");

    if (cluster_indices.empty())
    {
        ROS_WARN("No frontier cluster.");
        return PLAN_RESULT::NO_FRONTIER;
    }

    timer = time_utils::Timer("solve Local Cluster TSPLKH");
    timer.start();

    const int drone_num = 1;
    const std::string file_name = tsp_dir_ + "/drone_local_" + std::to_string(drone_id_);

    std::vector<int> temp_indices;
    if (!lkh_interface_.solveATSP(file_name, cost_matrix, temp_indices))
    {
        ROS_ERROR("Failed to solve local cluster ATSP with LKH.");
        return PLAN_RESULT::FAIL;
    }

    const int valid_start_idx = 2 + drone_num;
    GlobalCoveragePlanner::extractTourIndices(temp_indices, valid_start_idx, indices);

    if (!grid_pos.empty())
        indices.pop_back();

    for (size_t i = 0; i < indices.size(); ++i)
        indices[i] = cluster_indices[indices[i]];

    double dist = 0.0;
    bool optimistic = false;
    local_cluster_tour.clear();
    map_process::PATH_SEARCH_RESULT res = path_searcher_->searchFinePath(
        cur_pos, top_viewpoints[indices[0]].first, local_cluster_tour, dist, -1, optimistic);

    if (res == map_process::PATH_SEARCH_RESULT::FAIL)
    {
        path_searcher_->searchCoarsePathWithWSRoadMap(cur_pos, top_viewpoints[indices[0]].first,
                                                      local_cluster_tour, dist, optimistic,
                                                      straight_max_dist_);
    }
    else
    {
        path_searcher_->optimizePathWithInterpolation(local_cluster_tour, straight_max_dist_, 2.0,
                                                      optimistic);
    }

    for (size_t i = 0; i + 1 < indices.size(); ++i)
    {
        eigen_utils::Vec_Vec3d path;
        res = path_searcher_->searchFinePath(top_viewpoints[indices[i]].first,
                                             top_viewpoints[indices[i + 1]].first, path, dist, -1,
                                             false);
        path_searcher_->optimizePathWithInterpolation(path, straight_max_dist_, 2.0, optimistic);
        local_cluster_tour.insert(local_cluster_tour.end(), path.begin(), path.end());
    }

    if (!grid_pos.empty())
        local_cluster_tour.push_back(grid_pos[0]);

    timer.stop(kTimerLogFlags.solve_local_cluster_tsplkh, "ms");
    return PLAN_RESULT::SUCCEED;
}

PLAN_RESULT LocalFrontierPlanner::planLocalFrontierClustersTourBySOP(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
    const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
    const std::vector<int>& optimal_grid_indices, std::vector<int>& refine_cluster_indices,
    eigen_utils::Vec_Vec3d& optimal_local_tours, eigen_utils::Vec3d& default_target_pos) const
{
    optimal_local_tours.clear();
    refine_cluster_indices.clear();

    const int drone_num = 1;

    eigen_utils::Vec_Vec3d cluster_top_vpoints;
    for (const int idx : cluster_indices)
        cluster_top_vpoints.emplace_back(top_viewpoints[idx].first);

    if (cluster_top_vpoints.empty())
    {
        ROS_WARN("No frontier cluster.");
        return PLAN_RESULT::FAIL;
    }

    Eigen::MatrixXd local_sop_matrix;
    time_utils::Timer timer("Compute Local SOP CostMatrix");
    timer.start();

    std::vector<eigen_utils::Vec_Vec3d> considered_vertices;
    dynamic_expanding_grid_->computeLocalSOPCostMatrix({cur_pos}, {cur_vel}, {cur_yaw[0]},
                                                       cluster_top_vpoints, optimal_grid_indices,
                                                       local_sop_matrix, considered_vertices);
    timer.stop(kTimerLogFlags.compute_local_sop_cost_matrix, "ms");

    const std::string file_name = tsp_dir_ + "/drone_local_SOP_" + std::to_string(drone_id_);
    std::vector<int> temp_indices, optimal_local_indices;
    if (!lkh_interface_.solveSOP(file_name, local_sop_matrix, temp_indices))
    {
        ROS_ERROR("Failed to solve local SOP with LKH.");
        return PLAN_RESULT::FAIL;
    }

    const int valid_start_idx = drone_num;
    GlobalCoveragePlanner::extractTourIndices(temp_indices, valid_start_idx, optimal_local_indices);

    const int cluster_offset = drone_num;
    std::unordered_set<int> local_cluster_indices_set;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
        local_cluster_indices_set.insert(static_cast<int>(i) + cluster_offset);

    for (size_t i = 1; i < optimal_local_indices.size(); ++i)
    {
        if (local_cluster_indices_set.find(optimal_local_indices[i]) !=
            local_cluster_indices_set.end())
            refine_cluster_indices.push_back(
                cluster_indices[optimal_local_indices[i] - cluster_offset]);
        else
            break;
    }

    eigen_utils::Vec3d start_pos, end_pos;
    for (size_t i = 0; i < optimal_local_indices.size(); ++i)
    {
        optimal_local_tours.insert(optimal_local_tours.end(),
                                   considered_vertices[optimal_local_indices[i]].begin(),
                                   considered_vertices[optimal_local_indices[i]].end());

        if (i == optimal_local_indices.size() - 1)
            break;

        start_pos = considered_vertices[optimal_local_indices[i]].size() == 1
                        ? considered_vertices[optimal_local_indices[i]][0]
                        : considered_vertices[optimal_local_indices[i]][1];
        end_pos = considered_vertices[optimal_local_indices[i + 1]][0];

        eigen_utils::Vec_Vec3d path;
        if (!dynamic_expanding_grid_->getCachedLocalPath(start_pos, end_pos, path))
        {
            ROS_ERROR("Failed to fetch cached local SOP path.");
            return PLAN_RESULT::FAIL;
        }

        optimal_local_tours.insert(optimal_local_tours.end(), path.begin(), path.end());
    }

    default_target_pos = considered_vertices[optimal_local_indices[1]][0];
    return PLAN_RESULT::SUCCEED;
}

PLAN_RESULT LocalFrontierPlanner::refineLocalTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, const std::vector<int>& refine_cluster_indices,
    const eigen_utils::Vec3d& default_target_pos, eigen_utils::Vec_Vec3d& global_tour,
    eigen_utils::Vec_Vec3d& refined_tour, eigen_utils::Vec3d& refined_target_pos,
    bool& refined) const
{
    refined = false;
    refined_tour.clear();

    time_utils::Timer timer("Refine Local Tour");
    timer.start();

    std::vector<eigen_utils::Vec_Vec3d> n_points;
    std::vector<std::vector<std::optional<double>>> n_yaws;
    eigen_utils::Vec_Vec3d refined_pts;
    std::vector<std::optional<double>> refined_yaws;

    frontier_manager_->getViewpointsForClusterIndices(refine_cluster_indices, n_points);
    if (n_points.empty())
    {
        ROS_ERROR("No frontier points for refinement.");
        return PLAN_RESULT::NO_FRONTIER;
    }

    for (size_t i = 0; i < n_points.size(); ++i)
    {
        std::vector<std::optional<double>> yaws;
        if (i == 0)
        {
            for (const eigen_utils::Vec3d& pt : n_points[i])
                yaws.push_back(process_utils::ProcessUtils::calculateYaw(cur_pos, pt));
        }
        else
        {
            yaws.assign(n_points[i].size(), std::nullopt);
        }

        n_yaws.push_back(std::move(yaws));
    }

    if (local_viewpoint_refiner_->refineTour(cur_pos, cur_vel, cur_yaw, n_points, n_yaws,
                                             refined_pts, refined_yaws, refined_tour))
    {
        global_tour = refined_tour;
        refined_target_pos = refined_pts.front();
        refined = true;
    }
    else
    {
        refined_target_pos = default_target_pos;
    }

    timer.stop(kTimerLogFlags.refine_local_tour, "ms");
    return PLAN_RESULT::SUCCEED;
}

void LocalFrontierPlanner::computeFinalLocalClustersCostMatrix(
    const eigen_utils::Vec_Vec3d& cur_pos, const eigen_utils::Vec_Vec3d& cur_vel,
    const eigen_utils::Vec_Vec3d& cur_yaw, const std::vector<int>& cluster_indices,
    const std::vector<std::pair<eigen_utils::Vec3d, double>>& top_viewpoints,
    const eigen_utils::Vec_Vec3d& grid_pos, Eigen::MatrixXd& final_cost_matrix) const
{
    double cost = 0.0;
    eigen_utils::Vec_Vec3d path;
    map_process::PATH_SEARCH_RESULT result;

    const int drone_num = cur_pos.size();
    const int cluster_num = cluster_indices.size();
    int dimension = 1 + drone_num + cluster_num;
    if (!grid_pos.empty())
        dimension += 1;

    final_cost_matrix = Eigen::MatrixXd::Zero(dimension, dimension);

    const double large_num = map_process::constants::kPathCostFallback;
    const int cluster_offset = 1 + drone_num;

    for (int i = 0; i < drone_num; ++i)
    {
        final_cost_matrix(0, i + 1) = -large_num;
        final_cost_matrix(i + 1, 0) = large_num;
    }

    for (int i = 0; i < cluster_num; ++i)
    {
        final_cost_matrix(0, i + cluster_offset) = large_num;
        final_cost_matrix(i + cluster_offset, 0) = 0;
    }

    for (int i = 0; i < drone_num; ++i)
        for (int j = 0; j < drone_num; ++j)
            final_cost_matrix(i + 1, j + 1) = large_num;

    for (int i = 0; i < drone_num; ++i)
    {
        for (int j = 0; j < cluster_num; ++j)
        {
            const eigen_utils::Vec3d& from_vp = cur_pos[i];
            const eigen_utils::Vec3d& to_vp = top_viewpoints[cluster_indices[j]].first;

            result = path_searcher_->searchFinePath(from_vp, to_vp, path, cost, -1, false);
            if (result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0, cur_vel[i],
                                                                             cur_yaw[i][0]);
            else
                cost = large_num;

            final_cost_matrix(i + 1, j + cluster_offset) = cost;
            final_cost_matrix(j + cluster_offset, i + 1) = 0;
        }
    }

    for (int i = 0; i < cluster_num; ++i)
    {
        for (int j = i + 1; j < cluster_num; ++j)
        {
            const eigen_utils::Vec3d& from_vp = top_viewpoints[cluster_indices[i]].first;
            const eigen_utils::Vec3d& to_vp = top_viewpoints[cluster_indices[j]].first;

            result = path_searcher_->searchFinePath(from_vp, to_vp, path, cost, -1, false);
            if (result == map_process::PATH_SEARCH_RESULT::SUCCESS)
            {
                final_cost_matrix(i + cluster_offset, j + cluster_offset) =
                    path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
                final_cost_matrix(j + cluster_offset, i + cluster_offset) =
                    final_cost_matrix(i + cluster_offset, j + cluster_offset);
            }
            else
            {
                final_cost_matrix(i + cluster_offset, j + cluster_offset) = large_num;
                final_cost_matrix(j + cluster_offset, i + cluster_offset) = large_num;
            }
        }
    }

    for (int i = 0; i < dimension; ++i)
        final_cost_matrix(i, i) = large_num;

    if (grid_pos.empty())
        return;

    final_cost_matrix(0, dimension - 1) = large_num;
    final_cost_matrix(dimension - 1, 0) = -large_num;

    for (int i = 0; i < drone_num; ++i)
    {
        final_cost_matrix(i + 1, dimension - 1) = large_num;
        final_cost_matrix(dimension - 1, i + 1) = large_num;
    }

    for (int i = 0; i < cluster_num; ++i)
    {
        const auto& from_vp = top_viewpoints[cluster_indices[i]].first;
        const auto& to_vp = grid_pos[0];

        result = path_searcher_->searchFinePath(from_vp, to_vp, path, cost, -1, true);
        if (result == map_process::PATH_SEARCH_RESULT::SUCCESS)
            cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
        else
            cost = (from_vp - to_vp).norm() * 2.0;

        final_cost_matrix(i + cluster_offset, dimension - 1) = cost;
        final_cost_matrix(dimension - 1, i + cluster_offset) = cost;
    }
}
} // namespace fastex_explorer
