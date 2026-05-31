/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-30 21:02:35
 * @Description:
 */

#include <omp.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "map_process/grid/dynamic_expanding_grid.h"
#include "plan_env/edt_environment.h"
#include "time_utils/time_utils.h"
#include "utils/HPR.h"
#include <vis_utils/marker_utils.h>

namespace map_process
{
void DynamicExpandingGrid::computeGlobalCoverageCostMatrix(const eigen_utils::Vec_Vec3d& position,
                                                           const eigen_utils::Vec_Vec3d& velocity,
                                                           const std::vector<double>& y1,
                                                           Eigen::MatrixXd& matrix)
{
    data_.global_paths_.clear();

    // 1.Initial the cost matrix
    const int drone_num = position.size();
    const int relevant_grid_num = data_.relevant_graph_vertices_.size();
    const int matrix_dim = 1 + drone_num + relevant_grid_num; // 1: virtual depot
    matrix = Eigen::MatrixXd::Zero(matrix_dim, matrix_dim);

    // 2.Fill the cost matrix
    const int large_num = 1e4;
    int grid_start_idx = 1 + drone_num;

    // 2.1 Fill the cost between the virtual depot[0] and drone[1:drone_num+1].
    for (int i = 0; i < drone_num; ++i)
    {
        matrix(0, 1 + i) = -large_num;
        matrix(1 + i, 0) = large_num;
    }

    // 2.2 Fill the cost between the virtual depot[0] and the relevant grid vertices[drone_num+1:].
    for (int i = 0; i < relevant_grid_num; ++i)
    {
        matrix(0, grid_start_idx + i) = large_num;
        matrix(grid_start_idx + i, 0) = 0;
    }

    // 2.3 Fill the cost between the drones[1:drone_num+1].
    for (int i = 0; i < drone_num; ++i)
        for (int j = i + 1; j < drone_num; ++j)
        {
            matrix(1 + i, 1 + j) = large_num;
            matrix(1 + j, 1 + i) = large_num;
        }

    // 2.4 Fill the cost between the drones[1:drone_num+1] and the relevant grid
    // vertices[drone_num+1:].
    double cost;
    eigen_utils::Vec_Vec3d path;

    time_utils::Timer::Ptr timer =
        std::make_shared<time_utils::Timer>("computeCostFromDroneToGrid");
    timer->start();

    for (int i = 0; i < drone_num; ++i)
        for (int j = 0; j < relevant_grid_num; ++j)
        {
            std::tie(cost, path) = computeCostAndPathFromDroneToGrid(
                position[i], data_.relevant_graph_vertices_[j].centroid_, velocity[i], y1[i],
                large_num);

            matrix(1 + i, grid_start_idx + j) = cost;
            matrix(grid_start_idx + j, 1 + i) = 0;

            data_
                .global_cost_matrix_map_[position[i]][data_.relevant_graph_vertices_[j].centroid_] =
                cost;

            // Save the path
            data_.global_paths_[position[i]][data_.relevant_graph_vertices_[j].centroid_] = path;

            std::reverse(path.begin(), path.end());
            data_.global_paths_[data_.relevant_graph_vertices_[j].centroid_][position[i]] = path;
        }

    timer->stop(false, "ms");

    // 2.5 Fill the cost between the relevant grid vertices[drone_num+1:].
    timer.reset(new time_utils::Timer("computeCostFromGridToGrid"));
    timer->start();

    for (int i = 0; i < relevant_grid_num; ++i)
        for (int j = i + 1; j < relevant_grid_num; ++j)
        {
            std::tie(cost, path) = computeCostAndPathFromGridToGrid(
                data_.relevant_graph_vertices_[i].centroid_,
                data_.relevant_graph_vertices_[j].centroid_, large_num);

            matrix(grid_start_idx + i, grid_start_idx + j) = cost;
            matrix(grid_start_idx + j, grid_start_idx + i) = cost;

            data_.global_cost_matrix_map_[data_.relevant_graph_vertices_[i].centroid_]
                                         [data_.relevant_graph_vertices_[j].centroid_] = cost;
            data_.global_cost_matrix_map_[data_.relevant_graph_vertices_[j].centroid_]
                                         [data_.relevant_graph_vertices_[i].centroid_] = cost;

            // Save the path
            data_.global_paths_[data_.relevant_graph_vertices_[i].centroid_]
                               [data_.relevant_graph_vertices_[j].centroid_] = path;

            std::reverse(path.begin(), path.end());
            data_.global_paths_[data_.relevant_graph_vertices_[j].centroid_]
                               [data_.relevant_graph_vertices_[i].centroid_] = path;
        }

    timer->stop(false, "ms");

    // 2.6 Fill the diagonal elements
    for (int i = 0; i < matrix_dim; ++i)
        matrix(i, i) = large_num;
}

void DynamicExpandingGrid::computeIncrementalGlobalCoverageCostMatrix(
    const eigen_utils::Vec_Vec3d& position, const eigen_utils::Vec_Vec3d& velocity,
    const std::vector<double>& y1, const std::vector<std::vector<int>>& generalized_indices,
    std::vector<int>& considered_vertices_indices, Eigen::MatrixXd& matrix)
{
    data_.global_cost_matrix_map_.clear();

    // 1.Initial the cost matrix
    const int drone_num = position.size();

    int considered_grid_num = 0;
    eigen_utils::Vec_Vec3d considered_vertices;
    std::unordered_map<int, int> segment_map;

    for (const auto& indices : generalized_indices)
    {
        if (indices.size() > 1)
        {
            considered_grid_num += 2;
            considered_vertices_indices.push_back(indices.front());
            considered_vertices_indices.push_back(indices.back());
            considered_vertices.push_back(
                data_.relevant_graph_vertices_[indices.front()].centroid_);
            considered_vertices.push_back(data_.relevant_graph_vertices_[indices.back()].centroid_);

            segment_map[std::min(indices.front(), indices.back())] =
                std::max(indices.front(), indices.back());
        }
        else if (indices.empty())
            ROS_ERROR("One of the considered generalized indices is empty!");
        else
        {
            ++considered_grid_num;
            considered_vertices_indices.push_back(indices.front());
            considered_vertices.push_back(
                data_.relevant_graph_vertices_[indices.front()].centroid_);
        }
    }

    const int matrix_dim = 1 + drone_num + considered_grid_num; // 1: virtual depot
    matrix = Eigen::MatrixXd::Zero(matrix_dim, matrix_dim);

    // 2.Fill the cost matrix
    const int large_num = 1e4;
    int grid_start_idx = 1 + drone_num;

    // 2.1 Fill the cost between the virtual depot[0] and drone[1:drone_num+1].
    for (int i = 0; i < drone_num; ++i)
    {
        matrix(0, 1 + i) = -large_num;
        matrix(1 + i, 0) = large_num;
    }

    // 2.2 Fill the cost between the virtual depot[0] and the considered grid
    // vertices[drone_num+1:].
    for (int i = 0; i < considered_grid_num; ++i)
    {
        matrix(0, grid_start_idx + i) = large_num;
        matrix(grid_start_idx + i, 0) = 0;
    }

    // 2.3 Fill the cost between the drones[1:drone_num+1].
    for (int i = 0; i < drone_num; ++i)
        for (int j = i + 1; j < drone_num; ++j)
        {
            matrix(1 + i, 1 + j) = large_num;
            matrix(1 + j, 1 + i) = large_num;
        }

    // 2.4 Fill the cost between the drones[1:drone_num+1] and the considered grid
    // vertices[drone_num+1:].
    double cost;
    eigen_utils::Vec_Vec3d path, best_path;

    time_utils::Timer::Ptr timer =
        std::make_shared<time_utils::Timer>("computeCostFromDroneToGrid");
    timer->start();

    for (int i = 0; i < drone_num; ++i)
        for (int j = 0; j < considered_grid_num; ++j)
        {
            std::tie(cost, path) = computeCostAndPathFromDroneToGrid(
                position[i], considered_vertices[j], velocity[i], y1[i], large_num);
            matrix(1 + i, grid_start_idx + j) = cost;
            matrix(grid_start_idx + j, 1 + i) = 0;

            data_.global_cost_matrix_map_[position[i]][considered_vertices[j]] = cost;

            // Save the path
            data_.global_paths_[position[i]][considered_vertices[j]] = path;

            std::reverse(path.begin(), path.end());
            data_.global_paths_[considered_vertices[j]][position[i]] = path;
        }

    timer->stop(false, "ms");

    // 2.5 Fill the cost between the considered grid vertices[drone_num+1:].
    timer.reset(new time_utils::Timer("computeCostFromGridToGrid"));
    timer->start();

    for (int i = 0; i < considered_grid_num; ++i)
        for (int j = i + 1; j < considered_grid_num; ++j)
        {
            auto it = segment_map.find(
                std::min(considered_vertices_indices[i], considered_vertices_indices[j]));
            if (it != segment_map.end() && it->second == std::max(considered_vertices_indices[i],
                                                                  considered_vertices_indices[j]))
            {
                matrix(grid_start_idx + i, grid_start_idx + j) = -large_num;
                matrix(grid_start_idx + j, grid_start_idx + i) = -large_num;
            }
            else
            {
                std::tie(cost, path) = computeCostAndPathFromGridToGrid(
                    considered_vertices[i], considered_vertices[j], large_num);
                matrix(grid_start_idx + i, grid_start_idx + j) = cost;
                matrix(grid_start_idx + j, grid_start_idx + i) = cost;

                data_.global_cost_matrix_map_[considered_vertices[i]][considered_vertices[j]] =
                    cost;
                data_.global_cost_matrix_map_[considered_vertices[j]][considered_vertices[i]] =
                    cost;

                // Save the path
                data_.global_paths_[considered_vertices[i]][considered_vertices[j]] = path;

                std::reverse(path.begin(), path.end());
                data_.global_paths_[considered_vertices[j]][considered_vertices[i]] = path;
            }
        }

    timer->stop(false, "ms");

    // 2.6 Fill the diagonal elements
    for (int i = 0; i < matrix_dim; ++i)
        matrix(i, i) = large_num;
}

/**
 * @brief Compute the cost from the drone to the grid
 *
 * @param from_pos the position of the drone
 * @param to_pos the centroid of the grid
 * @param inf_cost the default infinite cost
 * @return double
 */
std::pair<double, eigen_utils::Vec_Vec3d> DynamicExpandingGrid::computeCostAndPathFromDroneToGrid(
    const eigen_utils::Vec3d& from_pos, const eigen_utils::Vec3d& to_pos,
    const eigen_utils::Vec3d& velocity, const double y1, const double inf_cost)
{
    const int dist_thre = 80.0;
    double cost = inf_cost;
    eigen_utils::Vec_Vec3d path;

    // If the to_pos is in the exploring state, then search the path from the from_pos to the to_pos
    if (data_.vertices_explore_state_.at(to_pos) == GRID_EXPLORE_STATE::EXPLORING)
    {
        int history_pos_graph_from_id = history_pos_graph_->getVertexId(from_pos.cast<float>());
        int history_pos_graph_to_id = history_pos_graph_->getVertexId(to_pos.cast<float>());

        if (history_pos_graph_from_id < 0 || history_pos_graph_to_id < 0)
        {
            ROS_ERROR("The vertex is not in the history graph!");
            return std::make_pair(cost, path);
        }

        double dist;
        std::vector<int> waypoint_ids;
        eigen_utils::Vec_Vec3f shortest_path;
        history_pos_graph_->findPath(history_pos_graph_from_id, history_pos_graph_to_id,
                                     waypoint_ids, shortest_path, dist);

        if (dist > dist_thre)
        {
            cost = dist;

            path.clear();
            path.reserve(shortest_path.size());
            for (const auto& point : shortest_path)
                path.push_back(point.cast<double>());
        }
        else
        {
            const double no_path_penalty = 1.5;
            eigen_utils::Vec3iSet nbr_indices;
            eigen_utils::Vec3i from_located_index, to_located_index;
            eigen_utils::Vec3d from_min_bound, from_max_bound, to_min_bound, to_max_bound,
                search_box_min, search_box_max;

            // Get the located grid indices of the endpoints.
            const auto& grid = *data_.grid_;
            grid.getPointLocatedGrid(from_pos, from_located_index);
            grid.getPointLocatedGrid(to_pos, to_located_index);

            const bool from_exists = grid.checkGridExisted(from_located_index);
            const bool to_exists = grid.checkGridExisted(to_located_index);
            if (!from_exists || !to_exists)
            {
                ROS_WARN_STREAM_THROTTLE(1.0, "Grid index missing, fallback to coarse path. from="
                                                  << eigen_utils::vec_to_string(from_located_index)
                                                  << ", to="
                                                  << eigen_utils::vec_to_string(to_located_index));
                map_process::PATH_SEARCH_RESULT path_result =
                    path_searcher_->searchCoarsePathWithWSRoadMap(from_pos, to_pos, path, cost,
                                                                  true, params_.max_straight_dist_);
                if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                    cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0,
                                                                                 velocity, y1);
                else
                {
                    cost = (from_pos - to_pos).norm() * no_path_penalty;
                    path.clear();
                }
            }
            else
            {
                grid.getGridMinMaxBox(from_located_index, from_min_bound, from_max_bound);
                grid.getGridMinMaxBox(to_located_index, to_min_bound, to_max_bound);

                // Get the neighbor grid indices of the from_located_index.
                process_utils::ProcessUtils::allNeighbors(from_located_index, nbr_indices);

                // If the to_pos is in the occupied state, then change the to_pos to the nearest
                // point in the grid
                eigen_utils::Vec3d consider_pos = to_pos;

                // If the to_pos is in the neighbor grid, then search the fine path.
                if (nbr_indices.count(to_located_index) > 0)
                {
                    search_box_min = from_pos.cwiseMin(to_pos);
                    search_box_max = from_pos.cwiseMax(to_pos);
                    search_box_min -= eigen_utils::Vec3d(4.0, 4.0, 4.0);
                    search_box_min += eigen_utils::Vec3d(4.0, 4.0, 4.0);

                    map_process::PATH_SEARCH_RESULT result = path_searcher_->searchFineBoundedPath(
                        from_pos, consider_pos, search_box_min, search_box_max, path, cost, -1,
                        true);

                    if (result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                        cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0,
                                                                                     velocity, y1);
                    else
                    {
                        cost = (from_pos - to_pos).norm() * no_path_penalty;
                        path.clear();
                    }
                }
                else // If the to_pos is not in the neighbor grid, then search the coarse path.
                {
                    map_process::PATH_SEARCH_RESULT path_result =
                        path_searcher_->searchCoarsePathWithWSRoadMap(
                            from_pos, to_pos, path, cost, true, params_.max_straight_dist_);

                    if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                        cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0,
                                                                                     velocity, y1);
                    else
                    {
                        cost = (from_pos - to_pos).norm() * no_path_penalty;
                        path.clear();
                    }
                }
            }
        }
    }

    return std::make_pair(cost, path);
}

/**
 * @brief Compute the cost from the grid to the grid
 *
 * @param from_pos the centroid of the from grid
 * @param to_pos the centroid of the to grid
 * @param inf_cost the default infinite cost
 * @return double
 */
std::pair<double, eigen_utils::Vec_Vec3d> DynamicExpandingGrid::computeCostAndPathFromGridToGrid(
    const eigen_utils::Vec3d& from_pos, const eigen_utils::Vec3d& to_pos, const double inf_cost)
{
    double cost = inf_cost;
    eigen_utils::Vec_Vec3d pathd;
    eigen_utils::Vec_Vec3f pathf;

    std::vector<int> waypoint_ids;
    data_.relevant_graph_->findShortestPath(
        data_.relevant_graph_->getVertexId(from_pos.cast<float>()),
        data_.relevant_graph_->getVertexId(to_pos.cast<float>()), waypoint_ids, pathf, cost);

    for (const eigen_utils::Vec3f& point : pathf)
        pathd.push_back(point.cast<double>());

    cost = path_searcher_->calculateMovementCostWithPathSegments(pathd, 5.0);

    return std::make_pair(cost, pathd);
}

void DynamicExpandingGrid::computeLocalSOPCostMatrix(
    const eigen_utils::Vec_Vec3d& cur_pos, const eigen_utils::Vec_Vec3d& cur_vel,
    const std::vector<double>& cur_yaw, const eigen_utils::Vec_Vec3d& cluster_centroids,
    const std::vector<int>& grid_indices, Eigen::MatrixXd& local_matrix,
    std::vector<eigen_utils::Vec_Vec3d>& considered_vertices)
{
    // 1. segment the grid indices into several groups
    std::vector<eigen_utils::Vec_Vec3d> grid_centroids_groups;
    std::vector<std::vector<int>> grid_indices_groups;
    segmentGridIndices(grid_indices, grid_indices_groups, grid_centroids_groups);

    // Visualize the grid centroids (For debugging)
    {
        for (size_t i = 0; i < grid_centroids_groups.size(); ++i)
        {
            const auto& group = grid_centroids_groups[i];
            if (group.size() == 1)
            {
                publishPointsMarker(points_group_pub_, group,
                                    eigen_utils::Vec4f(0.0, 1.0, 0.0, 1.0), 2.0, "world",
                                    "grid_centroids_groups", i);
            }
            else
            {
                publishPointsMarker(points_group_pub_, group,
                                    eigen_utils::Vec4f(0.0, 0.0, 1.0, 1.0), 2.0, "world",
                                    "grid_centroids_groups", i);
            }
        }
        static size_t last_cnt = 0;
        for (size_t i = grid_centroids_groups.size(); i < last_cnt; ++i)
        {
            publishPointsMarker(points_group_pub_, eigen_utils::Vec_Vec3d(),
                                eigen_utils::Vec4f(0.0, 0.0, 0.0, 1.0), 1.0, "world",
                                "grid_centroids_groups", i);
        }
        last_cnt = grid_centroids_groups.size();
    }

    // 2. extract the vertices considered in the local SOP
    considered_vertices = {{cur_pos}};
    for (const auto& pt : cluster_centroids)
        considered_vertices.push_back({pt});
    for (const auto& group : grid_centroids_groups)
        considered_vertices.push_back(group);

    // 3. Compute the local SOP cost matrix
    local_matrix = Eigen::MatrixXd::Zero(considered_vertices.size(), considered_vertices.size());

    const int large_num = 1e4;

    const int drone_num = cur_pos.size();
    const int cluster_num = cluster_centroids.size();
    const int grid_num = grid_centroids_groups.size();

    const int cluster_start_idx = drone_num;
    const int grid_start_idx = drone_num + cluster_num;

    double cost = -1;
    eigen_utils::Vec_Vec3d path;
    map_process::PATH_SEARCH_RESULT path_result;

    // 3.1 For the cost between the drones [0: drone_num]
    for (int i = 0; i < drone_num; ++i)
        for (int j = i + 1; j < drone_num; ++j)
        {
            local_matrix(i, j) = large_num; // large number
            local_matrix(j, i) = large_num;
        }

    // 3.2 For the cost between the drones [0: drone_num] and the cluster centroids [drone_num:
    // drone_num + cluster_num]
    for (int i = 0; i < drone_num; ++i)
        for (int j = 0; j < cluster_num; ++j)
        {
            path_result = path_searcher_->searchFinePath(cur_pos[i], cluster_centroids[j], path,
                                                         cost, -1, false);

            if (path_result == map_process::PATH_SEARCH_RESULT::FAIL)
            {
                path_result = path_searcher_->searchCoarsePathWithWSRoadMap(
                    cur_pos[i], cluster_centroids[j], path, cost, false,
                    params_.max_straight_dist_);
            }
            else
            {
                path_searcher_->optimizePathWithInterpolation(path, params_.max_straight_dist_, 2.0,
                                                              false);
            }

            if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0, cur_vel[i],
                                                                             cur_yaw[i]);
            else
            {
                cost = large_num * 0.1;
                path.clear();

                ROS_ERROR("No path from drone to cluster centroid (SOP)!");
            }

            local_matrix(i, cluster_start_idx + j) = cost;
            local_matrix(cluster_start_idx + j, i) = -1;

            // Save the path
            data_.local_paths_[cur_pos[i]][cluster_centroids[j]] = path;
        }

    // 3.3 For the cost between the drones [0: drone_num] and the grid centroids [drone_num +
    // cluster_num:]

    for (int i = 0; i < drone_num; ++i)
        for (int j = 0; j < grid_num; ++j)
        {
            local_matrix(i, grid_start_idx + j) = large_num; // large number
            local_matrix(grid_start_idx + j, i) = -1;
        }

    // 3.4 For the cost between the cluster centroids [drone_num: drone_num +
    // cluster_centroids.size()]
    for (int i = 0; i < cluster_num; ++i)
        for (int j = i + 1; j < cluster_num; ++j)
        {
            path_result = path_searcher_->searchFinePath(cluster_centroids[i], cluster_centroids[j],
                                                         path, cost, -1, false);

            if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
            {
                path_searcher_->optimizePathWithInterpolation(path, params_.max_straight_dist_, 2.0,
                                                              false);

                cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
            }
            else
            {
                cost = large_num * 0.5;
                path.clear();

                ROS_WARN("No path between two cluster centroids (SOP)!");
            }

            local_matrix(drone_num + i, drone_num + j) = cost;
            local_matrix(drone_num + j, drone_num + i) = cost;

            // Save the path
            data_.local_paths_[cluster_centroids[i]][cluster_centroids[j]] = path;

            std::reverse(path.begin(), path.end());
            data_.local_paths_[cluster_centroids[j]][cluster_centroids[i]] = path;
        }

    // 3.5 For the cost between the cluster centroids [drone_num: drone_num + cluster_num] and the
    // grid centroids [drone_num + cluster_num:]
    time_utils::Timer::Ptr timer =
        std::make_shared<time_utils::Timer>("computeCostFromClusterToGrid");
    timer->start();

    for (int i = 0; i < cluster_num; ++i)
        for (int j = 0; j < grid_num; ++j)
        {
            // calculate the cost from the cluster centroid to the grid centroid
            path_result = path_searcher_->searchCoarsePathWithWSRoadMap(
                cluster_centroids[i], grid_centroids_groups[j][0], path, cost, true,
                params_.max_straight_dist_);

            if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
            {
                cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
            }
            else
            {
                ROS_ERROR("No path between cluster to grid (SOP)!");
                cost = large_num * 0.9;
                path.clear();
            }

            local_matrix(cluster_start_idx + i, grid_start_idx + j) = cost;

            // Save the path
            data_.local_paths_[cluster_centroids[i]][grid_centroids_groups[j][0]] = path;

            // calculate the cost from the grid centroid to the cluster centroid
            if (grid_centroids_groups[j][0].isApprox(grid_centroids_groups[j][1], 1e-3))
            {
                std::reverse(path.begin(), path.end());
            }
            else
            {
                path_result = path_searcher_->searchCoarsePathWithWSRoadMap(
                    grid_centroids_groups[j][1], cluster_centroids[i], path, cost, true,
                    params_.max_straight_dist_);

                if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                    cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
                else
                {
                    cost = large_num * 0.9;
                    path.clear();

                    ROS_ERROR("No path between grid to cluster (SOP)!");
                }
            }

            local_matrix(grid_start_idx + j, cluster_start_idx + i) = cost;

            // Save the path
            data_.local_paths_[grid_centroids_groups[j][1]][cluster_centroids[i]] = path;
        }

    timer->stop(false, "ms");

    // 3.6 For the cost between the grid centroids [drone_num + cluster_num:]

    for (int i = 0; i < grid_num; ++i)
    {
        const auto& source_group = grid_centroids_groups[i][1];
        auto source_iter = data_.global_cost_matrix_map_.find(source_group);

        for (int j = i + 1; j < grid_num; ++j)
        {
            const auto& target_group = grid_centroids_groups[j][0];
            bool cost_found = false;

            if (source_iter != data_.global_cost_matrix_map_.end())
            {
                const auto& target_map = source_iter->second;
                auto target_iter = target_map.find(target_group);

                if (target_iter != target_map.end())
                {
                    cost = target_iter->second;
                    path = data_.global_paths_.at(source_group).at(target_group);
                    cost_found = true;
                }
            }

            if (!cost_found)
            {
                std::tie(cost, path) =
                    computeCostAndPathFromGridToGrid(source_group, target_group, large_num);
            }

            local_matrix(grid_start_idx + i, grid_start_idx + j) = cost;
            local_matrix(grid_start_idx + j, grid_start_idx + i) = -1;

            data_.local_paths_[source_group][target_group] = path;
        }
    }

    // 3.7 For the diagonal elements
    for (int i = 0; i < local_matrix.rows(); ++i)
        local_matrix(i, i) = large_num;
}

/**
 * @brief Find the difference between the current grid and the last grid
 *
 * @param current_grid current relevant grid set
 * @param last_grid last relevant grid set
 * @param removed_grid removed grids
 * @param added_grid added grids
 */
void DynamicExpandingGrid::analyzeGridChanges(const eigen_utils::Vec3iSet& current_grid,
                                              const eigen_utils::Vec3iSet& last_grid,
                                              eigen_utils::Vec3iSet& removed_grid,
                                              eigen_utils::Vec3iSet& added_grid,
                                              eigen_utils::Vec3iSet& unchanged_grid)
{
    // 1.Clear the result grid
    removed_grid.clear();
    added_grid.clear();
    unchanged_grid.clear();

    for (const auto& grid_id : last_grid)
        if (current_grid.count(grid_id) == 0)
            removed_grid.insert(grid_id);
        else
            unchanged_grid.insert(grid_id);

    for (const auto& grid_id : current_grid)
        if (last_grid.count(grid_id) == 0)
            added_grid.insert(grid_id);
}

void DynamicExpandingGrid::findFirstGridCentroids(const std::vector<int>& grid_indices,
                                                  eigen_utils::Vec3dSet<3>& first_grid_centroids)
{
    first_grid_centroids.clear();

    const eigen_utils::Vec3i& first_grid_index =
        data_.relevant_graph_vertices_[grid_indices[0]].index_;

    auto grid_iter = data_.grid_indices_centroids_.find(first_grid_index);
    if (grid_iter != data_.grid_indices_centroids_.end())
    {
        const eigen_utils::Vec_Vec3d& centroids = grid_iter->second;
        first_grid_centroids.insert(centroids.begin(), centroids.end());
    }

    // Visualize the first grid centroids (For debugging)
    publishPointsMarker(first_grid_centroids_pub_, first_grid_centroids,
                        eigen_utils::Vec4f(1.0, 0.0, 0.0, 0.5), 3.0, "world",
                        "first_grid_centroids", 0);
}

void DynamicExpandingGrid::findFirstGridNeighborCentroids(
    const eigen_utils::Vec3dSet<3>& first_grid_centroids,
    eigen_utils::Vec3dSet<3>& first_grid_neighbor_centroids)
{
    first_grid_neighbor_centroids.clear();

    for (const auto& pt : first_grid_centroids)
    {
        int s_id = data_.relevant_graph_->getVertexId(pt.cast<float>());
        if (s_id < 0)
        {
            ROS_ERROR("The point is not in the relevant graph!");
            continue;
        }

        const auto& edges = data_.relevant_graph_->getLinkedEdges(s_id);
        for (const auto& e : edges)
        {
            const int nbr_id = e.first;
            const eigen_utils::Vec3d& nbr_centroid =
                data_.grid_centroids_match_.at(data_.relevant_graph_->getVertexPos(nbr_id));

            if (first_grid_centroids.find(nbr_centroid) == first_grid_centroids.end())
                first_grid_neighbor_centroids.insert(nbr_centroid);
        }
    }

    // Visualize the first grid neighbor centroids (For debugging)
    publishPointsMarker(first_grid_neighbor_centroids_pub_, first_grid_neighbor_centroids,
                        eigen_utils::Vec4f(0.0, 1.0, 0.0, 0.5), 3.0, "world",
                        "first_grid_neighbor_centroids", 0);
}

void DynamicExpandingGrid::segmentGridIndices(
    const std::vector<int>& grid_indices, std::vector<std::vector<int>>& grid_indices_groups,
    std::vector<eigen_utils::Vec_Vec3d>& grid_centroids_groups)
{
    grid_indices_groups.clear();
    grid_centroids_groups.clear();

    // 1.Find the centroids of the first grid.
    eigen_utils::Vec3dSet<3> first_grid_centroids;
    findFirstGridCentroids(grid_indices, first_grid_centroids);

    // 2.Find the neighbors of the first grid.
    eigen_utils::Vec3dSet<3> first_grid_neighbor_centroids;
    findFirstGridNeighborCentroids(first_grid_centroids, first_grid_neighbor_centroids);

    auto addGroup = [&grid_indices_groups, &grid_centroids_groups](
                        const int in_index, const eigen_utils::Vec3d& in_centroid,
                        const int out_index, const eigen_utils::Vec3d& out_centroid) {
        grid_indices_groups.push_back({in_index, out_index});
        grid_centroids_groups.push_back({in_centroid, out_centroid});
    };

    eigen_utils::Vec3dSet<3> reserved_first_grid_neighbor_centroids = first_grid_neighbor_centroids;

    int in_index = -1, out_index = -1;
    eigen_utils::Vec3d in_centroid, out_centroid;

    for (size_t i = 0; i < grid_indices.size(); ++i)
    {
        const int cur_id = grid_indices[i];
        const eigen_utils::Vec3d& cur_centroid = data_.relevant_graph_vertices_[cur_id].centroid_;

        // If it is a member of first_grid_centroids, skip it;
        if (first_grid_centroids.find(cur_centroid) != first_grid_centroids.end())
        {
            continue;
        }

        // End condition: if first_grid_neighbor_centroids is empty, add the current point to
        // point_group and add point_group to grid_centroids_groups;
        if (reserved_first_grid_neighbor_centroids.empty())
        {
            addGroup(cur_id, cur_centroid, cur_id, cur_centroid);
            break;
        }

        // If it is a member of first_grid_neighbor_centroids, add it as a single member to
        // point_group and add this point_group to grid_centroids_groups, then remove it from
        // first_grid_neighbor_centroids;
        if (first_grid_neighbor_centroids.find(cur_centroid) != first_grid_neighbor_centroids.end())
        {
            addGroup(cur_id, cur_centroid, cur_id, cur_centroid);
            reserved_first_grid_neighbor_centroids.erase(cur_centroid);
            continue;
        }

        // If it is not a member of first_grid_neighbor_centroids, check if its previous and next
        // points are members of first_grid_neighbor_centroids. If the previous point is, add the
        // current point as the start point to point_group, and if the next point is, add the
        // current point as the end point to point_group. Skip for all points between these two
        // points. When both the start and end points are found, add this point_group with two
        // points to grid_centroids_groups.
        const eigen_utils::Vec3d& prev_centroid =
            (i > 0) ? data_.relevant_graph_vertices_[grid_indices[i - 1]].centroid_ : cur_centroid;
        const eigen_utils::Vec3d& next_centroid =
            (i < grid_indices.size() - 1)
                ? data_.relevant_graph_vertices_[grid_indices[i + 1]].centroid_
                : cur_centroid;

        bool is_prev_neighbor =
            (first_grid_centroids.find(prev_centroid) != first_grid_centroids.end() ||
             first_grid_neighbor_centroids.find(prev_centroid) !=
                 first_grid_neighbor_centroids.end());
        bool is_next_neighbor =
            (first_grid_centroids.find(next_centroid) != first_grid_centroids.end() ||
             first_grid_neighbor_centroids.find(next_centroid) !=
                 first_grid_neighbor_centroids.end());

        if (is_prev_neighbor)
        {
            in_index = cur_id;
            in_centroid = cur_centroid;
        }

        if (is_next_neighbor)
        {
            out_index = cur_id;
            out_centroid = cur_centroid;

            if (in_index == -1)
                throw std::runtime_error("The in_index is not set!");

            addGroup(in_index, in_centroid, out_index, out_centroid);
        }
    }
}

const DynamicExpandingGridParams& DynamicExpandingGrid::getParams() const { return params_; }

std::vector<RelevantGridAttributes> DynamicExpandingGrid::getRelevantGridVertexSnapshot() const
{
    return data_.relevant_graph_vertices_;
}

const DynamicExpandingGrid::RelevantGraphType&
DynamicExpandingGrid::getRelevantGraphReadonly() const
{
    return *data_.relevant_graph_;
}

bool DynamicExpandingGrid::getCachedGlobalPath(const eigen_utils::Vec3d& from,
                                               const eigen_utils::Vec3d& to,
                                               eigen_utils::Vec_Vec3d& path) const
{
    const auto source_iter = data_.global_paths_.find(from);
    if (source_iter == data_.global_paths_.end())
        return false;

    const auto target_iter = source_iter->second.find(to);
    if (target_iter == source_iter->second.end())
        return false;

    path = target_iter->second;
    return true;
}

bool DynamicExpandingGrid::getCachedGlobalCost(const eigen_utils::Vec3d& from,
                                               const eigen_utils::Vec3d& to, double& cost) const
{
    const auto source_iter = data_.global_cost_matrix_map_.find(from);
    if (source_iter == data_.global_cost_matrix_map_.end())
        return false;

    const auto target_iter = source_iter->second.find(to);
    if (target_iter == source_iter->second.end())
        return false;

    cost = target_iter->second;
    return true;
}

bool DynamicExpandingGrid::getCachedLocalPath(const eigen_utils::Vec3d& from,
                                              const eigen_utils::Vec3d& to,
                                              eigen_utils::Vec_Vec3d& path) const
{
    const auto source_iter = data_.local_paths_.find(from);
    if (source_iter == data_.local_paths_.end())
        return false;

    const auto target_iter = source_iter->second.find(to);
    if (target_iter == source_iter->second.end())
        return false;

    path = target_iter->second;
    return true;
}

bool DynamicExpandingGrid::getRelevantGridAttributeByIndex(int idx,
                                                           RelevantGridAttributes& attr) const
{
    if (idx < 0 || idx >= static_cast<int>(data_.relevant_graph_vertices_.size()))
        return false;

    attr = data_.relevant_graph_vertices_[idx];
    return true;
}

/**
 * @brief Gets the grid indices within a specified range.
 *
 * This function calculates and returns the grid indices that fall within the specified minimum and
 * maximum bounds in the dynamic expanding grid.
 *
 * @param min The minimum bound of the range.
 * @param max The maximum bound of the range.
 * @return eigen_utils::Vec_Vec3i A vector of grid indices within the specified range.
 */
eigen_utils::Vec_Vec3i
DynamicExpandingGrid::getGridIndicesInRange(const eigen_utils::Vec3d& min,
                                            const eigen_utils::Vec3d& max) const
{
    return data_.grid_->getGridIndicesInRange(min, max);
}

/**
 * @brief Get DynamicExpandingGrid Relevant Grid
 *
 * @param relevant_grid relevant grid
 */
void DynamicExpandingGrid::getGridTour(const eigen_utils::Vec3d& current_position,
                                       const std::vector<int>& indices,
                                       eigen_utils::Vec_Vec3d& key_points,
                                       eigen_utils::Vec_Vec3d& path) const
{
    time_utils::Timer::Ptr timer = std::make_shared<time_utils::Timer>("Get Grid Tour");
    timer->start();

    key_points.clear();
    key_points.reserve(indices.size() + 1);
    key_points = {current_position};
    for (const auto& index : indices)
        key_points.push_back(data_.relevant_graph_vertices_.at(index).centroid_);

    path.clear();
    timer->stop(false, "ms");
}

void DynamicExpandingGrid::getRelevantFrontierClusters(const std::vector<int>& grid_indices,
                                                       std::vector<int>& cluster_indices) const
{
    cluster_indices.clear();

    for (const auto& index : grid_indices)
    {
        const auto& vertex = data_.relevant_graph_vertices_.at(index);

        std::unordered_map<int, eigen_utils::Vec3d> clusters_in_grid;
        data_.grid_->getGridFrontierClusters(vertex.index_, clusters_in_grid);
        for (const auto& [id, centroid] : clusters_in_grid)
            cluster_indices.push_back(id);
    }
}

/**
 * @brief Get DynamicExpandingGrid Relevant Markers
 *
 * @param pts1_vec points of the first set
 * @param pts2_vec points of the second set
 */
void DynamicExpandingGrid::getRelevantGridMarkers(
    std::vector<eigen_utils::Vec_Vec3d>& pts1_vec,
    std::vector<eigen_utils::Vec_Vec3d>& pts2_vec) const
{
    pts1_vec.clear();
    pts2_vec.clear();

    eigen_utils::Vec_Vec3d pts1, pts2;
    const auto& grid_ids = data_.current_relevant_grid_;
    for (const auto& grid_id : grid_ids)
    {
        data_.grid_->getGridMarkerWithGridIndex(grid_id, pts1, pts2);
        pts1_vec.push_back(pts1);
        pts2_vec.push_back(pts2);
    }
}

/**
 * @brief Get DynamicExpandingGrid Markers
 *
 * @param ignore_inactive_grid whether to ignore inactive grid
 * @param pts1_vec points of the first set
 * @param pts2_vec points of the second set
 */
void DynamicExpandingGrid::getAllGridMarkers(const bool ignore_inactive_grid,
                                             std::vector<eigen_utils::Vec_Vec3d>& pts1_vec,
                                             std::vector<eigen_utils::Vec_Vec3d>& pts2_vec) const
{
    pts1_vec.clear();
    pts2_vec.clear();

    eigen_utils::Vec_Vec3d pts1, pts2;
    data_.grid_->getAllGridMarkers(ignore_inactive_grid, pts1, pts2);
    pts1_vec.push_back(pts1);
    pts2_vec.push_back(pts2);
}

/**
 * @brief Get the relevant graph markers
 *
 * @param graph_markers graph markers
 * @param vertices_scale vertices scale
 * @param edges_scale edges scale
 * @param vertices_rgba vertices rgba
 * @param edges_rgba edges rgba
 */
void DynamicExpandingGrid::getRelevantGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                                   const double& vertices_scale,
                                                   const double& edges_scale,
                                                   const eigen_utils::Vec_Vec4d& vertices_rgba,
                                                   const eigen_utils::Vec_Vec4d& edges_rgba) const
{
    const ros::Time stamp = ros::Time::now();
    visualization_msgs::Marker vertices = vis_utils::marker_utils::makePointMarker(
        params_.frame_id_, "relevant_graph_vertices", 0,
        vis_utils::marker_utils::makeScale(vertices_scale, vertices_scale, vertices_scale),
        vis_utils::marker_utils::toRosColor(vertices_rgba[0]), visualization_msgs::Marker::ADD,
        stamp);
    visualization_msgs::Marker edges = vis_utils::marker_utils::makeLineListMarker(
        params_.frame_id_, "relevant_graph_edges", 1, edges_scale,
        vis_utils::marker_utils::toRosColor(edges_rgba[0]), visualization_msgs::Marker::ADD, stamp);

    auto snapshot = data_.relevant_graph_->getGraphVisualizationSnapshot();

    vertices.points.reserve(snapshot.vertices.size());
    vertices.colors.reserve(snapshot.vertices.size());
    edges.points.reserve((snapshot.edges.size() + snapshot.temporary_edges.size()) * 2);

    for (const auto& vertex_entry : snapshot.vertices)
    {
        const eigen_utils::Vec3f& vertex = vertex_entry.pos;
        vis_utils::marker_utils::appendPoint(vertices, vertex);

        std_msgs::ColorRGBA color;
        auto vertex_itr = data_.grid_centroids_match_.find(vertex);
        if (vertex_itr != data_.grid_centroids_match_.end())
        {
            switch (data_.vertices_explore_state_.at(vertex_itr->second))
            {
            case GRID_EXPLORE_STATE::UNEXPLORED:
                color = vis_utils::marker_utils::toRosColor(vertices_rgba[0]);
                vertices.colors.push_back(color);
                break;

            case GRID_EXPLORE_STATE::EXPLORING:
                color = vis_utils::marker_utils::toRosColor(vertices_rgba[1]);
                vertices.colors.push_back(color);
                break;

            default:
                break;
            }
        }
    }

    // Build a temporary map for fast lookups
    std::unordered_map<int, eigen_utils::Vec3f> id_pos_map;
    id_pos_map.reserve(snapshot.vertices.size());
    for (const auto& vertex_entry : snapshot.vertices)
    {
        id_pos_map.emplace(vertex_entry.id, vertex_entry.pos);
    }

    for (const auto& edge_entry : snapshot.edges)
    {
        int from_id = edge_entry.from_id;
        int to_id = edge_entry.to_id;

        if (id_pos_map.find(from_id) != id_pos_map.end() &&
            id_pos_map.find(to_id) != id_pos_map.end())
        {
            const auto& from_vertex = id_pos_map.at(from_id);
            const auto& to_vertex = id_pos_map.at(to_id);
            vis_utils::marker_utils::appendLine(edges, from_vertex, to_vertex);
        }
    }

    // Handle temporary edges similarly if needed
    // (Assuming temporary edges are kept separate or handled differently,
    // standard snapshot includes edges. If temporary edges are separate,
    // they might still need the lock or a separate snapshot mechanism,
    // but typically edges are edges.)
    for (const auto& edge_entry : snapshot.temporary_edges)
    {
        int from_id = edge_entry.from_id;
        int to_id = edge_entry.to_id;

        if (id_pos_map.find(from_id) != id_pos_map.end() &&
            id_pos_map.find(to_id) != id_pos_map.end())
        {
            const auto& from_vertex = id_pos_map.at(from_id);
            const auto& to_vertex = id_pos_map.at(to_id);

            vis_utils::marker_utils::appendLine(edges, from_vertex, to_vertex);
        }
    }

    vis_utils::marker_utils::assignMarkers(graph_markers, {vertices, edges});
}

void DynamicExpandingGrid::publishActiveVoxels(ros::Publisher& marker_pub,
                                               openvdb::BoolGrid::ConstPtr grid,
                                               const eigen_utils::Vec3d& voxel_size) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker = vis_utils::marker_utils::makeMarker(
        "world", "active_voxels", 0, visualization_msgs::Marker::CUBE,
        vis_utils::marker_utils::makeScale(voxel_size.x(), voxel_size.y(), voxel_size.z()),
        vis_utils::marker_utils::toRosColor(eigen_utils::Vec4d(0.0, 1.0, 0.0, 0.3)));

    int id = 0;
    for (openvdb::BoolGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
    {
        const openvdb::Coord coord = iter.getCoord();
        marker.pose.position.x = coord.x() * voxel_size.x() + voxel_size.x() / 2.0;
        marker.pose.position.y = coord.y() * voxel_size.y() + voxel_size.y() / 2.0;
        marker.pose.position.z = coord.z() * voxel_size.z() + voxel_size.z() / 2.0;

        marker.id = id++;
        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}

template <typename PointsType>
void DynamicExpandingGrid::publishPointsMarker(ros::Publisher& marker_pub, const PointsType& points,
                                               const eigen_utils::Vec4f& color, const double scale,
                                               const std::string& frame_id, const std::string& ns,
                                               const int id) const
{
    visualization_msgs::Marker marker = vis_utils::marker_utils::makeMarker(
        frame_id, ns, id, visualization_msgs::Marker::SPHERE_LIST,
        vis_utils::marker_utils::makeScale(scale, scale, scale),
        vis_utils::marker_utils::toRosColor(color),
        points.empty() ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD);

    if (points.empty())
    {
        marker_pub.publish(marker);
        return;
    }

    for (const auto& pt : points)
        vis_utils::marker_utils::appendPoint(marker, pt);

    marker_pub.publish(marker);
}

} // namespace map_process
