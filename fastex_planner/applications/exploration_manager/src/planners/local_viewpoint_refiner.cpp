/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:35
 * @LastEditTime: 2026-05-30 22:32:26
 * @Description:
 */

#include <tuple>
#include <unordered_map>

#include <ros/ros.h>

#include "exploration_manager/planners/local_viewpoint_refiner.h"
#include "utils/plan_graph_ikdtree.h"

namespace fastex_explorer
{
LocalViewpointRefiner::LocalViewpointRefiner(
    const map_process::PathSearcher::SharedPtr& path_searcher, const double straight_max_dist)
    : path_searcher_(path_searcher), straight_max_dist_(straight_max_dist)
{
}

bool LocalViewpointRefiner::refineTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, const std::vector<eigen_utils::Vec_Vec3d>& n_points,
    const std::vector<std::vector<std::optional<double>>>& n_yaws,
    eigen_utils::Vec_Vec3d& refined_pts, std::vector<std::optional<double>>& refined_yaws,
    eigen_utils::Vec_Vec3d& local_refined_tour) const
{
    refined_pts.clear();
    refined_yaws.clear();
    local_refined_tour.clear();
    const auto& searcher_params = path_searcher_->getParams();

    using RefinedGraphVertexType = map_process::IkdTreePlanGraph<void, void>::VertexType;
    using RefinedGraphEdgeType = map_process::IkdTreePlanGraph<void, void>::EdgeType;
    map_process::IkdTreePlanGraph<void, void> refined_graph(false);

    int first_node_id = -1, final_node_id = -1;
    std::vector<int> last_group, cur_group;
    eigen_utils::Vec_Vec3d path;
    double cost;
    map_process::PATH_SEARCH_RESULT path_res;
    std::unordered_map<int, std::pair<int, int>> vertex_id_to_n_points_idx;
    std::unordered_map<int, std::unordered_map<int, eigen_utils::Vec_Vec3d>> local_paths;

    bool has_valid_first_segment = false;

    std::tie(first_node_id, std::ignore) =
        refined_graph.addVertex(RefinedGraphVertexType(cur_pos.cast<float>(), true), false);
    last_group.push_back(first_node_id);

    for (size_t i = 0; i < n_points.size(); ++i)
    {
        for (size_t j = 0; j < n_points[i].size(); ++j)
        {
            auto [cur_v_id, success] = refined_graph.addVertex(
                RefinedGraphVertexType(n_points[i][j].cast<float>(), true), false);
            if (!success)
            {
                ROS_ERROR("Failed to add vertex to refined graph.");
                continue;
            }

            vertex_id_to_n_points_idx[cur_v_id] = std::make_pair(i, j);

            for (size_t k = 0; k < last_group.size(); ++k)
            {
                eigen_utils::Vec3d last_pt;
                if (i == 0)
                    last_pt = cur_pos;
                else
                {
                    auto [last_group_i, last_group_j] = vertex_id_to_n_points_idx.at(last_group[k]);
                    last_pt = n_points[last_group_i][last_group_j];
                }

                path_res = path_searcher_->searchFinePath(last_pt, n_points[i][j], path, cost, -1.0,
                                                          false);
                if (path_res == map_process::PATH_SEARCH_RESULT::SUCCESS)
                {
                    path_searcher_->optimizePathWithInterpolation(path, straight_max_dist_, 2.0,
                                                                  false);
                }

                local_paths[last_group[k]][cur_v_id] = path;

                if (i == 0)
                {
                    if (path_res == map_process::PATH_SEARCH_RESULT::SUCCESS)
                        has_valid_first_segment = true;

                    cost = path_searcher_->calculateMovementCostWithPathSegments(
                        path, 5.0, cur_vel, cur_yaw[0], n_yaws[0][j].value());
                }
                else if (i == 1)
                {
                    if (!has_valid_first_segment)
                        return false;

                    auto [last_group_i, last_group_j] = vertex_id_to_n_points_idx.at(last_group[k]);
                    cost = path_searcher_->calculateMovementCostWithPathSegments(
                        path, 5.0, searcher_params.vm_ * (last_pt - cur_pos).normalized(),
                        n_yaws[last_group_i][last_group_j], std::nullopt);
                }
                else
                {
                    cost = path_searcher_->calculateMovementCostWithPathSegments(path, 5.0);
                }

                refined_graph.addOneWayEdge(last_group[k], cur_v_id,
                                            RefinedGraphEdgeType(cost, true));
            }

            cur_group.push_back(cur_v_id);

            if (i == n_points.size() - 1)
            {
                final_node_id = cur_v_id;
                break;
            }
        }

        last_group.swap(cur_group);
        cur_group.clear();
    }

    std::vector<int> optimal_vertex_indices;
    bool success =
        refined_graph.DijkstraSearch(first_node_id, final_node_id, optimal_vertex_indices, cost);
    if (!success)
    {
        ROS_ERROR("Failed to find optimal local cluster path.");
        return false;
    }

    for (size_t k = 1; k < optimal_vertex_indices.size(); ++k)
    {
        auto [i, j] = vertex_id_to_n_points_idx.at(optimal_vertex_indices[k]);
        refined_pts.push_back(n_points[i][j]);
        refined_yaws.push_back(n_yaws[i][j]);

        const auto& path_temp =
            local_paths.at(optimal_vertex_indices[k - 1]).at(optimal_vertex_indices[k]);
        local_refined_tour.insert(local_refined_tour.end(), path_temp.begin(), path_temp.end());
    }

    return true;
}
} // namespace fastex_explorer
