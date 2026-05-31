/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:34
 * @LastEditTime: 2026-05-31 17:21:12
 * @Description:
 */

#include <algorithm>
#include <functional>
#include <limits>
#include <list>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <ros/ros.h>

#include "exploration_manager/planners/global_coverage_planner.h"
#include "map_process/core/map_process_constants.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"

namespace fastex_explorer
{
namespace
{
const struct GlobalCoveragePlannerTimerLogFlags
{
    bool compute_grid_cost_matrix{false};
    bool solve_grid_tsplkh{false};
} kTimerLogFlags;
} // namespace

GlobalCoveragePlanner::GlobalCoveragePlanner(
    const map_process::DynamicExpandingGrid::SharedPtr& dynamic_expanding_grid,
    LkhInterface& lkh_interface, const std::string& tsp_dir, const int drone_id)
    : dynamic_expanding_grid_(dynamic_expanding_grid), lkh_interface_(lkh_interface),
      tsp_dir_(tsp_dir), drone_id_(drone_id)
{
}

PLAN_RESULT GlobalCoveragePlanner::planCoverageTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, bool& refresh_global,
    eigen_utils::Vec_Vec3d& global_grid_tour, std::vector<int>& optimal_indices,
    IncrementalVisualizationData* vis_data)
{
    const double divide_range = 40.0;
    const int segment_pt_num = 5;

    if (vis_data != nullptr)
        vis_data->clear();

    if (refresh_global)
    {
        ROS_WARN("Plan Global Grid Tour Fully.");

        const PLAN_RESULT res =
            planFullCoverageTour(cur_pos, cur_vel, cur_yaw, global_grid_tour, optimal_indices);
        if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
            return res;

        last_optimal_grid_indices_ = optimal_indices;
        last_relevant_graph_vertices_data_ = dynamic_expanding_grid_->getRelevantGridVertexSnapshot();
        refresh_global = false;
        return PLAN_RESULT::SUCCEED;
    }

    Eigen::MatrixXd global_matrix;

    std::vector<int> independent_pt_ids;
    std::vector<std::vector<int>> segment_pt_ids;
    eigen_utils::Vec3iSet local_grid_indices_set;
    defineLocalRangeAndPartitionNodes(cur_pos, divide_range, segment_pt_num, independent_pt_ids,
                                      segment_pt_ids, local_grid_indices_set, vis_data);

    std::vector<int> independent_reserved_indices;
    std::vector<std::vector<int>> segment_reserved_indices;
    std::vector<int> independent_added_indices;
    std::vector<int> segment_added_indices;
    std::vector<map_process::RelevantGridAttributes> cur_relevant_graph_vertices_data;
    processAddedAndDeletedNodes(independent_pt_ids, segment_pt_ids, independent_reserved_indices,
                                segment_reserved_indices, independent_added_indices,
                                segment_added_indices, local_grid_indices_set,
                                cur_relevant_graph_vertices_data);

    std::vector<std::vector<int>> planning_considered_indices;
    extractNodesForPlanning(independent_reserved_indices, independent_added_indices,
                            segment_reserved_indices, planning_considered_indices);

    std::vector<int> considered_vertices_indices;
    buildIncrementalCoverageCostMatrix(cur_pos, cur_vel, cur_yaw, planning_considered_indices,
                                       considered_vertices_indices, global_matrix);

    const int drone_num = 1;
    const int dimension = global_matrix.rows();
    if (dimension <= 1 + drone_num)
    {
        ROS_WARN("No Grid.");
        return PLAN_RESULT::NO_FRONTIER;
    }

    const std::string file_name = tsp_dir_ + "/drone_grid_incremental_" + std::to_string(drone_id_);

    std::vector<int> temp_indices, lkh_result;
    if (!lkh_interface_.solveATSP(file_name, global_matrix, temp_indices))
    {
        ROS_ERROR("Failed to solve incremental global ATSP with LKH.");
        return PLAN_RESULT::FAIL;
    }

    const int valid_start_idx = 2 + drone_num;
    extractTourIndices(temp_indices, valid_start_idx, lkh_result);

    std::vector<int> final_indices;
    reconstructGlobalPath(lkh_result, considered_vertices_indices, segment_reserved_indices,
                          final_indices, refresh_global);
    insertNewNodes(final_indices, segment_added_indices, cur_relevant_graph_vertices_data);

    optimal_indices = final_indices;
    last_optimal_grid_indices_ = optimal_indices;
    last_relevant_graph_vertices_data_ = cur_relevant_graph_vertices_data;

    eigen_utils::Vec_Vec3d path;
    dynamic_expanding_grid_->getGridTour(cur_pos, optimal_indices, global_grid_tour, path);
    return PLAN_RESULT::SUCCEED;
}

PLAN_RESULT GlobalCoveragePlanner::planFullCoverageTour(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw, eigen_utils::Vec_Vec3d& global_grid_tour,
    std::vector<int>& optimal_indices) const
{
    optimal_indices.clear();
    Eigen::MatrixXd global_matrix;

    const int drone_num = 1;

    time_utils::Timer timer("Compute Grid CostMatrix");
    timer.start();
    buildFullCoverageCostMatrix(cur_pos, cur_vel, cur_yaw, global_matrix);
    const int dimension = global_matrix.rows();
    timer.stop(kTimerLogFlags.compute_grid_cost_matrix, "ms");

    if (dimension <= 1 + drone_num)
    {
        ROS_WARN("No Grid.");
        return PLAN_RESULT::NO_FRONTIER;
    }

    timer = time_utils::Timer("solve Grid TSPLKH");
    timer.start();

    const std::string file_name = tsp_dir_ + "/drone_grid_" + std::to_string(drone_id_);
    std::vector<int> temp_indices;
    if (!lkh_interface_.solveATSP(file_name, global_matrix, temp_indices))
    {
        ROS_ERROR("Failed to solve global ATSP with LKH.");
        return PLAN_RESULT::FAIL;
    }

    const int valid_start_idx = 2 + drone_num;
    extractTourIndices(temp_indices, valid_start_idx, optimal_indices);
    timer.stop(kTimerLogFlags.solve_grid_tsplkh, "ms");

    eigen_utils::Vec_Vec3d path;
    dynamic_expanding_grid_->getGridTour(cur_pos, optimal_indices, global_grid_tour, path);
    return PLAN_RESULT::SUCCEED;
}

void GlobalCoveragePlanner::buildFullCoverageCostMatrix(const eigen_utils::Vec3d& cur_pos,
                                                        const eigen_utils::Vec3d& cur_vel,
                                                        const eigen_utils::Vec3d& cur_yaw,
                                                        Eigen::MatrixXd& global_matrix) const
{
    dynamic_expanding_grid_->computeGlobalCoverageCostMatrix({cur_pos}, {cur_vel}, {cur_yaw[0]},
                                                             global_matrix);
}

void GlobalCoveragePlanner::buildIncrementalCoverageCostMatrix(
    const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
    const eigen_utils::Vec3d& cur_yaw,
    const std::vector<std::vector<int>>& planning_considered_indices,
    std::vector<int>& considered_vertices_indices, Eigen::MatrixXd& global_matrix) const
{
    dynamic_expanding_grid_->computeIncrementalGlobalCoverageCostMatrix(
        {cur_pos}, {cur_vel}, {cur_yaw[0]}, planning_considered_indices,
        considered_vertices_indices, global_matrix);
}

void GlobalCoveragePlanner::defineLocalRangeAndPartitionNodes(
    const eigen_utils::Vec3d& cur_pos, const double divide_range, const int segment_pt_num,
    std::vector<int>& independent_pt_ids, std::vector<std::vector<int>>& segment_pt_ids,
    eigen_utils::Vec3iSet& local_grid_indices_set, IncrementalVisualizationData* vis_data) const
{
    independent_pt_ids.clear();
    segment_pt_ids.clear();

    process_utils::CubeBox local_box(
        cur_pos - eigen_utils::Vec3d(divide_range, divide_range, divide_range),
        cur_pos + eigen_utils::Vec3d(divide_range, divide_range, divide_range));

    const eigen_utils::Vec_Vec3i local_grid_indices =
        dynamic_expanding_grid_->getGridIndicesInRange(local_box.min_, local_box.max_);
    local_grid_indices_set =
        eigen_utils::Vec3iSet(local_grid_indices.begin(), local_grid_indices.end());

    bool last_independent = false;
    for (const int idx : last_optimal_grid_indices_)
    {
        const eigen_utils::Vec3i& pt_idx = last_relevant_graph_vertices_data_[idx].index_;
        if (local_grid_indices_set.find(pt_idx) != local_grid_indices_set.end())
        {
            independent_pt_ids.push_back(idx);
            last_independent = true;
        }
        else
        {
            if (segment_pt_ids.empty() || last_independent)
            {
                segment_pt_ids.push_back({idx});
            }
            else if (segment_pt_ids.back().size() < static_cast<size_t>(segment_pt_num))
            {
                segment_pt_ids.back().push_back(idx);
            }
            else
            {
                segment_pt_ids.push_back({idx});
            }
            last_independent = false;
        }
    }

    if (vis_data == nullptr)
        return;

    for (const int idx : independent_pt_ids)
        vis_data->independent_pts.push_back(last_relevant_graph_vertices_data_[idx].centroid_);

    for (const std::vector<int>& segment : segment_pt_ids)
    {
        eigen_utils::Vec_Vec3d edge;
        for (const int idx : segment)
        {
            const auto& centroid = last_relevant_graph_vertices_data_[idx].centroid_;
            vis_data->segment_pts.push_back(centroid);
            edge.push_back(centroid);
        }
        vis_data->segment_edges.push_back(std::move(edge));
    }
}

void GlobalCoveragePlanner::processAddedAndDeletedNodes(
    const std::vector<int>& independent_pt_ids, const std::vector<std::vector<int>>& segment_pt_ids,
    std::vector<int>& independent_reserved_indices,
    std::vector<std::vector<int>>& segment_reserved_indices,
    std::vector<int>& independent_added_indices, std::vector<int>& segment_added_indices,
    const eigen_utils::Vec3iSet& local_grid_indices_set,
    std::vector<map_process::RelevantGridAttributes>& cur_relevant_graph_vertices_data) const
{
    independent_reserved_indices.clear();
    segment_reserved_indices.clear();
    independent_added_indices.clear();
    segment_added_indices.clear();

    cur_relevant_graph_vertices_data = dynamic_expanding_grid_->getRelevantGridVertexSnapshot();

    eigen_utils::Vec3dMap<int, 3> cur_vertices_set, last_vertices_set;
    for (size_t i = 0; i < cur_relevant_graph_vertices_data.size(); ++i)
        cur_vertices_set[cur_relevant_graph_vertices_data[i].centroid_] = i;
    for (size_t i = 0; i < last_relevant_graph_vertices_data_.size(); ++i)
        last_vertices_set[last_relevant_graph_vertices_data_[i].centroid_] = i;

    for (const int idx : independent_pt_ids)
    {
        const eigen_utils::Vec3d& pt = last_relevant_graph_vertices_data_[idx].centroid_;
        const auto iter = cur_vertices_set.find(pt);
        if (iter != cur_vertices_set.end())
            independent_reserved_indices.push_back(iter->second);
    }

    for (const std::vector<int>& segment : segment_pt_ids)
    {
        std::vector<int> current_segment;
        for (const int idx : segment)
        {
            const eigen_utils::Vec3d& pt = last_relevant_graph_vertices_data_[idx].centroid_;
            const auto iter = cur_vertices_set.find(pt);
            if (iter != cur_vertices_set.end())
            {
                current_segment.push_back(iter->second);
            }
            else if (!current_segment.empty())
            {
                segment_reserved_indices.push_back(current_segment);
                current_segment.clear();
            }
        }

        if (!current_segment.empty())
            segment_reserved_indices.push_back(current_segment);
    }

    for (size_t i = 0; i < cur_relevant_graph_vertices_data.size(); ++i)
    {
        const auto& attr = cur_relevant_graph_vertices_data[i];
        if (last_vertices_set.find(attr.centroid_) != last_vertices_set.end())
            continue;

        if (local_grid_indices_set.find(attr.index_) != local_grid_indices_set.end())
            independent_added_indices.push_back(i);
        else
            segment_added_indices.push_back(i);
    }
}

void GlobalCoveragePlanner::extractNodesForPlanning(
    const std::vector<int>& independent_reserved_indices,
    const std::vector<int>& independent_added_indices,
    const std::vector<std::vector<int>>& segment_reserved_indices,
    std::vector<std::vector<int>>& planning_considered_indices)
{
    planning_considered_indices.clear();

    for (const int idx : independent_reserved_indices)
        planning_considered_indices.push_back({idx});
    for (const int idx : independent_added_indices)
        planning_considered_indices.push_back({idx});
    for (const std::vector<int>& segment : segment_reserved_indices)
        planning_considered_indices.push_back(segment);
}

void GlobalCoveragePlanner::reconstructGlobalPath(
    const std::vector<int>& lkh_result, const std::vector<int>& considered_vertices_indices,
    const std::vector<std::vector<int>>& segment_reserved_indices, std::vector<int>& final_indices,
    bool& precise_global)
{
    final_indices.clear();

    std::unordered_map<int, std::unordered_map<int, std::vector<int>>> segment_map;
    for (const std::vector<int>& segment : segment_reserved_indices)
    {
        segment_map[segment.front()][segment.back()] = segment;

        std::vector<int> reversed_segment(segment.rbegin(), segment.rend());
        segment_map[segment.back()][segment.front()] = reversed_segment;
    }

    for (size_t i = 0; i < lkh_result.size();)
    {
        const int cur_idx = considered_vertices_indices[lkh_result[i]];
        const auto it1 = segment_map.find(cur_idx);
        if (it1 == segment_map.end())
        {
            final_indices.push_back(cur_idx);
            ++i;
            continue;
        }

        if (it1->first == it1->second.begin()->first)
        {
            final_indices.push_back(cur_idx);
            ++i;
            continue;
        }

        if (i == lkh_result.size() - 1)
        {
            precise_global = true;
            final_indices.push_back(cur_idx);
            ++i;
            ROS_ERROR_STREAM("segment start at the last point");
            continue;
        }

        const int next_idx = considered_vertices_indices[lkh_result[i + 1]];
        const auto& inner_map = segment_map[cur_idx];
        const auto it2 = inner_map.find(next_idx);
        if (it2 != inner_map.end())
        {
            const std::vector<int>& segment = it2->second;
            final_indices.insert(final_indices.end(), segment.begin(), segment.end());
            i += 2;
        }
        else
        {
            precise_global = true;
            final_indices.push_back(cur_idx);
            ++i;
            ROS_ERROR_STREAM("segment_map[" << cur_idx << "][" << next_idx << "] not found");
        }
    }
}

void GlobalCoveragePlanner::insertNewNodes(
    std::vector<int>& final_indices, const std::vector<int>& segment_added_indices,
    const std::vector<map_process::RelevantGridAttributes>& cur_relevant_graph_vertices_data) const
{
    const auto& exploration_graph = dynamic_expanding_grid_->getRelevantGraphReadonly();

    std::unordered_map<int, int> vertex_graph_id_to_idx, idx_to_vertex_graph_id;
    for (size_t i = 0; i < cur_relevant_graph_vertices_data.size(); ++i)
    {
        const int v_id =
            exploration_graph.getVertexId(cur_relevant_graph_vertices_data[i].centroid_.cast<float>());
        if (v_id != -1)
        {
            vertex_graph_id_to_idx[v_id] = i;
            idx_to_vertex_graph_id[i] = v_id;
        }
        else
        {
            ROS_ERROR("v_id == -1");
        }
    }

    std::vector<int> given_vertex_indices;
    for (const int idx : segment_added_indices)
    {
        const int v_id =
            exploration_graph.getVertexId(cur_relevant_graph_vertices_data[idx].centroid_.cast<float>());
        if (v_id != -1)
            given_vertex_indices.push_back(v_id);
        else
            ROS_ERROR("v_id == -1");
    }

    std::unordered_set<int> subgraph_vertices(given_vertex_indices.begin(),
                                              given_vertex_indices.end());
    std::unordered_map<int, int> components;
    std::unordered_set<int> visited;
    int component_id = 0;

    std::function<void(int)> dfs_subgraph = [&](const int v) {
        visited.insert(v);
        components[v] = component_id;

        for (const auto& edge : exploration_graph.getLinkedEdges(v))
        {
            const int nbr = edge.first;
            if (subgraph_vertices.find(nbr) != subgraph_vertices.end() &&
                visited.find(nbr) == visited.end())
                dfs_subgraph(nbr);
        }
    };

    for (const int v : given_vertex_indices)
    {
        if (visited.find(v) != visited.end())
            continue;

        dfs_subgraph(v);
        ++component_id;
    }

    std::unordered_map<int, std::vector<int>> groups;
    for (const int idx : given_vertex_indices)
        groups[components[idx]].push_back(idx);

    for (auto& group_pair : groups)
    {
        std::vector<int>& group_vertices = group_pair.second;
        std::unordered_map<int, int> depth_map;
        std::queue<int> queue;
        std::unordered_set<int> visited_group;

        for (const int v : group_vertices)
        {
            bool is_adjacent = false;
            for (const auto& [nbr, edge] : exploration_graph.getLinkedEdges(v))
            {
                if (subgraph_vertices.find(nbr) == subgraph_vertices.end())
                {
                    is_adjacent = true;
                    break;
                }
            }

            if (is_adjacent)
            {
                depth_map[v] = 1;
                queue.push(v);
                visited_group.insert(v);
            }
            else
            {
                depth_map[v] = std::numeric_limits<int>::max();
            }
        }

        while (!queue.empty())
        {
            const int cur_v = queue.front();
            queue.pop();

            const int cur_depth = depth_map[cur_v];
            for (const auto& [nbr, edge] : exploration_graph.getLinkedEdges(cur_v))
            {
                if (subgraph_vertices.find(nbr) != subgraph_vertices.end() &&
                    visited_group.find(nbr) == visited_group.end())
                {
                    depth_map[nbr] = cur_depth + 1;
                    queue.push(nbr);
                    visited_group.insert(nbr);
                }
            }
        }

        std::vector<std::pair<int, int>> vertex_depths;
        for (const int v : group_vertices)
        {
            if (depth_map[v] == std::numeric_limits<int>::max())
                ROS_ERROR("Vertex %d has infinite depth.", v);
            vertex_depths.emplace_back(v, depth_map[v]);
        }

        std::sort(vertex_depths.begin(), vertex_depths.end(),
                  [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
                      return lhs.second < rhs.second;
                  });

        group_vertices.clear();
        for (const auto& pair : vertex_depths)
            group_vertices.push_back(pair.first);
    }

    std::unordered_map<int, std::vector<int>> grouped_indices;
    for (const auto& group_pair : groups)
    {
        std::vector<int> indices;
        indices.reserve(group_pair.second.size());
        for (const int v : group_pair.second)
            indices.push_back(vertex_graph_id_to_idx.at(v));
        grouped_indices[group_pair.first] = std::move(indices);
    }

    std::unordered_set<int> idx_in_global(final_indices.begin(), final_indices.end());
    std::unordered_map<int, std::unordered_map<int, double>> global_path_segment_costs;
    std::list<int> final_indices_list(final_indices.begin(), final_indices.end());
    std::unordered_map<int, std::list<int>::iterator> indices_iter_map;
    for (auto it = final_indices_list.begin(); it != final_indices_list.end(); ++it)
        indices_iter_map[*it] = it;

    for (const auto& [comp_id, group_vertices] : grouped_indices)
    {
        (void)comp_id;
        for (const int idx : group_vertices)
        {
            std::vector<int> adjacent_ids;
            for (const auto& [nbr, edge] : exploration_graph.getLinkedEdges(idx_to_vertex_graph_id.at(idx)))
            {
                if (idx_in_global.find(vertex_graph_id_to_idx.at(nbr)) != idx_in_global.end())
                    adjacent_ids.push_back(vertex_graph_id_to_idx.at(nbr));
            }

            double best_difference = std::numeric_limits<double>::max();
            double best_cost_prev = std::numeric_limits<double>::max();
            double best_cost_next = std::numeric_limits<double>::max();
            std::list<int>::iterator best_insert_it_prev = final_indices_list.end();
            std::list<int>::iterator best_insert_it_next = final_indices_list.end();

            for (const int adjacent_id : adjacent_ids)
            {
                const auto map_it = indices_iter_map.find(adjacent_id);
                if (map_it == indices_iter_map.end())
                {
                    ROS_ERROR("adjacent_id not found.");
                    continue;
                }

                const std::list<int>::iterator it = map_it->second;
                const std::list<int>::iterator it_prev =
                    it == final_indices_list.begin() ? final_indices_list.end() : std::prev(it);
                const std::list<int>::iterator it_next = std::next(it);
                double segment_old_cost = 0.0;

                if (it_prev != final_indices_list.end())
                {
                    if (global_path_segment_costs.find(*it_prev) == global_path_segment_costs.end())
                    {
                        std::tie(segment_old_cost, std::ignore) =
                            dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                                cur_relevant_graph_vertices_data[*it_prev].centroid_,
                                cur_relevant_graph_vertices_data[*it].centroid_,
                                map_process::constants::kPathCostFallback);
                        global_path_segment_costs[*it_prev][*it] = segment_old_cost;
                    }
                    else
                    {
                        segment_old_cost = global_path_segment_costs.at(*it_prev).at(*it);
                    }

                    const auto [cost1, path1] =
                        dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                            cur_relevant_graph_vertices_data[*it_prev].centroid_,
                            cur_relevant_graph_vertices_data[idx].centroid_,
                            map_process::constants::kPathCostFallback);
                    const auto [cost2, path2] =
                        dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                            cur_relevant_graph_vertices_data[idx].centroid_,
                            cur_relevant_graph_vertices_data[*it].centroid_,
                            map_process::constants::kPathCostFallback);
                    (void)path1;
                    (void)path2;

                    const double difference = cost1 + cost2 - segment_old_cost;
                    if (difference < best_difference)
                    {
                        best_difference = difference;
                        best_cost_prev = cost1;
                        best_cost_next = cost2;
                        best_insert_it_prev = it_prev;
                        best_insert_it_next = it;
                    }
                }

                if (it_next != final_indices_list.end())
                {
                    if (global_path_segment_costs.find(*it) == global_path_segment_costs.end())
                    {
                        std::tie(segment_old_cost, std::ignore) =
                            dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                                cur_relevant_graph_vertices_data[*it].centroid_,
                                cur_relevant_graph_vertices_data[*it_next].centroid_,
                                map_process::constants::kPathCostFallback);
                        global_path_segment_costs[*it][*it_next] = segment_old_cost;
                    }
                    else
                    {
                        segment_old_cost = global_path_segment_costs.at(*it).at(*it_next);
                    }

                    const auto [cost1, path1] =
                        dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                            cur_relevant_graph_vertices_data[*it].centroid_,
                            cur_relevant_graph_vertices_data[idx].centroid_,
                            map_process::constants::kPathCostFallback);
                    const auto [cost2, path2] =
                        dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                            cur_relevant_graph_vertices_data[idx].centroid_,
                            cur_relevant_graph_vertices_data[*it_next].centroid_,
                            map_process::constants::kPathCostFallback);
                    (void)path1;
                    (void)path2;

                    const double difference = cost1 + cost2 - segment_old_cost;
                    if (difference < best_difference)
                    {
                        best_difference = difference;
                        best_cost_prev = cost1;
                        best_cost_next = cost2;
                        best_insert_it_prev = it;
                        best_insert_it_next = it_next;
                    }
                }
                else
                {
                    const auto [cost1, path1] =
                        dynamic_expanding_grid_->computeCostAndPathFromGridToGrid(
                            cur_relevant_graph_vertices_data[*it].centroid_,
                            cur_relevant_graph_vertices_data[idx].centroid_,
                            map_process::constants::kPathCostFallback);
                    (void)path1;
                    if (cost1 < best_difference)
                    {
                        best_difference = cost1;
                        best_cost_prev = cost1;
                        best_insert_it_prev = it;
                        best_insert_it_next = it_next;
                    }
                }
            }

            if (best_insert_it_prev != final_indices_list.end())
            {
                std::list<int>::iterator new_it;
                if (best_insert_it_next != final_indices_list.end())
                {
                    new_it = final_indices_list.insert(best_insert_it_next, idx);
                    global_path_segment_costs[*best_insert_it_prev][idx] = best_cost_prev;
                    global_path_segment_costs[idx][*best_insert_it_next] = best_cost_next;
                }
                else
                {
                    new_it = final_indices_list.insert(final_indices_list.end(), idx);
                    global_path_segment_costs[*best_insert_it_prev][idx] = best_cost_prev;
                }

                indices_iter_map[idx] = new_it;
                idx_in_global.insert(idx);
            }
            else
            {
                ROS_ERROR("best_insert_it_prev == final_indices_list.end()");
            }
        }
    }

    final_indices.assign(final_indices_list.begin(), final_indices_list.end());
}

void GlobalCoveragePlanner::extractTourIndices(const std::vector<int>& raw_optimal_indices,
                                               const int valid_start_idx,
                                               std::vector<int>& filtered_indices)
{
    filtered_indices.clear();

    for (const int idx : raw_optimal_indices)
    {
        if (idx < valid_start_idx)
            continue;

        filtered_indices.push_back(idx - valid_start_idx);
    }
}
} // namespace fastex_explorer
