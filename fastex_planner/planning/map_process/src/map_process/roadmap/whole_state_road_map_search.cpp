/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-09-19 12:09:37
 * @LastEditTime: 2026-05-31 11:44:40
 * @Description:
 */

#include "map_process/roadmap/whole_state_road_map.h"

#include "map_process/core/map_process_constants.h"
#include "utils/hash_grid.hpp"
#include <vis_utils/marker_utils.h>

#include <random>

namespace map_process
{

/**
 * @brief Find the shortest path between the start point and the end point
 *
 * @param start_point the start point of the path
 * @param end_point the end point of the path
 * @param optimistic whether to use optimistic collision checking
 * @param shortest_path the shortest path between the start point and the end point
 * @param cost the cost of the shortest path
 * @return true if the shortest path is found
 * @return false if the shortest path is not found
 */
bool WSRoadMap::findShortestPath(const eigen_utils::Vec3f& start_point,
                                 const eigen_utils::Vec3f& end_point, const bool optimistic,
                                 eigen_utils::Vec_Vec3f& shortest_path, double& cost) const
{
    return findShortestPathImpl(start_point, end_point, optimistic, shortest_path, cost);
}

/**
 * @brief Find the shortest path between two points within a bounding box.
 *
 * @param start_point Start point of the path.
 * @param end_point End point of the path.
 * @param confined_box Bounding box to constrain the search.
 * @param optimistic Whether to use optimistic collision checking.
 * @param shortest_path The output shortest path.
 * @param cost The cost of the path.
 * @return true If the shortest path is found.
 */
bool WSRoadMap::findShortestPathWithinBox(const eigen_utils::Vec3f& start_point,
                                          const eigen_utils::Vec3f& end_point,
                                          const process_utils::CubeBox& confined_box,
                                          const bool optimistic,
                                          eigen_utils::Vec_Vec3f& shortest_path, double& cost) const
{
    return findShortestPathImpl(start_point, end_point, optimistic, shortest_path, cost,
                                confined_box);
}

bool WSRoadMap::findShortestPathImpl(
    const eigen_utils::Vec3f& start_point, const eigen_utils::Vec3f& end_point,
    const bool optimistic, eigen_utils::Vec_Vec3f& shortest_path, double& cost,
    const std::optional<process_utils::CubeBox>& bounding_box) const
{
    double cost_on_roadmap = 0.0, cost_from_start_to_map = 0.0, cost_from_map_to_end = 0.0;
    eigen_utils::Vec_Vec3f path_on_roadmap, path_from_start_to_map, path_from_map_to_end;

    int s_nearest_id, e_nearest_id;
    eigen_utils::Vec3f s_key_point, e_key_point;

    s_nearest_id = graph_.getVertexId(start_point);
    e_nearest_id = graph_.getVertexId(end_point);

    bool is_s_key_point_found = s_nearest_id >= 0;
    bool is_e_key_point_found = e_nearest_id >= 0;

    if (!is_s_key_point_found)
    {
        is_s_key_point_found = findNearestValidPointInGraph(
            start_point, optimistic, s_key_point, s_nearest_id, path_from_start_to_map,
            cost_from_start_to_map, 8.0, bounding_box);
    }

    if (!is_e_key_point_found)
    {
        is_e_key_point_found = findNearestValidPointInGraph(
            end_point, optimistic, e_key_point, e_nearest_id, path_from_map_to_end,
            cost_from_map_to_end, 8.0, bounding_box);
    }

    if (!is_s_key_point_found)
    {
        ROS_ERROR_STREAM(
            __func__ << ": Failed to find a shortest path from the start point to any goal.");
        return false;
    }

    if (!is_e_key_point_found)
    {
        ROS_ERROR_STREAM(
            __func__ << ": Failed to find a shortest path from the end point to any goal.");
        return false;
    }

    std::reverse(path_from_map_to_end.begin(), path_from_map_to_end.end());

    // 3.Try to find a shortest path between the start and end key points on the roadmap
    std::vector<int> waypoint_ids;
    bool is_path_found = false;

    if (bounding_box.has_value())
    {
        is_path_found = graph_.findShortestPathWithinBox(
            s_nearest_id, e_nearest_id, waypoint_ids, path_on_roadmap, cost_on_roadmap,
            bounding_box->min_.cast<float>(), bounding_box->max_.cast<float>());
    }
    else
    {
        is_path_found = graph_.findShortestPath(s_nearest_id, e_nearest_id, waypoint_ids,
                                                path_on_roadmap, cost_on_roadmap);
    }

    shortest_path.clear();
    if (is_path_found)
    {
        shortest_path.reserve(path_from_start_to_map.size() + path_on_roadmap.size() +
                              path_from_map_to_end.size());
        shortest_path.insert(shortest_path.end(), path_from_start_to_map.begin(),
                             path_from_start_to_map.end());
        shortest_path.insert(shortest_path.end(), path_on_roadmap.begin(), path_on_roadmap.end());
        shortest_path.insert(shortest_path.end(), path_from_map_to_end.begin(),
                             path_from_map_to_end.end());

        cost = cost_from_start_to_map + cost_on_roadmap + cost_from_map_to_end;
    }

    return is_path_found;
}

/**
 * @brief Search the nearest vertex of the point
 *
 * @param point the point that needs to search the nearest vertex
 * @param nearest_point the nearest point of the point
 * @param nearest_point_id the id of the nearest point
 * @return true If the nearest point is found
 * @return false If the nearest point is not found
 */
bool WSRoadMap::nearestSearch(const eigen_utils::Vec3f& point, eigen_utils::Vec3f& nearest_point,
                              int& nearest_point_id)
{
    std::vector<int> ids = graph_.getNearestVertex(point, 1);

    if (ids.empty())
    {
        return false;
    }
    else
    {
        nearest_point_id = ids[0];
        nearest_point = graph_.getVertexPos(nearest_point_id);
        return true;
    }
}

/**
 * @brief Searches for the nearest vertices to a given point in the graph
 *
 * This function searches for the nearest vertices to a given point in the graph
 * and returns their positions and IDs. It uses a k-nearest neighbors search
 * algorithm with a specified maximum search distance.
 *
 * @param point The query point
 * @param k The number of nearest neighbors to find
 * @param max_distance The maximum search distance
 * @param neighbor_vertexs The list of positions of the nearest vertices found
 * @param neighbor_vertex_ids The list of IDs of the nearest vertices found
 * @return true If nearest vertices are found
 * @return false If no nearest vertices are found
 */
bool WSRoadMap::nearestSearch(const eigen_utils::Vec3f& point, const int k,
                              eigen_utils::Vec_Vec3f& neighbor_vertexs,
                              std::vector<int>& neighbor_vertex_ids, const double max_distance)
{
    std::vector<int> ids = graph_.getNearestVertex(point, k, max_distance);

    if (ids.empty())
    {
        return false;
    }
    else
    {
        neighbor_vertex_ids.swap(ids);

        neighbor_vertexs.clear();
        neighbor_vertexs.reserve(neighbor_vertex_ids.size());
        for (const int id : neighbor_vertex_ids)
            neighbor_vertexs.push_back(graph_.getVertexPos(id));

        return true;
    }
}

/**
 * @brief Search the neighbor vertexs of the point in the range
 *
 * @param point the point that needs to search the neighbor vertexs
 * @param range the range of the search
 * @param neighbor_vertexs the neighbor vertexs of the point
 * @param neighbor_vertex_ids the ids of the neighbor vertexs
 * @return true If the neighbor vertexs are found
 * @return false  If the neighbor vertexs are not found
 */
bool WSRoadMap::nearRangeSearch(const eigen_utils::Vec3f& point, const double range,
                                eigen_utils::Vec_Vec3f& neighbor_vertexs,
                                std::vector<int>& neighbor_vertex_ids)
{
    neighbor_vertex_ids = graph_.getRadiusNeighborVertexsIDs(point, range, neighbor_vertexs);
    return !neighbor_vertex_ids.empty();
}

/**
 * @brief Perform a box neighbor search in the WS roadmap.
 *
 * This function searches for vertices within a specified bounding box in the WS roadmap. It
 * retrieves the vertices and their IDs that are within the bounding box.
 *
 * @param box_min The minimum corner of the bounding box.
 * @param box_max The maximum corner of the bounding box.
 * @param neighbor_vertexs Output vector to store the positions of the neighboring vertices.
 * @param neighbor_vertex_ids Output vector to store the IDs of the neighboring vertices.
 * @return true if any vertices are found within the bounding box, false otherwise.
 */
bool WSRoadMap::BoxNeighborSearch(const eigen_utils::Vec3f& box_min,
                                  const eigen_utils::Vec3f& box_max,
                                  eigen_utils::Vec_Vec3f& neighbor_vertexs,
                                  std::vector<int>& neighbor_vertex_ids)
{
    neighbor_vertex_ids = graph_.getBoxNeighborVertexsIDs(box_min, box_max, neighbor_vertexs);
    return !neighbor_vertex_ids.empty();
}

/**
 * @brief Search the KD-tree for nearby vertices and filter by occupancy and bounding box.
 *
 * @param point Query point.
 * @param optimistic Whether to treat UNKNOWN cells as free.
 * @param max_candidates Max number of candidates (>0 limits, <=0 returns all filtered).
 * @param neighbors_search_range Max search radius [m].
 * @param bounding_box Optional bounding box to restrict candidates.
 * @return Filtered nearest vertex positions.
 */
eigen_utils::Vec_Vec3f WSRoadMap::searchNearestFilteredVertices(
    const eigen_utils::Vec3f& point, const bool optimistic, const int max_candidates,
    const double neighbors_search_range,
    const std::optional<process_utils::CubeBox>& bounding_box) const
{
    PointVector nearest_points;
    std::vector<float> nearest_dists;
    active_points_ikdtree_->Nearest_Search(PointType(point.x(), point.y(), point.z()), 100,
                                           nearest_points, nearest_dists, neighbors_search_range);

    const bool box_check = bounding_box.has_value();
    PointVector valid_nearest_points;
    eigen_utils::Vec3d pos;

    for (const auto& pt : nearest_points)
    {
        pos = eigen_utils::Vec3d(pt.x, pt.y, pt.z);

        if ((!optimistic && sdf_map_->getOccupancy(pos) == sdf_map_->UNKNOWN) ||
            (box_check &&
             !process_utils::ProcessUtils::isInBox(pos, bounding_box->min_, bounding_box->max_)))
            continue;

        valid_nearest_points.push_back(pt);

        if (max_candidates > 0 &&
            valid_nearest_points.size() >= static_cast<size_t>(max_candidates))
            break;
    }

    eigen_utils::Vec_Vec3f nearest_points_eigen;
    for (const auto& pt : valid_nearest_points)
        nearest_points_eigen.emplace_back(pt.x, pt.y, pt.z);

    return nearest_points_eigen;
}

/**
 * @brief Find the nearest valid (collision-free, connected) point in the graph.
 *
 * Searches for nearby vertices, filters by occupancy and bounding box,
 * then attempts straight-line and detailed path connections.
 *
 * @param point Query point.
 * @param optimistic Whether to use optimistic collision checking.
 * @param nearest_point Output nearest point found.
 * @param nearest_point_id Output vertex ID of the nearest point.
 * @param path Output path to the nearest point.
 * @param path_cost Output cost of the path.
 * @param neighbors_search_range Max search radius [m].
 * @param bounding_box Optional bounding box.
 * @return true If a valid nearest point is found.
 */
bool WSRoadMap::findNearestValidPointInGraph(
    const eigen_utils::Vec3f& point, const bool optimistic, eigen_utils::Vec3f& nearest_point,
    int& nearest_point_id, eigen_utils::Vec_Vec3f& path, double& path_cost,
    const double neighbors_search_range,
    const std::optional<process_utils::CubeBox>& bounding_box) const
{
    if (!is_graph_initialized_)
    {
        nearest_point_id = -1;
        path.clear();
        path_cost = constants::kPathCostFallback;
        ROS_ERROR("The graph is not initialized!");
        return false;
    }

    // 1. Search & filter nearest candidate vertices
    const eigen_utils::Vec_Vec3f nearest_points_eigen =
        searchNearestFilteredVertices(point, optimistic, 10, neighbors_search_range, bounding_box);

    if (nearest_points_eigen.empty())
    {
        nearest_point_id = -1;
        path.clear();
        path_cost = constants::kPathCostFallback;
        ROS_ERROR_STREAM(__func__ << ": No nearest points found for the point: "
                                  << point.transpose());
        return false;
    }

    // 2. Try to find a shortest path between point and nearest candidates
    bool is_nearest_point_found = findCollisionFreeStraightPath(
        point, nearest_points_eigen, optimistic, nearest_point, path, path_cost);

    if (!is_nearest_point_found)
    {
        is_nearest_point_found = findDetailedPathToNearestGoal(
            point, nearest_points_eigen, optimistic, constants::kNeighborSearchRange, nearest_point,
            path, path_cost);
    }

    if (!is_nearest_point_found)
    {
        nearest_point_id = -1;
        path.clear();
        path_cost = constants::kPathCostFallback;
        ROS_ERROR_STREAM(
            __func__ << ": Failed to find a shortest path from the point to any goal.");
        return false;
    }

    nearest_point_id = graph_.getVertexId(nearest_point);
    return true;
}

/**
 * @brief Finds the nearest valid points in the graph from a given point.
 *
 * This function searches for the nearest valid points in the graph from the given point.
 * It uses either an optimistic or strict collision check based on the `optimistic` parameter.
 * The function returns true if at least one valid point is found, and false otherwise.
 *
 * @param point The starting point for the search.
 * @param optimistic A boolean flag indicating whether to use optimistic collision checking.
 * @param near_points A reference to a vector where the nearest valid points will be stored.
 * @param near_point_ids A reference to a vector where the IDs of the nearest valid points will be
 * stored.
 * @param paths A reference to a vector where the paths to the nearest valid points will be stored.
 * @param path_costs A reference to a vector where the costs of the paths will be stored.
 * @param neighbors_search_range The search range for finding the nearest points.
 * @param bounding_box An optional bounding box to limit the search area.
 * @return true If at least one valid point is found.
 * @return false If no valid points are found.
 */
bool WSRoadMap::findNearestValidPointsInGraph(
    const eigen_utils::Vec3f& point, const bool optimistic, eigen_utils::Vec_Vec3f& near_points,
    std::vector<int>& near_point_ids, std::vector<eigen_utils::Vec_Vec3f>& paths,
    std::vector<double>& path_costs, const double neighbors_search_range,
    const std::optional<process_utils::CubeBox>& bounding_box) const
{
    near_points.clear();
    near_point_ids.clear();
    paths.clear();
    path_costs.clear();

    if (!is_graph_initialized_)
    {
        ROS_ERROR("The graph is not initialized!");
        return false;
    }

    // 1. Search & filter nearest candidate vertices
    const eigen_utils::Vec_Vec3f nearest_points_eigen =
        searchNearestFilteredVertices(point, optimistic, -1, neighbors_search_range, bounding_box);

    if (nearest_points_eigen.empty())
    {
        ROS_ERROR_STREAM(__func__ << ": No nearest points found for the point: "
                                  << point.transpose());
        return false;
    }

    // 2. Try to find shortest paths between point and nearest candidates
    bool is_nearest_point_found =
        findCollisionFreeStraightPaths(point, nearest_points_eigen, optimistic, near_points, paths,
                                       path_costs, wsgp_.connectable_num_);

    if (!is_nearest_point_found)
    {
        eigen_utils::Vec3f nearest_point;
        eigen_utils::Vec_Vec3f path;
        double path_cost;
        is_nearest_point_found = findDetailedPathToNearestGoal(
            point, nearest_points_eigen, optimistic, constants::kNeighborSearchRange, nearest_point,
            path, path_cost);

        if (is_nearest_point_found)
        {
            near_points.push_back(nearest_point);
            paths.push_back(path);
            path_costs.push_back(path_cost);
        }
    }

    if (!is_nearest_point_found)
    {
        ROS_ERROR_STREAM(
            __func__ << ": Failed to find a shortest path from the point to any goal.");
        near_points.clear();
        near_point_ids.clear();
        paths.clear();
        path_costs.clear();
        return false;
    }

    for (const auto& near_point : near_points)
        near_point_ids.push_back(graph_.getVertexId(near_point));

    return true;
}

/**
 * @brief Finds a collision-free path to the nearest point.
 *
 * This function attempts to find a collision-free path from a given point to the nearest point in a
 * list of points. It uses either optimistic or strict collision checking based on the `optimistic`
 * flag.
 *
 * @param point The starting point of the path, of type eigen_utils::Vec3f.
 * @param nearest_points A vector of points representing the nearest goals, of type PointVector.
 * @param optimistic A boolean flag indicating whether to use optimistic collision checking.
 * @param key_point The key point in the path, of type eigen_utils::Vec3f.
 * @param path The resulting path, of type eigen_utils::Vec_Vec3f.
 * @param path_cost The cost of the resulting path.
 * @return true if a collision-free path is found, false otherwise.
 */
bool WSRoadMap::findCollisionFreeStraightPath(const eigen_utils::Vec3f& point,
                                              const eigen_utils::Vec_Vec3f& nearest_points,
                                              const bool optimistic, eigen_utils::Vec3f& key_point,
                                              eigen_utils::Vec_Vec3f& path, double& path_cost) const
{
    eigen_utils::Vec_Vec3f key_points;
    std::vector<eigen_utils::Vec_Vec3f> paths;
    std::vector<double> path_costs;

    if (findCollisionFreeStraightPaths(point, nearest_points, optimistic, key_points, paths,
                                       path_costs, 1))
    {
        key_point = key_points[0];
        path = std::move(paths[0]);
        path_cost = path_costs[0];
        return true;
    }
    return false;
}

/**
 * @brief Finds collision-free straight paths from a given point to a set of nearest points.
 *
 * This function checks for collision-free paths from the given point to each of the nearest points.
 * It uses either an optimistic or strict collision check based on the `optimistic` parameter.
 * The function returns true if at least one collision-free path is found, and false otherwise.
 *
 * @param point The starting point for the path search.
 * @param nearest_points A vector of points to which paths are to be checked.
 * @param optimistic A boolean flag indicating whether to use optimistic collision checking.
 * @param key_points A reference to a vector where the key points of collision-free paths will be
 * stored.
 * @param paths A reference to a vector where the collision-free paths will be stored.
 * @param path_costs A reference to a vector where the costs of the collision-free paths will be
 * stored.
 * @param max_n The maximum number of collision-free paths to find.
 * @return true If at least one collision-free path is found.
 * @return false If no collision-free paths are found.
 */
bool WSRoadMap::findCollisionFreeStraightPaths(const eigen_utils::Vec3f& point,
                                               const eigen_utils::Vec_Vec3f& nearest_points,
                                               const bool optimistic,
                                               eigen_utils::Vec_Vec3f& key_points,
                                               std::vector<eigen_utils::Vec_Vec3f>& paths,
                                               std::vector<double>& path_costs,
                                               const int max_n) const
{
    key_points.clear();
    paths.clear();
    path_costs.clear();

    fast_planner::SDFMap::CollisionCheckResult result;

    int n = 0;
    for (const eigen_utils::Vec3f& nearest_point : nearest_points)
    {
        if (optimistic)
        {
            result = sdf_map_->isOptimisticInflatedCollisionFreeStraight(
                eigen_utils::Vec3d(point.x(), point.y(), point.z()),
                eigen_utils::Vec3d(nearest_point.x(), nearest_point.y(), nearest_point.z()));
        }
        else
        {
            result = sdf_map_->isStrictInflatedCollisionFreeStraight(
                eigen_utils::Vec3d(point.x(), point.y(), point.z()),
                eigen_utils::Vec3d(nearest_point.x(), nearest_point.y(), nearest_point.z()));
        }

        if (result.is_collision_free)
        {
            key_points.push_back(nearest_point);
            paths.push_back({point, nearest_point});
            path_costs.push_back((point - nearest_point).norm());
            n++;

            if (n >= max_n)
                break;
        }
    }

    return n > 0;
}

/**
 * @brief Finds the detailed path to the nearest goal.
 *
 * This function finds the shortest path from a given point to the nearest goal within a specified
 * search range.
 *
 * @param point The starting point of the path, of type eigen_utils::Vec3f.
 * @param nearest_points A vector of points representing the nearest goals, of type PointVector.
 * @param optimistic A boolean flag indicating whether to use optimistic search.
 * @param search_range The search range for finding the nearest goal.
 * @param key_point The key point in the path, of type eigen_utils::Vec3f.
 * @param path The resulting path, of type eigen_utils::Vec_Vec3f.
 * @param path_cost The cost of the resulting path.
 * @return true if a path is found, false otherwise.
 */
bool WSRoadMap::findDetailedPathToNearestGoal(const eigen_utils::Vec3f& point,
                                              const eigen_utils::Vec_Vec3f& nearest_points,
                                              const bool optimistic, const double search_range,
                                              eigen_utils::Vec3f& key_point,
                                              eigen_utils::Vec_Vec3f& path, double& path_cost) const
{
    eigen_utils::Vec3fSet<3> goals;
    for (const auto& pt : nearest_points)
        goals.insert(pt);

    process_utils::CubeBox voxel_search_box(
        point.cast<double>() - eigen_utils::Vec3d(search_range, search_range, search_range),
        point.cast<double>() + eigen_utils::Vec3d(search_range, search_range, search_range));

    std::tie(path_cost, path) =
        findShortestPathFromStartToAnyGoal(point, goals, optimistic, voxel_search_box);

    if (path_cost < 0)
    {
        ROS_ERROR_STREAM(
            __func__ << ", Failed to find a shortest path from the point to any goal.");
        path.clear();
        return false;
    }
    else
    {
        key_point = path.back();
        return true;
    }
}

/**
 * @brief Uses Dijkstra's algorithm to search for the shortest path from the start point to any of
 * the goal points in 3D space.
 *
 * This function initializes a goal map, a priority queue, and cost and parent maps. It then
 * iteratively explores the neighbors of the current node, updating the cost and parent maps as it
 * finds shorter paths. If a goal point is reached, it backtracks to construct the path and returns
 * the path length and sequence of path points. If no goal point is found, it returns an empty path
 * with a cost of -1.0.
 *
 * @param start The starting point in 3D coordinates.
 * @param goals A set of goal points in 3D coordinates.
 * @param optimistic A flag indicating whether to use optimistic collision checking.
 * @param search_box The search box for the Dijkstra algorithm.
 * @return std::pair<double, eigen_utils::Vec_Vec3f> Returns a pair containing the path length and
 * the sequence of path points.
 */
std::pair<double, eigen_utils::Vec_Vec3f> WSRoadMap::findShortestPathFromStartToAnyGoal(
    const eigen_utils::Vec3f start, const eigen_utils::Vec3fSet<3>& goals, const bool optimistic,
    const process_utils::CubeBox& search_box) const
{
    eigen_utils::Vec3i start_idx;
    sdf_map_->posToIndex(start.cast<double>(), start_idx);

    // Check if the start point is in an obstacle
    if (sdf_map_->getInflateOccupancy(start_idx) != 0 ||
        (!optimistic && sdf_map_->getOccupancy(start_idx) != sdf_map_->FREE))
    {
        ROS_ERROR_STREAM("The start point is not safe,  optimistic: "
                         << optimistic
                         << ", inflate occupancy: " << sdf_map_->getInflateOccupancy(start_idx)
                         << ", occupancy: " << sdf_map_->getOccupancy(start_idx)
                         << ", start point: " << start.transpose());
        return std::make_pair(-1.0, eigen_utils::Vec_Vec3f());
    }

    // Initialize the goal map
    bool has_goal = false;
    eigen_utils::Vec3iMap<eigen_utils::Vec3f> goals_map;
    for (const auto& goal : goals)
    {
        eigen_utils::Vec3i idx;
        sdf_map_->posToIndex(goal.cast<double>(), idx);

        if (sdf_map_->getInflateOccupancy(idx) != 0 ||
            (!optimistic && sdf_map_->getOccupancy(idx) != sdf_map_->FREE))
            continue;

        goals_map[idx] = goal;
        has_goal = true;
    }

    if (!has_goal)
    {
        ROS_ERROR("No goal point is in the safe space!");
        return std::make_pair(-1.0, eigen_utils::Vec_Vec3f());
    }

    // Initialize the priority queue
    using CostIndexPair = std::pair<double, eigen_utils::Vec3i>;
    auto compare = [](const CostIndexPair& a, const CostIndexPair& b) { return a.first > b.first; };

    std::priority_queue<CostIndexPair, std::vector<CostIndexPair>, decltype(compare)> pq(compare);
    eigen_utils::Vec3iMap<double> cost_map;
    eigen_utils::Vec3iMap<eigen_utils::Vec3i> parent_map;
    eigen_utils::Vec3iSet closed_set;

    // Initialize the start point
    pq.push(std::make_pair(0.0, start_idx));
    cost_map[start_idx] = 0.0;
    parent_map[start_idx] = start_idx;

    eigen_utils::Vec3i search_box_min, search_box_max;
    sdf_map_->posToIndex(search_box.min_, search_box_min);
    sdf_map_->posToIndex(search_box.max_, search_box_max);

    eigen_utils::Vec_Vec3i nbrs;
    while (!pq.empty())
    {
        auto [cur_cost, cur_idx] = pq.top();
        pq.pop();

        // If the current node is in the closed set, skip it
        if (closed_set.find(cur_idx) != closed_set.end())
            continue;

        // Add the current node to the closed set
        closed_set.insert(cur_idx);

        // If the current node is any of the goal points, backtrack the path and return
        if (goals_map.find(cur_idx) != goals_map.end())
        {
            eigen_utils::Vec_Vec3f path;
            path.push_back(goals_map[cur_idx]);

            eigen_utils::Vec3i idx = parent_map[cur_idx];
            eigen_utils::Vec3d pos;

            while (idx != start_idx)
            {
                sdf_map_->indexToPos(idx, pos);
                path.push_back(pos.cast<float>());
                idx = parent_map[idx];
            }

            path.push_back(start); // Add the start point to the path
            std::reverse(path.begin(), path.end());

            return std::make_pair(cur_cost, path);
        }

        // Iterate through the neighbors of the current node
        nbrs = process_utils::ProcessUtils::allNeighbors(cur_idx);
        for (const auto& nbr_idx : nbrs)
        {
            // Skip if the neighbor is out of bounds or occupied
            if (sdf_map_->getInflateOccupancy(nbr_idx) != 0 ||
                (!optimistic && sdf_map_->getOccupancy(nbr_idx) != sdf_map_->FREE) ||
                !process_utils::ProcessUtils::isInBox(nbr_idx, search_box_min, search_box_max) ||
                !sdf_map_->isInBox(nbr_idx))
                continue;

            // Calculate the cost from the current node to the neighbor
            double new_cost = cur_cost + (nbr_idx - cur_idx).cast<double>().norm();

            // If the neighbor has not been visited or a shorter path is found,
            // update its cost and parent, and push it to the priority queue
            if (cost_map.find(nbr_idx) == cost_map.end() || new_cost < cost_map[nbr_idx])
            {
                cost_map[nbr_idx] = new_cost;
                parent_map[nbr_idx] = cur_idx;
                pq.push(std::make_pair(new_cost, nbr_idx));
            }
        }
    }

    // If no goal point is found, return an empty path with cost -1.0
    return std::make_pair(-1.0, eigen_utils::Vec_Vec3f());
}

/**
 * @brief Generate the markers of the graph
 *
 * @param graph_markers the markers of the graph
 * @param vertices_scale the scale of the vertices
 * @param edges_scale the scale of the edges
 * @param vertices_rgba the color of the vertices
 * @param edges_rgba the color of the edges
 */
void WSRoadMap::generateRoadGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                         const double& vertices_scale, const double& edges_scale,
                                         const eigen_utils::Vec4d& vertices_rgba,
                                         const eigen_utils::Vec4d& edges_rgba) const
{
    const ros::Time stamp = ros::Time::now();
    const auto vertices_marker_scale =
        vis_utils::marker_utils::makeScale(vertices_scale, vertices_scale, vertices_scale);
    visualization_msgs::Marker vertices = vis_utils::marker_utils::makePointMarker(
        wsgp_.frame_id_, "WSRoadMap", 0, vertices_marker_scale,
        vis_utils::marker_utils::toRosColor(vertices_rgba), visualization_msgs::Marker::ADD, stamp);
    visualization_msgs::Marker auto_edges = vis_utils::marker_utils::makeLineListMarker(
        wsgp_.frame_id_, "WSRoadMap", 1, edges_scale,
        vis_utils::marker_utils::toRosColor(edges_rgba), visualization_msgs::Marker::ADD, stamp);
    visualization_msgs::Marker inactive_vertices = vis_utils::marker_utils::makePointMarker(
        wsgp_.frame_id_, "WSRoadMap", 2, vertices_marker_scale,
        vis_utils::marker_utils::toRosColor(eigen_utils::Vec4d(0.5, 0.5, 0.5, 0.3)),
        visualization_msgs::Marker::ADD, stamp);
    visualization_msgs::Marker inactive_edges = vis_utils::marker_utils::makeLineListMarker(
        wsgp_.frame_id_, "WSRoadMap", 3, edges_scale,
        vis_utils::marker_utils::toRosColor(eigen_utils::Vec4d(0.5, 0.5, 0.5, 0.3)),
        visualization_msgs::Marker::ADD, stamp);
    visualization_msgs::Marker manual_edges = vis_utils::marker_utils::makeLineListMarker(
        wsgp_.frame_id_, "WSRoadMap", 4, edges_scale * 2,
        vis_utils::marker_utils::toRosColor(eigen_utils::Vec4d(0.0, 1.0, 0.0, 0.5)),
        visualization_msgs::Marker::ADD, stamp);

    auto snapshot = graph_.getGraphVisualizationSnapshot();

    // Local map for fast vertex lookup by ID without copying full vertex records again.
    struct VertexVizInfo
    {
        eigen_utils::Vec3f pos;
        bool active;
        bool manually_updated;
    };

    std::unordered_map<int, VertexVizInfo> id_vertex_map;
    id_vertex_map.reserve(snapshot.vertices.size());

    vertices.points.reserve(snapshot.vertices.size());
    inactive_vertices.points.reserve(snapshot.vertices.size());
    auto_edges.points.reserve(snapshot.edges.size() * 2);
    inactive_edges.points.reserve(snapshot.edges.size() * 2);
    manual_edges.points.reserve(snapshot.edges.size() * 2);

    for (const auto& vertex_entry : snapshot.vertices)
    {
        const bool manually_updated =
            vertex_entry.extra_data.vertex_type_ == WSVertexExtraData::VertexType::VIEWPOINT ||
            vertex_entry.extra_data.vertex_type_ == WSVertexExtraData::VertexType::REGION_CENTER;
        id_vertex_map.emplace(vertex_entry.id, VertexVizInfo{vertex_entry.pos, vertex_entry.active,
                                                             manually_updated});

        if (!vertex_entry.active)
            vis_utils::marker_utils::appendPoint(inactive_vertices, vertex_entry.pos);
        else
            vis_utils::marker_utils::appendPoint(vertices, vertex_entry.pos);
    }

    for (const auto& edge_entry : snapshot.edges)
    {
        const int from_id = edge_entry.from_id;
        const int to_id = edge_entry.to_id;

        const auto from_iter = id_vertex_map.find(from_id);
        const auto to_iter = id_vertex_map.find(to_id);
        if (from_iter == id_vertex_map.end() || to_iter == id_vertex_map.end())
            continue;

        const auto& s_vp = from_iter->second;
        const auto& t_vp = to_iter->second;

        if (!edge_entry.active || !s_vp.active || !t_vp.active)
        {
            vis_utils::marker_utils::appendLine(inactive_edges, s_vp.pos, t_vp.pos);
        }
        else if (s_vp.manually_updated || t_vp.manually_updated)
        {
            vis_utils::marker_utils::appendLine(manual_edges, s_vp.pos, t_vp.pos);
        }
        else
        {
            vis_utils::marker_utils::appendLine(auto_edges, s_vp.pos, t_vp.pos);
        }
    }

    vis_utils::marker_utils::assignMarkers(
        graph_markers, {vertices, auto_edges, inactive_vertices, inactive_edges, manual_edges});
}

/**
 * @brief Generate the markers of the points
 *
 * @tparam PointListType the type of the points
 * @param points the points that need to generate the markers
 * @param color the color of the markers
 * @param scale the scale of the markers
 * @return visualization_msgs::MarkerArray the markers of the points
 */
template <typename PointListType>
visualization_msgs::MarkerArray
WSRoadMap::generatePointsMarkers(const PointListType& points, const eigen_utils::Vec4d& color,
                                 const eigen_utils::Vec3d& scale) const
{
    visualization_msgs::Marker vertices = vis_utils::marker_utils::makePointMarker(
        wsgp_.frame_id_, "WSRoadMap", 0,
        vis_utils::marker_utils::makeScale(scale.x(), scale.y(), scale.z()),
        vis_utils::marker_utils::toRosColor(color));

    for (const auto& node : points)
        vis_utils::marker_utils::appendPoint(vertices, node);

    visualization_msgs::MarkerArray graph_markers;
    vis_utils::marker_utils::assignMarkers(graph_markers, {vertices});
    return graph_markers;
}

#ifndef NDEBUG
/**
 * @brief Debug helper: print graph statistics (active vertices, positions).
 */
void WSRoadMap::test()
{
    int active_num = 0;
    eigen_utils::Vec3fMap<int, 3> vertex_poses;

    for (const auto& [v_id, vertex] : graph_.getAllVertices())
    {
        if (vertex.active_)
        {
            active_num++;
            vertex_poses[vertex.pos_] = 0;
        }
    }

    ROS_WARN("The active vertices num in graph is %d, %zu", active_num, vertex_poses.size());

    KD_TREE<PointType>::PointVector active_points, box_pts;
    active_points_ikdtree_->flatten(active_points_ikdtree_->Root_Node, active_points, NOT_RECORD);

    BoxPointType box;
    for (int i = 0; i < 3; i++)
    {
        box.vertex_min[i] = -0.1;
        box.vertex_max[i] = 3.1;
    }
    active_points_ikdtree_->Box_Search(box, box_pts);

    ROS_WARN("The active points num in kd_tree is %zu", active_points.size());

    if (vertex_poses.size() != active_points.size())
    {
        ROS_ERROR(
            "The active vertices num in graph is not equal to the active points num in kd_tree");

        for (const auto pt : active_points)
        {
            eigen_utils::Vec3f pos(pt.x, pt.y, pt.z);
            if (vertex_poses.find(pos) == vertex_poses.end())
                ROS_ERROR_STREAM("The point is not in the graph: " << pos.transpose());
            else
                vertex_poses[pos] += 1;
        }

        for (const auto& [pos, num] : vertex_poses)
        {
            if (num == 0)
                ROS_ERROR_STREAM("The vertex is not in the kd_tree: " << pos.transpose());
        }
    }
}
#endif
} // namespace map_process
