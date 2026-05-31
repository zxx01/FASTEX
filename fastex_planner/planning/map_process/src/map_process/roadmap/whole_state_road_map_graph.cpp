/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-09-19 12:09:37
 * @LastEditTime: 2026-05-02 21:39:03
 * @Description:
 */

#include "map_process/roadmap/whole_state_road_map.h"

#include "map_process/core/map_process_constants.h"
#include <vis_utils/marker_utils.h>
#include "utils/hash_grid.hpp"

#include <random>

namespace map_process
{

/**
 * @brief Add fixed sample points to the topological graph.
 *
 * Checks if each point is already in the graph; new points are inserted
 * and connected to neighbours within the connectable range.
 *
 * @param fixed_points Points to add.
 * @param resolution Neighbour-search resolution.
 */
void WSRoadMap::addFixedPointsToGraph(const eigen_utils::Vec_Vec3f& fixed_points,
                                      const eigen_utils::Vec3f resolution)
{
    if (!is_graph_initialized_)
        initTopoGraph(wsgp_.initial_position_);

    // 1. Add the fixed points to the graph
    PointVector nearest_points;
    std::vector<float> nearest_dists;

    for (const auto& point : fixed_points)
    {
        if (!process_utils::ProcessUtils::isInBox<eigen_utils::Vec3d>(
                point.cast<double>(), samplepoint_min_bound_, samplepoint_max_bound_))
        {
            continue;
        }

        // Check if the point is already in the graph
        fixed_points_ikdtree_->Nearest_Search(PointType(point.x(), point.y(), point.z()), 1,
                                              nearest_points, nearest_dists, 0.5);
        if (!nearest_points.empty())
            continue;

        // If the point is not in the graph, add it to the graph
        WSVertexExtraData vertex_extra_data(WSVertexExtraData::VertexSource::FIXED,
                                            WSVertexExtraData::VertexState::UNKNOWN,
                                            WSVertexExtraData::VertexType::COMMON);
        auto [id, flag] = graph_.addVertex(VertexType(point, true, vertex_extra_data), true);

        // Build or update the kdtree for the fixed points
        if (flag) // if the point is newly added
        {
            PointVector fixed_points_vector;
            fixed_points_vector.emplace_back(point.x(), point.y(), point.z());

            // Build or update the kdtree for the fixed points
            if (!is_fixed_points_ikdtree_built_)
            {
                fixed_points_ikdtree_->Build(fixed_points_vector);
                is_fixed_points_ikdtree_built_ = true;
            }
            else
            {
                fixed_points_ikdtree_->Add_Points(fixed_points_vector, false);
            }

            // Build or update the kdtree for the active points
            if (!is_active_points_ikdtree_built_)
            {
                active_points_ikdtree_->Build(fixed_points_vector);
                is_active_points_ikdtree_built_ = true;
            }
            else
            {
                active_points_ikdtree_->Add_Points(fixed_points_vector, false);
            }
        }
    }

    // 2. Connect the fixed points to their neighbors in the graph directly
    BoxPointType bbox;
    eigen_utils::Vec3f box_margin(0.1, 0.1, 0.1);
    eigen_utils::Vec3f box_min, box_max;

    PointVector neighbor_points;
    eigen_utils::Vec3f neighbor_point;
    int point_id, neighbor_point_id;

    for (const auto& point : fixed_points)
    {
        box_min = point - resolution - box_margin;
        box_max = point + resolution + box_margin;
        for (int i = 0; i < 3; ++i)
        {
            bbox.vertex_min[i] = box_min[i];
            bbox.vertex_max[i] = box_max[i];
        }

        fixed_points_ikdtree_->Box_Search(bbox, neighbor_points);

        for (const auto& pt : neighbor_points)
        {
            neighbor_point << pt.x, pt.y, pt.z;
            if (point.isApprox(neighbor_point, 1e-3))
                continue;

            point_id = graph_.getVertexId(point);
            neighbor_point_id = graph_.getVertexId(neighbor_point);

            if (point_id > neighbor_point_id)
            {
                process_utils::CubeBox bbox;
                bbox.min_ = point.cwiseMin(neighbor_point).cast<double>();
                bbox.max_ = point.cwiseMax(neighbor_point).cast<double>();

                WSEdgeExtraData edge_extra_data(bbox, false, true);
                graph_.addTwoWayEdge(
                    point_id, neighbor_point_id,
                    EdgeType((neighbor_point - point).norm(), true, edge_extra_data));
            }
        }
    }
}

/**
 * @brief Update the state of existing vertices and edges connections within a specified box
 *
 * This function updates the state of vertices and edges within a specified bounding box.
 * It first identifies the vertices within the box and updates their states based on the
 * occupancy information from the SDF map. It then deletes vertices located in obstacles
 * and updates the kdtree for fixed and free points. Finally, it updates the edges connected
 * to the valid vertices, checking for collision-free paths and updating their states.
 *
 * @param box_min The minimum corner of the bounding box
 * @param box_max The maximum corner of the bounding box
 * @param frontiers_boxes A vector of frontier boxes used for updating edge states
 */

void WSRoadMap::updateExistingVerticesStateAndEdgesConnection(
    const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max,
    const std::vector<process_utils::CubeBox>& frontiers_boxes)
{
    if (!is_graph_initialized_)
        return;

    // 1.Find the vertices in the update box
    eigen_utils::Vec_Vec3f update_points;
    std::vector<int> update_indices =
        graph_.getBoxNeighborVertexsIDs(box_min, box_max, update_points);
    if (update_indices.empty())
        return;

    // 2.Record the vertices located in the obstacle and update the vertex state for the rest
    std::vector<int> valid_points_indices, delete_points_indices, add_active_points_indices;
    for (size_t i = 0; i < update_indices.size(); i++)
    {
        const int vertex_id = update_indices[i];
        VertexType& cur_vertex = graph_.getVertex(vertex_id);

        // Skip the vertex that needs to be updated manually
        if (isManuallyUpdatedVertex(cur_vertex))
            continue;

        const eigen_utils::Vec3f& pt = cur_vertex.pos_;

        if (sdf_map_->getInflateOccupancy(eigen_utils::Vec3d(pt.x(), pt.y(), pt.z())) != 0)
        {
            // Delete the vertex if it is located in the obstacle
            delete_points_indices.push_back(vertex_id);
        }
        else
        {
            cur_vertex.extra_data_.last_vertex_state_ = cur_vertex.extra_data_.cur_vertex_state_;

            if (sdf_map_->getOccupancy(eigen_utils::Vec3d(pt.x(), pt.y(), pt.z())) ==
                sdf_map_->FREE)
            {
                // Update the vertex state based on the occupancy information ---- free
                cur_vertex.extra_data_.cur_vertex_state_ = WSVertexExtraData::VertexState::FREE;

                if (!cur_vertex.active_)
                {
                    cur_vertex.active_ = true;
                    add_active_points_indices.push_back(vertex_id);
                }
            }
            else if (sdf_map_->getOccupancy(eigen_utils::Vec3d(pt.x(), pt.y(), pt.z())) ==
                     sdf_map_->UNKNOWN)
            {
                // Update the vertex state based on the occupancy information ---- unknown
                cur_vertex.extra_data_.cur_vertex_state_ = WSVertexExtraData::VertexState::UNKNOWN;
            }

            valid_points_indices.push_back(vertex_id);
        }
    }

    // 3.Update the kdtree for the active points

    if (!add_active_points_indices.empty())
    {
        PointVector add_active_points;
        for (const int vertex_id : add_active_points_indices)
        {
            const VertexType& cur_vertex = graph_.getVertex(vertex_id);
            add_active_points.emplace_back(cur_vertex.pos_.x(), cur_vertex.pos_.y(),
                                           cur_vertex.pos_.z());
        }

        active_points_ikdtree_->Add_Points(add_active_points, false);
    }

    // 4.Delete the vertices and update the kdtree for the fixed and active points
    if (!delete_points_indices.empty())
    {
        PointVector fixed_delete_points, active_delete_points;
        for (const int vertex_id : delete_points_indices)
        {
            const VertexType& cur_vertex = graph_.getVertex(vertex_id);
            if (cur_vertex.extra_data_.vertex_source_ == WSVertexExtraData::VertexSource::FIXED)
                fixed_delete_points.emplace_back(cur_vertex.pos_.x(), cur_vertex.pos_.y(),
                                                 cur_vertex.pos_.z());

            active_delete_points.emplace_back(cur_vertex.pos_.x(), cur_vertex.pos_.y(),
                                              cur_vertex.pos_.z());

            graph_.deleteVertex(vertex_id, true);
        }

        if (!fixed_delete_points.empty())
            fixed_points_ikdtree_->Delete_Points(fixed_delete_points);

        if (!active_delete_points.empty())
            active_points_ikdtree_->Delete_Points(active_delete_points);
    }

    // 5. Build the AABB tree for the frontier boxes
    aabb::Tree frontier_aabb_tree(3, 0.0, frontiers_boxes.size() + 1, true);
    for (size_t i = 0; i < frontiers_boxes.size(); i++)
    {
        const eigen_utils::Vec3d& box_min = frontiers_boxes[i].min_;
        const eigen_utils::Vec3d& box_max = frontiers_boxes[i].max_;
        std::vector<double> lb(box_min.data(), box_min.data() + box_min.size());
        std::vector<double> ub(box_max.data(), box_max.data() + box_max.size());
        frontier_aabb_tree.insertParticle(i, lb, ub);
    }

    // 6. Update the edges connected to the valid vertices.
    //    Update active && cross-unknown edges (i.e., edges that intersect with the cluster box from
    //    the last update), and inactive edges whose bounding box intersects with the current
    //    cluster's AABB box. Also update the overlap_frontiers_ and cross_unknown_ states of these
    //    edges. Add these edges to an update list.

    std::unordered_map<int, std::unordered_set<int>> update_edges_map;
    for (const int cur_vertex_id : valid_points_indices)
    {
        // Get the edges linked to the vertex
        std::unordered_map<int, EdgeType>& linked_edges = graph_.getLinkedEdges(cur_vertex_id);
        if (linked_edges.empty())
            continue;

        const VertexType& cur_vertex = graph_.getVertex(cur_vertex_id);

        // Skip the edge connecting to vertex that needs to be updated manually
        if (isManuallyUpdatedVertex(cur_vertex))
            continue;

        std::vector<std::pair<int, int>> delete_edges;

        for (auto& [neighbor_vertex_id, edge] : linked_edges)
        {
            const VertexType& neighbor_vertex = graph_.getVertex(neighbor_vertex_id);

            if (isManuallyUpdatedVertex(neighbor_vertex))
                continue;

            // Skip the edge if it is already in the update list
            const int smaller_vertex_id = std::min(cur_vertex_id, neighbor_vertex_id);
            const int larger_vertex_id = std::max(cur_vertex_id, neighbor_vertex_id);
            auto it = update_edges_map.find(smaller_vertex_id);
            if (it != update_edges_map.end() && it->second.count(larger_vertex_id))
                continue;

            bool updated = false, deleted = false;

            // If last cross_unknown state is true, update the edge
            if (edge.extra_data_.cross_unknown_)
            {
                // update 'overlap_frontiers' state of the edge
                const process_utils::CubeBox& edge_bbox = edge.extra_data_.bbox_;
                std::vector<double> lb(edge_bbox.min_.data(),
                                       edge_bbox.min_.data() + edge_bbox.min_.size());
                std::vector<double> ub(edge_bbox.max_.data(),
                                       edge_bbox.max_.data() + edge_bbox.max_.size());
                std::vector<unsigned int> overlap_aabbs =
                    frontier_aabb_tree.query(aabb::AABB(lb, ub));
                edge.extra_data_.overlap_frontiers_ = !overlap_aabbs.empty();

                if (edge.active_ || edge.extra_data_.overlap_frontiers_)
                {
                    // If the edge is not collision-free, delete it
                    fast_planner::SDFMap::CollisionCheckResult result =
                        sdf_map_->isOptimisticInflatedCollisionFreeStraight(
                            cur_vertex.pos_.cast<double>(), neighbor_vertex.pos_.cast<double>());
                    if (!result.is_collision_free)
                    {
                        deleted = true;
                        delete_edges.emplace_back(cur_vertex_id, neighbor_vertex_id);
                    }

                    // update 'cross_unknown' state of the edge
                    edge.extra_data_.cross_unknown_ = result.crosses_unknown;

                    updated = true;
                }
            }
            else if (!edge.active_)
            {
                ROS_ERROR("ERROR: edge is inactive but cross_unknown is false!");
                edge.active_ = true;
            }

            // Add the vertices id of the edge to the update list if it is updated
            if (updated && !deleted)
                update_edges_map[smaller_vertex_id].insert(larger_vertex_id);
        }

        // Delete the edges that are not collision-free
        for (auto& edge : delete_edges)
            graph_.deleteTwoWayEdge(edge.first, edge.second);
    }

    // 7. Update the active state of the edges in the update list.
    //    Update the active state of the edges in the update list, considering only edges with at
    //    least one vertex in the free state. If the edge's cross_unknown_ is true and
    //    overlap_frontiers_ is false, set active to false, otherwise set it to true.

    for (const auto& pair : update_edges_map)
    {
        const int cur_vertex_id = pair.first;
        const std::unordered_set<int>& neighbor_vertex_ids = pair.second;

        std::unordered_map<int, EdgeType>& cur_edges = graph_.getLinkedEdges(cur_vertex_id);

        for (const int neighbor_vertex_id : neighbor_vertex_ids)
        {
            // Skip the edge if it is not in the graph
            auto edge_iter = cur_edges.find(neighbor_vertex_id);
            if (edge_iter == cur_edges.end())
                continue;

            const VertexType& cur_vertex = graph_.getVertex(cur_vertex_id);
            const VertexType& neighbor_vertex = graph_.getVertex(neighbor_vertex_id);

            if (cur_vertex.extra_data_.cur_vertex_state_ == WSVertexExtraData::VertexState::FREE ||
                neighbor_vertex.extra_data_.cur_vertex_state_ ==
                    WSVertexExtraData::VertexState::FREE)
            {
                EdgeType& edge = edge_iter->second;

                if (edge.extra_data_.cross_unknown_ && !edge.extra_data_.overlap_frontiers_)
                    edge.active_ = false;
                else
                    edge.active_ = true;
            }
        }
    }
}

/**
 * @brief Grow the topological graph by sampling points within a specified box.
 *
 * Samples points in the local area, filters out those in obstacles, unknown
 * areas, out of bounds, or too close to existing nodes. Adds suitable points
 * to the graph and connects them to neighbours.
 *
 * @param box_min Minimum corner of the bounding box.
 * @param box_max Maximum corner of the bounding box.
 */
void WSRoadMap::growingTopoGraphBySamplePointsWithinBox(const eigen_utils::Vec3f& box_min,
                                                        const eigen_utils::Vec3f& box_max)
{
    if (!is_graph_initialized_)
        initTopoGraph(wsgp_.initial_position_);

    // 1.Sample points in the local area
    eigen_utils::Vec_Vec3f sample_points;
    eigen_utils::Vec3f sample_box_min = box_min, sample_box_max = box_max;
    sample_box_min.z() = planner_dim_ == 2 ? wsgp_.initial_position_.z() : box_min.z();
    sample_box_max.z() = planner_dim_ == 2 ? wsgp_.initial_position_.z() : box_max.z();

    process_utils::ProcessUtils::samplePointsInCuboid(
        sample_box_min, sample_box_max,
        eigen_utils::Vec3f(wsgp_.sample_dist_, wsgp_.sample_dist_, wsgp_.sample_dist_),
        sample_points);

    // Add noise to the sample points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, 0.5 * wsgp_.sample_dist_);

    for (auto& point : sample_points)
    {
        point.x() += dist(gen);
        point.y() += dist(gen);
        point.z() += dist(gen);
    }

    eigen_utils::Vec_Vec3f suitable_points, selected_points;

    // 2.Filter the sample points — remove those in obstacle/unknown area,
    // out of the boundary or too close to the local graph node, and add the suitable points to the
    // graph
    eigen_utils::Vec3f search_box_min, search_box_max;
    search_box_min =
        box_min - eigen_utils::Vec3f(wsgp_.connectable_range_, wsgp_.connectable_range_,
                                     wsgp_.connectable_range_);
    search_box_max =
        box_max + eigen_utils::Vec3f(wsgp_.connectable_range_, wsgp_.connectable_range_,
                                     wsgp_.connectable_range_);
    graph_.getBoxNeighborVertexsIDs(search_box_min, search_box_max, selected_points);

    PointVector add_points_vector;

    eigen_utils::Vec3d box_margin(wsgp_.bound_margin_, wsgp_.bound_margin_, wsgp_.bound_margin_);
    eigen_utils::Vec3d global_bbox_min = samplepoint_min_bound_ + box_margin;
    eigen_utils::Vec3d global_bbox_max = samplepoint_max_bound_ - box_margin;

    utils::HashGridManager<eigen_utils::Vec_Vec3f> selected_points_hash_grid(
        eigen_utils::Vec3d::Zero(), wsgp_.min_interval_);
    std::vector<VertexType> add_vertices;
    static const eigen_utils::Vec_Vec3i axis_neighbors = {
        {0, 0, 0},   {1, 0, 0},   {-1, 0, 0}, {0, 1, 0},   {0, -1, 0},  {0, 0, 1},   {0, 0, -1},
        {1, 1, 0},   {1, -1, 0},  {-1, 1, 0}, {-1, -1, 0}, {1, 0, 1},   {1, 0, -1},  {-1, 0, 1},
        {-1, 0, -1}, {0, 1, 1},   {0, 1, -1}, {0, -1, 1},  {0, -1, -1}, {1, 1, 1},   {1, 1, -1},
        {1, -1, 1},  {1, -1, -1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1}};

    for (const eigen_utils::Vec3f& point : selected_points)
    {
        selected_points_hash_grid
            .getOrCreateGridData(selected_points_hash_grid.getGridIndex(point.cast<double>()))
            .push_back(point);
    }

    for (const eigen_utils::Vec3f& point : sample_points)
    {
        bool suitable = true;
        bool region_explored = true;

        // make sure the new point is not in or near the obstacle
        if (!process_utils::ProcessUtils::isInBox<eigen_utils::Vec3d>(
                point.cast<double>(), global_bbox_min, global_bbox_max) ||
            sdf_map_->getInflateOccupancy(eigen_utils::Vec3d(point.x(), point.y(), point.z())) != 0)
        {
            suitable = false;
        }

        // make sure the new point is not too close to the local graph node

        eigen_utils::Vec3i sample_index_in_grid =
            selected_points_hash_grid.getGridIndex(point.cast<double>());
        if (suitable)
        {
            double square_min_interval = wsgp_.min_interval_ * wsgp_.min_interval_;

            for (const auto& offset : axis_neighbors)
            {
                auto neighbor_index = sample_index_in_grid + offset;
                auto data = selected_points_hash_grid.getGridData(neighbor_index);

                if (data.has_value())
                {
                    for (const auto& nearby : *data)
                    {
                        if ((point - nearby).squaredNorm() < square_min_interval)
                        {
                            suitable = false;
                        }
                    }
                }
            }
        }

        // make sure the new point is in the valid region
        if (suitable)
        {
            std::vector<double> lb = {point.x(), point.y(), point.z()};
            std::vector<double> ub = {point.x(), point.y(), point.z()};
            std::vector<unsigned int> overlap_aabb_ids =
                valid_region_aabb_tree_->query(aabb::AABB(lb, ub));
            suitable = !overlap_aabb_ids.empty();

            if (suitable)
                region_explored = valid_region_explored_state_.at(overlap_aabb_ids[0]);
        }

        if (suitable)
        {
            suitable_points.push_back(point);
            selected_points_hash_grid.getOrCreateGridData(sample_index_in_grid).push_back(point);
            add_points_vector.emplace_back(point.x(), point.y(), point.z());
            selected_points.push_back(point);

            const int occ =
                sdf_map_->getOccupancy(eigen_utils::Vec3d(point.x(), point.y(), point.z()));
            WSVertexExtraData::VertexState state = (occ == sdf_map_->UNKNOWN)
                                                       ? WSVertexExtraData::VertexState::UNKNOWN
                                                       : WSVertexExtraData::VertexState::FREE;

            WSVertexExtraData vertex_extra_data(WSVertexExtraData::VertexSource::SAMPLE, state,
                                                WSVertexExtraData::VertexType::COMMON);

            bool point_active =
                state == WSVertexExtraData::VertexState::UNKNOWN ? !region_explored : true;

            add_vertices.emplace_back(point, point_active, vertex_extra_data);
        }
    }

    graph_.addVertices(add_vertices, true);

    // 3.Build the kdtree for the active points
    if (!add_points_vector.empty())
    {
        if (!is_active_points_ikdtree_built_)
        {
            active_points_ikdtree_->Build(add_points_vector);
            is_active_points_ikdtree_built_ = true;
        }
        else
        {
            active_points_ikdtree_->Add_Points(add_points_vector, false);
        }
    }

    // 4.Update the connections of the selected points
    std::vector<int> selected_points_ids;
    selected_points_ids.reserve(selected_points.size());
    for (const auto& point : selected_points)
        selected_points_ids.push_back(graph_.getVertexId(point));
    updateGraphVerticesOptimisticConnections(selected_points_ids);

    // 5.Delete the points that are not connected to the graph
    checkAndEraseIsolatedVerticesWithinBox(search_box_min, search_box_max);
}

/**
 * @brief Check and erase isolated vertices within a specified box.
 *
 * This function identifies vertices within the specified bounding box that have no connected edges
 * and deletes them from the graph. It also removes these vertices from the active and fixed points
 * kd-trees.
 *
 * @param box_min The minimum corner of the bounding box.
 * @param box_max The maximum corner of the bounding box.
 */
void WSRoadMap::checkAndEraseIsolatedVerticesWithinBox(const eigen_utils::Vec3f& box_min,
                                                       const eigen_utils::Vec3f& box_max)
{
    eigen_utils::Vec_Vec3f selected_points;
    std::vector<int> selected_ids =
        graph_.getBoxNeighborVertexsIDs(box_min, box_max, selected_points);
    if (selected_ids.empty())
        return;

    std::vector<int> delete_ids;
    for (const int vertex_id : selected_ids)
    {
        if (graph_.getLinkedEdgesNum(vertex_id) == 0)
            delete_ids.push_back(vertex_id);
    }

    PointVector delete_points_vector;

    for (const int vertex_id : delete_ids)
    {
        const VertexType& vertex = graph_.getVertex(vertex_id);
        if (isManuallyUpdatedVertex(vertex))
            continue;

        delete_points_vector.emplace_back(vertex.pos_.x(), vertex.pos_.y(), vertex.pos_.z());
        graph_.deleteVertex(vertex_id, true);
    }

    if (!delete_points_vector.empty())
    {
        active_points_ikdtree_->Delete_Points(delete_points_vector);
        fixed_points_ikdtree_->Delete_Points(delete_points_vector);
    }
}

/**
 * @brief Inactivate vertices with unknown state within a specified box
 *
 * This function deactivates vertices that are in the UNKNOWN state within the specified
 * bounding box. It first retrieves the vertices within the box and then sets their
 * active status to false if their current state is UNKNOWN.
 *
 * @param box_min The minimum corner of the bounding box
 * @param box_max The maximum corner of the bounding box
 */
void WSRoadMap::inactivateUnknownStateVerticesInBox(const eigen_utils::Vec3f& box_min,
                                                    const eigen_utils::Vec3f& box_max)
{
    eigen_utils::Vec_Vec3f selected_points;
    std::vector<int> selected_ids =
        graph_.getBoxNeighborVertexsIDs(box_min, box_max, selected_points);
    if (selected_ids.empty())
        return;

    PointVector delete_points_vector;

    for (const int vertex_id : selected_ids)
    {
        VertexType& vertex = graph_.getVertex(vertex_id);

        // Skip the vertex that is already inactive
        if (!vertex.active_)
            continue;

        // Inactivate the vertex if its state is UNKNOWN
        if (vertex.extra_data_.cur_vertex_state_ == WSVertexExtraData::VertexState::UNKNOWN)
        {
            vertex.active_ = false;
            delete_points_vector.emplace_back(vertex.pos_.x(), vertex.pos_.y(), vertex.pos_.z());
        }
    }

    if (!delete_points_vector.empty())
        active_points_ikdtree_->Delete_Points(delete_points_vector);

    eigen_utils::Vec3f box_center = (box_min + box_max) * 0.5f;
    std::vector<double> lb = {box_center.x(), box_center.y(), box_center.z()};
    std::vector<double> ub = {box_center.x(), box_center.y(), box_center.z()};
    std::vector<unsigned int> overlap_aabb_ids = valid_region_aabb_tree_->query(aabb::AABB(lb, ub));
    if (!overlap_aabb_ids.empty())
        valid_region_explored_state_[overlap_aabb_ids[0]] = true;
}

/**
 * @brief Activate vertices with unknown state within a specified box
 *
 * This function activates vertices that are in the UNKNOWN state within the specified
 * bounding box. It first retrieves the vertices within the box and then sets their
 * active status to true if their current state is UNKNOWN.
 *
 * @param box_min The minimum corner of the bounding box
 * @param box_max The maximum corner of the bounding box
 */
void WSRoadMap::activateUnknownStateVerticesInBox(const eigen_utils::Vec3f& box_min,
                                                  const eigen_utils::Vec3f& box_max)
{
    eigen_utils::Vec_Vec3f selected_points;
    std::vector<int> selected_ids =
        graph_.getBoxNeighborVertexsIDs(box_min, box_max, selected_points);
    if (selected_ids.empty())
        return;

    PointVector add_points_vector;

    for (const int vertex_id : selected_ids)
    {
        VertexType& vertex = graph_.getVertex(vertex_id);

        // Skip the vertex that is already active
        if (vertex.active_)
            continue;

        // Activate the vertex if its state is UNKNOWN
        if (vertex.extra_data_.cur_vertex_state_ == WSVertexExtraData::VertexState::UNKNOWN)
        {
            vertex.active_ = true;
            add_points_vector.emplace_back(vertex.pos_.x(), vertex.pos_.y(), vertex.pos_.z());
        }
    }

    if (!add_points_vector.empty())
    {
        if (!is_active_points_ikdtree_built_)
        {
            active_points_ikdtree_->Build(add_points_vector);
            is_active_points_ikdtree_built_ = true;
        }
        else
        {
            active_points_ikdtree_->Add_Points(add_points_vector, false);
        }
    }

    eigen_utils::Vec3f box_center = (box_min + box_max) * 0.5f;
    std::vector<double> lb = {box_center.x(), box_center.y(), box_center.z()};
    std::vector<double> ub = {box_center.x(), box_center.y(), box_center.z()};
    std::vector<unsigned int> overlap_aabb_ids = valid_region_aabb_tree_->query(aabb::AABB(lb, ub));
    if (!overlap_aabb_ids.empty())
        valid_region_explored_state_[overlap_aabb_ids[0]] = false;
}

/**
 * @brief Update the graph vertices with strict free connections
 *
 * This function updates the graph vertices by connecting them to their neighbors within a specified
 * range. It ensures that the connections are collision-free and that each vertex is connected to a
 * limited number of neighbors.
 *
 * @param vertex_ids The IDs of the vertices to be updated
 */
void WSRoadMap::updateGraphVerticesStrictFreeConnections(const std::vector<int>& vertex_ids)
{
    for (const auto& vertex_id : vertex_ids)
    {
        if (!graph_.isVertexExisted(vertex_id))
        {
            ROS_ERROR("The vertex id %d is not existed in the graph!", vertex_id);
            continue;
        }

        // If the vertex has been connected to enough neighbors, skip it
        if (graph_.getLinkedEdgesNum(vertex_id) > wsgp_.connectable_num_)
            continue;

        // search the neighbors of the point
        eigen_utils::Vec3f vertex = graph_.getVertexPos(vertex_id);
        eigen_utils::Vec_Vec3f neighbor_vertexs;
        std::vector<int> neighbors_ids =
            graph_.getRadiusNeighborVertexsIDs(vertex, wsgp_.connectable_range_, neighbor_vertexs);
        if (neighbors_ids.empty())
            continue;

        // Use a priority queue to keep the neighbors sorted
        using Neighbor = std::pair<int, eigen_utils::Vec3f>;
        auto compare = [&](const Neighbor& a, const Neighbor& b) {
            return (a.second - vertex).squaredNorm() > (b.second - vertex).squaredNorm();
        };
        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(compare)> sorted_results(
            compare);

        for (size_t i = 0; i < neighbors_ids.size(); i++)
            sorted_results.push(std::make_pair(neighbors_ids[i], neighbor_vertexs[i]));

        // connect the point to the graph by searching the neighbors
        int connected_num = 0;
        while (!sorted_results.empty() && connected_num <= wsgp_.connectable_num_)
        {
            auto index_point_pair = sorted_results.top();
            sorted_results.pop();
            const int neighbor_id = index_point_pair.first;
            eigen_utils::Vec3f neighbor_vertex = index_point_pair.second;

            if (vertex_id != neighbor_id)
            {
                fast_planner::SDFMap::CollisionCheckResult result =
                    sdf_map_->isStrictInflatedCollisionFreeStraight(vertex.cast<double>(),
                                                                    neighbor_vertex.cast<double>());

                if (result.is_collision_free)
                {
                    process_utils::CubeBox bbox;
                    bbox.min_ = vertex.cwiseMin(neighbor_vertex).cast<double>();
                    bbox.max_ = vertex.cwiseMax(neighbor_vertex).cast<double>();

                    WSEdgeExtraData edge_extra_data(bbox, false, false);
                    graph_.addTwoWayEdge(
                        vertex_id, neighbor_id,
                        EdgeType((neighbor_vertex - vertex).norm(), true, edge_extra_data));

                    connected_num++;
                }
            }
        }
    }
}

/**
 * @brief Update the graph vertices with optimistic connections
 *
 * This function updates the graph by connecting the specified vertices to their
 * neighbors within a certain range. It ensures that each vertex is connected to
 * a limited number of neighbors and checks for collision-free paths before adding
 * edges to the graph.
 *
 * @param vertex_ids A vector of vertex IDs to be updated
 */
void WSRoadMap::updateGraphVerticesOptimisticConnections(const std::vector<int>& vertex_ids)
{
    for (const auto& vertex_id : vertex_ids)
    {
        if (!graph_.isVertexExisted(vertex_id))
        {
            ROS_ERROR("The vertex id %d is not existed in the graph!", vertex_id);
            continue;
        }

        // If the vertex has been connected to enough neighbors, skip it
        int cnt = graph_.getLinkedEdgesNum(vertex_id);
        if (cnt >= wsgp_.connectable_num_)
            continue;

        // search the neighbors of the point
        eigen_utils::Vec3f vertex = graph_.getVertexPos(vertex_id);
        eigen_utils::Vec_Vec3f neighbor_vertexs;

        std::vector<int> neighbors_ids =
            graph_.getNearestVertex(vertex, wsgp_.connectable_num_ + 1, wsgp_.connectable_range_);
        if (neighbors_ids.empty())
            continue;

        for (const auto& neighbor_id : neighbors_ids)
        {
            if (vertex_id == neighbor_id || graph_.getLinkedEdges(vertex_id).find(neighbor_id) !=
                                                graph_.getLinkedEdges(vertex_id).end())
                continue;

            const eigen_utils::Vec3f& neighbor_vertex = graph_.getVertexPos(neighbor_id);
            fast_planner::SDFMap::CollisionCheckResult result =
                sdf_map_->isOptimisticInflatedCollisionFreeStraight(vertex.cast<double>(),
                                                                    neighbor_vertex.cast<double>());

            if (result.is_collision_free)
            {
                process_utils::CubeBox bbox;
                bbox.min_ = vertex.cwiseMin(neighbor_vertex).cast<double>();
                bbox.max_ = vertex.cwiseMax(neighbor_vertex).cast<double>();

                WSEdgeExtraData edge_extra_data(bbox, false, result.crosses_unknown);
                graph_.addTwoWayEdge(
                    vertex_id, neighbor_id,
                    EdgeType((neighbor_vertex - vertex).norm(), true, edge_extra_data));

                if (++cnt >= wsgp_.connectable_num_)
                    break;
            }
        }
    }
}

/**
 * @brief Inserts a vertex into the whole state road map.
 *
 * This function inserts a vertex into the whole state road map and returns a pair containing the
 * vertex ID and a boolean indicating whether the insertion was successful.
 *
 * @param vertex The vertex to be inserted, of type VertexType.
 * @return std::pair<int, bool> A pair containing the vertex ID and a boolean indicating whether the
 * insertion was successful.
 */
std::pair<int, bool> WSRoadMap::insertVertex(const VertexType& vertex)
{
    return graph_.addVertex(vertex, true);
}

/**
 * @brief Deletes a vertex from the topological road map.
 *
 * This function removes a vertex identified by its ID from the topological road map.
 * All the edges connected to the vertex are also removed.
 *
 * @param vertex_id The ID of the vertex to be deleted.
 */
void WSRoadMap::deleteVertex(const int& vertex_id) { graph_.deleteVertex(vertex_id, true); }

/**
 * @brief Adds a bidirectional edge between two vertices in the topological road map.
 *
 * This function adds a bidirectional (two-way) edge between the vertices identified by their IDs.
 * The edge has an associated cost.
 *
 * @param start_vertex_id The ID of the starting vertex.
 * @param end_vertex_id The ID of the ending vertex.
 * @param cost The cost associated with the edge.
 * @return true if the edge was successfully added, false otherwise.
 */
bool WSRoadMap::addTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id,
                              const EdgeType& edge)
{
    return graph_.addTwoWayEdge(start_vertex_id, end_vertex_id, edge);
}

/**
 * @brief Deletes a bidirectional edge between two vertices in the topological road map.
 *
 * This function removes a bidirectional (two-way) edge between the vertices identified by their
 * IDs.
 *
 * @param start_vertex_id The ID of the starting vertex.
 * @param end_vertex_id The ID of the ending vertex.
 * @return true if the edge was successfully deleted, false otherwise.
 */
bool WSRoadMap::deleteTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id)
{
    return graph_.deleteTwoWayEdge(start_vertex_id, end_vertex_id);
}

} // namespace map_process
