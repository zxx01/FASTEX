/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-12-12 14:59:38
 * @LastEditTime: 2026-05-31 12:02:36
 * @Description:
 */

#ifndef HISTORY_POS_GRAPH_H
#define HISTORY_POS_GRAPH_H

#include <memory>
#include <stdexcept>

#include <visualization_msgs/MarkerArray.h>

#include "common_utils/eigen_utils.h"
#include "map_process/core/map_process_constants.h"
#include "map_process/core/map_process_params.h"
#include "map_process/searcher/path_searcher.h"
#include "utils/plan_graph_ikdtree.h"
#include <vis_utils/marker_utils.h>

namespace map_process
{
class HistoryPosGraph
{
  public:
    using SharedPtr = std::shared_ptr<HistoryPosGraph>;
    using UniquePtr = std::unique_ptr<HistoryPosGraph>;

    using GraphVertexType = IkdTreePlanGraph<void, void>::VertexType;
    using GraphEdgeType = IkdTreePlanGraph<void, void>::EdgeType;

    HistoryPosGraph();
    ~HistoryPosGraph() = default;

    void setPathSearcher(PathSearcher::SharedPtr path_searcher) { path_searcher_ = path_searcher; }

    void tryToAddHistoryPos(const eigen_utils::Vec3f& pos);

    // Insert and delete operations (Mannually)
    std::pair<int, bool> insertVertex(const GraphVertexType& vertex);
    void deleteVertex(const int& vertex_id);
    bool addTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id,
                       const GraphEdgeType& edge);
    bool deleteTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id);

    bool findPath(const int& start_vertex_id, const int& end_vertex_id,
                  std::vector<int>& waypoint_ids, eigen_utils::Vec_Vec3f& shortest_path,
                  double& cost);

    int getVertexId(const eigen_utils::Vec3f& point);
    eigen_utils::Vec3f getLastKeyPos() const;
    void generateRoadGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                  const double& vertices_scale, const double& edges_scale,
                                  const eigen_utils::Vec4d& vertices_rgba,
                                  const eigen_utils::Vec4d& edges_rgba) const;

  private:
    // params
    HistoryPosGraphParams params_;

    // data
    bool last_keypos_;
    float cumulated_distance_;
    eigen_utils::Vec3f last_pos_;

    eigen_utils::Vec_Vec3f history_poses_;
    std::unordered_set<int> history_pos_ids_;

    PathSearcher::SharedPtr path_searcher_;
    IkdTreePlanGraph<void, void>::SharedPtr graph_;
};

/**
 * @brief Construct a new History Pos Graph:: History Pos Graph object
 *
 */
inline HistoryPosGraph::HistoryPosGraph()
{
    last_keypos_ = false;
    cumulated_distance_ = 0.0f;
    graph_ = std::make_shared<IkdTreePlanGraph<void, void>>(true);
}

/**
 * @brief Try to add a historical position to the graph.
 *
 * This function attempts to add a new position to the historical position graph.
 * It checks if the new position should be a key position based on the accumulated distance
 * and the proximity to other points. If the position is a key position, it is added to the
 * history. The function also establishes connections with nearby points within a specified
 * distance.
 *
 * @param pos The position to be added.
 */
inline void HistoryPosGraph::tryToAddHistoryPos(const eigen_utils::Vec3f& pos)
{
    eigen_utils::Vec3d pos_d = pos.cast<double>();

    bool is_keypos = history_poses_.empty();

    // 0If the new position is approximately the same as the last position, do not add it to the
    // graph.
    if (!history_poses_.empty() && last_pos_.isApprox(pos, 1e-3))
        return;

    // 1. Remove the last non-key point if it exists
    if (!last_keypos_ && !history_poses_.empty())
    {
        if (history_pos_ids_.find(graph_->getVertexId(last_pos_)) == history_pos_ids_.end())
            graph_->deleteVertex(graph_->getVertexId(last_pos_), true);
    }

    // 2. Determine if the current position should be a key position
    if (!is_keypos)
    {
        cumulated_distance_ += (pos - last_pos_).norm();

        if (cumulated_distance_ > params_.min_insert_distance_)
        {
            std::vector<int> nbr_vec =
                graph_->getNearestVertex(pos, 1, params_.min_interval_distance_);

            if (nbr_vec.empty() || history_pos_ids_.find(nbr_vec[0]) == history_pos_ids_.end())
                is_keypos = true;
        }
    }

    // Update last position and key position flag
    last_pos_ = pos;
    last_keypos_ = is_keypos;

    // Add the current position as a vertex in the graph
    auto [v_id, success] = graph_->addVertex(GraphVertexType(pos, true), true);

    // Get the last key position vertex ID
    int last_key_vid = -1;
    if (!history_poses_.empty())
        last_key_vid = graph_->getVertexId(history_poses_.back());

    // If it's a key position, reset accumulated distance and update history
    if (is_keypos)
    {
        cumulated_distance_ = 0.0f;
        history_poses_.push_back(pos);
        history_pos_ids_.insert(v_id);
    }

    // 3. After successful insertion, establish connections with points within the
    // params_.max_link_distance_ range. For the last key position
    if (last_key_vid >= 0)
    {
        double cost;
        eigen_utils::Vec3d last_key_pos_d = graph_->getVertexPos(last_key_vid).cast<double>();

        auto res1 = path_searcher_->getSdfMap()->isStrictInflatedCollisionFreeStraight(
            pos_d, last_key_pos_d);
        if (res1.is_collision_free)
        {
            cost = (pos_d - last_key_pos_d).norm();
            graph_->addTwoWayEdge(v_id, last_key_vid, GraphEdgeType(cost, true));
        }
        else
        {
            eigen_utils::Vec_Vec3d shortest_path;
            PATH_SEARCH_RESULT res = path_searcher_->searchCoarsePathWithWSRoadMap(
                pos_d, last_key_pos_d, shortest_path, cost, false, 5.0);

            if (res == PATH_SEARCH_RESULT::SUCCESS)
                graph_->addTwoWayEdge(v_id, last_key_vid, GraphEdgeType(cost, true));
        }
    }

    // For the other neighbors of the current position
    eigen_utils::Vec_Vec3f neighbor_vertexs;
    std::vector<int> nbr_vec =
        graph_->getRadiusNeighborVertexsIDs(pos, params_.max_link_distance_, neighbor_vertexs);
    int pos_id = graph_->getVertexId(pos);

    for (const auto& nbr_id : nbr_vec)
    {
        // Skip if the neighbor is the current position or not part of the history
        if (nbr_id == last_key_vid || nbr_id == pos_id ||
            history_pos_ids_.find(nbr_id) == history_pos_ids_.end())
            continue;

        // Perform ray-cast between the current position and the neighbor
        double cost;
        eigen_utils::Vec3d nbr_pos_d = graph_->getVertexPos(nbr_id).cast<double>();

        auto res2 =
            path_searcher_->getSdfMap()->isStrictInflatedCollisionFreeStraight(pos_d, nbr_pos_d);
        if (res2.is_collision_free)
        {
            cost = (pos_d - nbr_pos_d).norm();
            graph_->addTwoWayEdge(v_id, nbr_id, GraphEdgeType(cost, true));
        }
    }
}

/**
 * @brief Inserts a vertex into the whole state road map.
 *
 * This function inserts a vertex into the whole state road map and returns a pair containing the
 * vertex ID and a boolean indicating whether the insertion was successful.
 *
 * @param vertex The vertex to be inserted, of type GraphVertexType.
 * @return std::pair<int, bool> A pair containing the vertex ID and a boolean indicating whether the
 * insertion was successful.
 */
inline std::pair<int, bool> HistoryPosGraph::insertVertex(const GraphVertexType& vertex)
{
    return graph_->addVertex(vertex, true);
}

/**
 * @brief Deletes a vertex from the topological road map.
 *
 * This function removes a vertex identified by its ID from the topological road map.
 * All the edges connected to the vertex are also removed.
 *
 * @param vertex_id The ID of the vertex to be deleted.
 */
inline void HistoryPosGraph::deleteVertex(const int& vertex_id)
{
    graph_->deleteVertex(vertex_id, true);
}

/**
 * @brief Adds a bidirectional edge between two vertices in the topological road map.
 *
 * This function adds a bidirectional (two-way) edge between the vertices identified by their IDs.
 * The edge has an associated cost.
 *
 * @param start_vertex_id The ID of the starting vertex.
 * @param end_vertex_id The ID of the ending vertex.
 * @param edge The edge to be added. It is of type GraphEdgeType.
 * @return true if the edge was successfully added, false otherwise.
 */
inline bool HistoryPosGraph::addTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id,
                                           const GraphEdgeType& edge)
{
    return graph_->addTwoWayEdge(start_vertex_id, end_vertex_id, edge);
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
inline bool HistoryPosGraph::deleteTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id)
{
    return graph_->deleteTwoWayEdge(start_vertex_id, end_vertex_id);
}

inline bool HistoryPosGraph::findPath(const int& start_vertex_id, const int& end_vertex_id,
                                      std::vector<int>& waypoint_ids,
                                      eigen_utils::Vec_Vec3f& shortest_path, double& cost)
{
    return graph_->findShortestPath(start_vertex_id, end_vertex_id, waypoint_ids, shortest_path,
                                    cost);
}

/**
 * @brief
 *
 * @param point
 * @return int
 */
inline int HistoryPosGraph::getVertexId(const eigen_utils::Vec3f& point)
{
    return graph_->getVertexId(point);
}

/**
 * @brief Get the last inserted position in the history.
 *
 * This function returns the last position that was inserted into the history.
 * If the history is empty, it throws a runtime error.
 *
 * @return eigen_utils::Vec3f The last inserted position.
 * @throws std::runtime_error If the history is empty.
 */
inline eigen_utils::Vec3f HistoryPosGraph::getLastKeyPos() const
{
    if (history_poses_.empty())
        throw std::runtime_error("history poses is empty.");

    return history_poses_.back();
}

inline void
HistoryPosGraph::generateRoadGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                          const double& vertices_scale, const double& edges_scale,
                                          const eigen_utils::Vec4d& vertices_rgba,
                                          const eigen_utils::Vec4d& edges_rgba) const
{
    const ros::Time stamp = ros::Time::now();
    visualization_msgs::Marker vertices = vis_utils::marker_utils::makePointMarker(
        "world", "HistoryPosGraph", 0,
        vis_utils::marker_utils::makeScale(vertices_scale, vertices_scale, vertices_scale),
        vis_utils::marker_utils::toRosColor(vertices_rgba), visualization_msgs::Marker::ADD, stamp);
    visualization_msgs::Marker edges = vis_utils::marker_utils::makeLineListMarker(
        "world", "HistoryPosGraph", 1, edges_scale, vis_utils::marker_utils::toRosColor(edges_rgba),
        visualization_msgs::Marker::ADD, stamp);

    auto snapshot = graph_->getGraphVisualizationSnapshot();

    // assignment
    int num = 0;
    std::unordered_map<int, eigen_utils::Vec3f> id_pos_map;
    id_pos_map.reserve(snapshot.vertices.size());
    vertices.points.reserve(snapshot.vertices.size());
    edges.points.reserve((snapshot.edges.size() + snapshot.temporary_edges.size()) * 2);

    for (const auto& vertex_entry : snapshot.vertices)
    {
        const auto& node = vertex_entry.pos;
        vis_utils::marker_utils::appendPoint(vertices, node);
        id_pos_map.emplace(vertex_entry.id, node);
        num++;
    }
    ROS_INFO("the graph vertices num is %i", num);

    int number = 0;
    for (const auto& edge_entry : snapshot.edges)
    {
        if (!edge_entry.active)
            continue;

        int from_id = edge_entry.from_id;
        int to_id = edge_entry.to_id;

        if (id_pos_map.find(from_id) == id_pos_map.end() ||
            id_pos_map.find(to_id) == id_pos_map.end())
            continue;

        auto s_vp = id_pos_map.at(from_id);
        auto t_vp = id_pos_map.at(to_id);

        vis_utils::marker_utils::appendLine(edges, s_vp, t_vp);
        number++;
    }
    ROS_INFO("the graph edges num is %i ", number);

    vis_utils::marker_utils::assignMarkers(graph_markers, {vertices, edges});
}
} // namespace map_process
#endif
