/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-31 11:45:15
 * @Description:
 */

#ifndef _IKDTREE_PLAN_GRAPH_
#define _IKDTREE_PLAN_GRAPH_

#include <iostream>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"
#include "utils/ikd_Tree.h"

namespace map_process
{
using PointType = ikdTree_PointType;
using PointVector = KD_TREE<PointType>::PointVector;

template <typename VertexExtraDataType = void> struct GraphVertex
{
    eigen_utils::Vec3f pos_;
    bool active_;
    VertexExtraDataType extra_data_;

    GraphVertex(const eigen_utils::Vec3f& pos, const bool active,
                const VertexExtraDataType& extra_data)
        : pos_(pos), active_(active), extra_data_(extra_data)
    {
    }
};

template <> struct GraphVertex<void>
{
    eigen_utils::Vec3f pos_;
    bool active_;

    GraphVertex(const eigen_utils::Vec3f& pos, const bool active) : pos_(pos), active_(active) {}
};

template <typename EdgeExtraDataType = void> struct GraphEdge
{
    double cost_;
    bool active_;
    EdgeExtraDataType extra_data_;

    GraphEdge(const double cost, const bool active, const EdgeExtraDataType& extra_data)
        : cost_(cost), active_(active), extra_data_(extra_data)
    {
    }
};

template <> struct GraphEdge<void>
{
    double cost_;
    bool active_;

    GraphEdge(const double cost, const bool active) : cost_(cost), active_(active) {}
};

template <typename VertexExtraDataType = void, typename EdgeExtraDataType = void>
class IkdTreePlanGraph
{
  public:
    using SharedPtr = std::shared_ptr<IkdTreePlanGraph>;
    using UniquePtr = std::unique_ptr<IkdTreePlanGraph>;

    using VertexType = GraphVertex<VertexExtraDataType>;
    using EdgeType = GraphEdge<EdgeExtraDataType>;

    IkdTreePlanGraph(const bool use_ikdtree = true);

    void resetGraph();
    void resetTemporaryEdges();
    void enableTemporaryEdges();
    void disableTemporaryEdges();

    // add and remove vertices;
    std::pair<int, bool> addVertex(const VertexType& point, bool update_kdtree = true);
    void addVertices(const eigen_utils::Vec_Vec3f& point_vec, bool update_kdtree = true);
    void addVertices(const std::vector<VertexType>& point_vec, bool update_kdtree = true);
    void deleteVertex(const int point_id, bool update_kdtree = true);
    void deleteVertices(const std::vector<int>& point_ids, bool update_kdtree = true);

    // add and remove edges;
    bool addOneWayEdge(const int from_id, const int to_id, const EdgeType& edge);
    bool addTwoWayEdge(const int from_id, const int to_id, const EdgeType& edge);
    bool addTemporaryOneWayEdge(const int from_id, const int to_id, const EdgeType& edge);
    bool addTemporaryTwoWayEdge(const int from_id, const int to_id, const EdgeType& edge);

    bool deleteOneWayEdge(const int from_id, const int to_id);
    bool deleteTwoWayEdge(const int from_id, const int to_id);
    bool deleteTemporaryOneWayEdge(const int from_id, const int to_id);
    bool deleteTemporaryTwoWayEdge(const int from_id, const int to_id);

    // extraction
    std::vector<int>
    getNearestVertex(const eigen_utils::Vec3f& point, const int k = 1,
                     const double max_distance = std::numeric_limits<double>::max());
    std::vector<int> getRadiusNeighborVertexsIDs(const eigen_utils::Vec3f& point,
                                                 const double& range,
                                                 eigen_utils::Vec_Vec3f& neighbor_vertexs);
    std::vector<int> getBoxNeighborVertexsIDs(const eigen_utils::Vec3f& box_min,
                                              const eigen_utils::Vec3f& box_max,
                                              eigen_utils::Vec_Vec3f& neighbor_vertexs);

    bool findShortestPath(const int from_id, const int to_id, std::vector<int>& waypoint_ids,
                          eigen_utils::Vec_Vec3f& shortest_path, double& cost) const;
    bool findShortestPathWithinBox(const int from_id, const int to_id,
                                   std::vector<int>& waypoint_ids,
                                   eigen_utils::Vec_Vec3f& shortest_path, double& cost,
                                   const eigen_utils::Vec3f& box_min,
                                   const eigen_utils::Vec3f& box_max) const;

    // query
    bool isVertexExisted(const int vertex_id) const;
    bool isVertexExisted(const VertexType& vertex) const;
    bool isVertexExisted(const eigen_utils::Vec3f& point) const;

    int getVertexId(const VertexType& vertex) const;
    int getVertexId(const eigen_utils::Vec3f& point) const;
    eigen_utils::Vec3f getVertexPos(const int vertex_id) const;

    struct GraphVisualizationSnapshot
    {
        using VertexExtraDataView = std::conditional_t<std::is_void_v<VertexExtraDataType>,
                                                       std::monostate, VertexExtraDataType>;

        struct VertexEntry
        {
            int id;
            eigen_utils::Vec3f pos;
            bool active;
            VertexExtraDataView extra_data;
        };

        struct EdgeEntry
        {
            int from_id;
            int to_id;
            bool active;
        };

        std::vector<VertexEntry> vertices;
        std::vector<EdgeEntry> edges;
        std::vector<EdgeEntry> temporary_edges;
    };
    GraphVisualizationSnapshot getGraphVisualizationSnapshot() const;

    std::pair<std::unordered_map<int, VertexType>,
              std::unordered_map<int, std::unordered_map<int, EdgeType>>>
    getGraphCopy() const;
    VertexType& getVertex(const int vertex_id);
    const VertexType& getVertex(const int vertex_id) const;
    int getLinkedEdgesNum(const int point_id) const;
    const std::unordered_map<int, VertexType>& getAllVertices() const;
    std::unordered_map<int, VertexType> getAllVerticesCopy() const;
    const std::unordered_map<int, std::unordered_map<int, EdgeType>>& getAllEdges() const;
    std::unordered_map<int, std::unordered_map<int, EdgeType>> getAllEdgesCopy() const;
    const std::unordered_map<int, std::unordered_map<int, EdgeType>>& getAllTemporaryEdges() const;
    std::unordered_map<int, EdgeType>& getLinkedEdges(const int point_id);
    const std::unordered_map<int, EdgeType>& getLinkedEdges(const int point_id) const;
    std::unordered_map<int, EdgeType>& getTemporaryLinkedEdges(const int point_id);
    const std::unordered_map<int, EdgeType>& getTemporaryLinkedEdges(const int point_id) const;

    int getVertexNum() const { return vertices_.size(); };
    int getEdgeNum() const { return edges_.size(); };

    bool DijkstraSearch(const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids,
                        double& cost) const;
    bool AStarSearch(const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids,
                     double& cost) const;
    bool AStarSearchWithinBox(const int start_v_id, const int end_v_id,
                              std::vector<int>& waypoint_ids, double& cost,
                              const eigen_utils::Vec3f& box_min,
                              const eigen_utils::Vec3f& box_max) const;

  private:
    // Internal search functions (lock-free, caller must hold the lock)
    bool DijkstraSearchInternal(const int start_v_id, const int end_v_id,
                                std::vector<int>& waypoint_ids, double& cost) const;
    bool AStarSearchInternal(const int start_v_id, const int end_v_id,
                             std::vector<int>& waypoint_ids, double& cost) const;
    bool AStarSearchWithinBoxInternal(const int start_v_id, const int end_v_id,
                                      std::vector<int>& waypoint_ids, double& cost,
                                      const eigen_utils::Vec3f& box_min,
                                      const eigen_utils::Vec3f& box_max) const;

    // kdtree: for fast search of neighbors
    bool use_ikdtree_;
    bool is_tree_built_;
    KD_TREE<PointType>::Ptr graph_ikdtree_;

    // graph data
    int vertex_index_;
    eigen_utils::Vec3fMap<int, 3> points_ids_;                         // store the ids of points.
    std::unordered_map<int, VertexType> vertices_;                     // vertex positions
    std::unordered_map<int, std::unordered_map<int, EdgeType>> edges_; // edges between vertices

    bool enable_temporary_edges_;
    std::unordered_map<int, std::unordered_map<int, EdgeType>> temporary_edges_;

    // mutex
    mutable std::shared_mutex graph_mutex_;
};

/**
 * @brief Construct a new Ikd Tree Plan Graph< Vertex Extra Data Type>:: Ikd Tree Plan Graph object
 *
 * @tparam VertexExtraDataType
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::IkdTreePlanGraph(
    const bool use_ikdtree)
{
    use_ikdtree_ = use_ikdtree;
    is_tree_built_ = false;
    enable_temporary_edges_ = false;

    if (use_ikdtree_)
        graph_ikdtree_ = std::make_shared<KD_TREE<PointType>>(0.3, 0.6, 0.2);
    else
        graph_ikdtree_ = nullptr;

    vertex_index_ = 0;
}

/**
 * @brief Reset the graph.
 *
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::resetGraph()
{
    std::unique_lock<std::shared_mutex> lock(graph_mutex_);

    is_tree_built_ = false;
    enable_temporary_edges_ = false;

    if (use_ikdtree_)
        graph_ikdtree_ = std::make_shared<KD_TREE<PointType>>(0.3, 0.6, 0.2);
    else
        graph_ikdtree_ = nullptr;

    vertex_index_ = 0;
    points_ids_.clear();
    vertices_.clear();
    edges_.clear();
    temporary_edges_.clear();
}

/**
 * @brief Reset temporary edges.
 *
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::resetTemporaryEdges()
{
    std::unique_lock<std::shared_mutex> lock(graph_mutex_);

    temporary_edges_.clear();
}

/**
 * @brief Enable temporary edges.
 *
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::enableTemporaryEdges()
{
    enable_temporary_edges_ = true;
}

/**
 * @brief Disable temporary edges.
 *
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::disableTemporaryEdges()
{
    enable_temporary_edges_ = false;
}

/**
 * @brief Adds a vertex to the graph.
 *
 * This function adds a vertex to the graph. If the vertex already exists, it returns the existing
 * vertex ID. Otherwise, it inserts the new vertex, updates the k-d tree if required, and returns
 * the new vertex ID.
 *
 * @tparam VertexExtraDataType The type of extra data associated with the vertex.
 * @param point The vertex to be added.
 * @param update_kdtree A boolean flag indicating whether to update the k-d tree.
 * @return std::pair<int, bool> A pair containing the vertex ID and a boolean indicating whether the
 * vertex was newly added.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
std::pair<int, bool>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addVertex(const VertexType& point,
                                                                    bool update_kdtree)
{
    if (!use_ikdtree_ && update_kdtree)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    // Check if the point is already in the graph, if so, return the id of the point
    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);
    auto it = points_ids_.find(point.pos_);
    if (it != points_ids_.end())
        return std::make_pair(it->second, false);

    // Add vertex
    vertices_.insert(std::make_pair(vertex_index_, point));
    points_ids_.insert(std::make_pair(point.pos_, vertex_index_));
    edges_.insert(std::make_pair(vertex_index_, std::unordered_map<int, EdgeType>()));
    vertex_index_++;

    // Update kdtree
    PointVector add_pc;
    add_pc.emplace_back(point.pos_.x(), point.pos_.y(), point.pos_.z());
    if (update_kdtree)
    {
        if (is_tree_built_)
        {
            graph_ikdtree_->Add_Points(add_pc, false);
        }
        else
        {
            graph_ikdtree_->Build(add_pc);
            is_tree_built_ = true;
        }
    }

    return std::make_pair(vertex_index_ - 1, true);
}

/**
 * @brief Add multiple vertices to the graph.
 *
 * This function adds multiple vertices to the graph. If a vertex already exists, it will be
 * skipped. The kdtree will be updated if required.
 *
 * @param point_vec The vector of points to be added.
 * @param update_kdtree If true, the kdtree will be updated after the vertices are added.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addVertices(
    const eigen_utils::Vec_Vec3f& point_vec, bool update_kdtree)
{
    if (!use_ikdtree_ && update_kdtree)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    PointVector add_pc;

    // Add vertices
    for (int i = 0; i < int(point_vec.size()); i++)
    {
        eigen_utils::Vec3f point = point_vec[i];
        if (points_ids_.count(point) != 0)
            continue;

        vertices_.insert(std::make_pair(vertex_index_, VertexType(point, true)));
        points_ids_.insert(std::make_pair(point, vertex_index_));
        edges_.insert(std::make_pair(vertex_index_, std::unordered_map<int, EdgeType>()));
        vertex_index_++;

        add_pc.emplace_back(point.x(), point.y(), point.z());
    }

    // Update kdtree
    if (update_kdtree)
    {
        if (is_tree_built_)
        {
            graph_ikdtree_->Add_Points(add_pc, false);
        }
        else
        {
            graph_ikdtree_->Build(add_pc);
            is_tree_built_ = true;
        }
    }
}

/**
 * @brief Add multiple vertices to the graph.
 *
 * This function adds multiple vertices to the graph. If a vertex already exists, it will be
 * skipped. The kdtree will be updated if required.
 *
 * @param point_vec The vector of points to be added.
 * @param update_kdtree If true, the kdtree will be updated after the vertices are added.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addVertices(
    const std::vector<VertexType>& point_vec, bool update_kdtree)
{
    if (!use_ikdtree_ && update_kdtree)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    PointVector add_pc;

    // Add vertices
    for (int i = 0; i < int(point_vec.size()); i++)
    {
        const VertexType& point = point_vec[i];
        if (points_ids_.find(point.pos_) != points_ids_.end())
            continue;

        vertices_.insert(std::make_pair(vertex_index_, point));
        points_ids_.insert(std::make_pair(point.pos_, vertex_index_));
        edges_.insert(std::make_pair(vertex_index_, std::unordered_map<int, EdgeType>()));
        vertex_index_++;

        add_pc.emplace_back(point.pos_.x(), point.pos_.y(), point.pos_.z());
    }

    // Update kdtree
    if (update_kdtree)
    {
        if (is_tree_built_)
        {
            graph_ikdtree_->Add_Points(add_pc, false);
        }
        else
        {
            graph_ikdtree_->Build(add_pc);
            is_tree_built_ = true;
        }
    }
}

/**
 * @brief Delete a vertex from the graph.
 *
 * This function deletes a vertex from the graph. If the vertex does not exist, it will be skipped.
 * The kdtree will be updated if required. All edges connected to the vertex will be deleted.
 *
 * @param point_id The id of the vertex to be deleted.
 * @param update_kdtree If true, the kdtree will be updated after the vertex is deleted.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteVertex(const int point_id,
                                                                       bool update_kdtree)
{
    if (!use_ikdtree_ && update_kdtree)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(point_id) == vertices_.end())
        return;

    const eigen_utils::Vec3f point = vertices_.at(point_id).pos_;

    // Update kdtree
    if (update_kdtree)
    {
        PointVector delete_point_cloud;
        delete_point_cloud.emplace_back(point.x(), point.y(), point.z());
        graph_ikdtree_->Delete_Points(delete_point_cloud);
    }

    // Delete edges
    std::vector<int> delete_link_ids;
    std::transform(edges_.at(point_id).begin(), edges_.at(point_id).end(),
                   std::back_inserter(delete_link_ids),
                   [](const auto& pair) { return pair.first; });
    for (const auto& id : delete_link_ids)
        edges_.at(id).erase(point_id);

    edges_.erase(point_id);

    // Delete vertex
    vertices_.erase(point_id);
    points_ids_.erase(point);
}

/**
 * @brief Delete multiple vertices from the graph.
 *
 * This function deletes multiple vertices from the graph. If a vertex does not exist, it will be
 * skipped. The kdtree will be updated if required. All edges connected to the vertices will be
 * deleted.
 *
 * @param point_ids The ids of the vertices to be deleted.
 * @param update_kdtree If true, the kdtree will be updated after the vertices are deleted.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline void IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteVertices(
    const std::vector<int>& point_ids, bool update_kdtree)
{
    if (!use_ikdtree_ && update_kdtree)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    PointVector delete_point_cloud;

    // Delete vertices
    for (const int point_id : point_ids)
    {
        if (vertices_.find(point_id) == vertices_.end())
            continue;

        const eigen_utils::Vec3f point = vertices_.at(point_id).pos_;

        // Update kdtree
        if (update_kdtree)
        {
            delete_point_cloud.emplace_back(point.x(), point.y(), point.z());
        }

        // Delete edges
        std::vector<int> delete_link_ids;
        std::transform(edges_.at(point_id).begin(), edges_.at(point_id).end(),
                       std::back_inserter(delete_link_ids),
                       [](const auto& pair) { return pair.first; });
        for (const auto& id : delete_link_ids)
            edges_.at(id).erase(point_id);

        edges_.erase(point_id);

        // Delete vertex
        vertices_.erase(point_id);
        points_ids_.erase(point);
    }

    // Update kdtree
    if (update_kdtree)
    {
        graph_ikdtree_->Delete_Points(delete_point_cloud);
    }
}

/**
 * @brief Add a one-way edge from vertex with id from_id to vertex with id to_id.
 *
 * @param from_id The id of the vertex from which the edge starts.
 * @param to_id The id of the vertex to which the edge ends.
 * @param edge The edge to be added.
 * @return true If the edge is added successfully.
 * @return false If the edge is not added successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addOneWayEdge(
    const int from_id, const int to_id, const EdgeType& edge)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = edges_[from_id];
    edges_a_id.insert(std::make_pair(to_id, edge));

    return true;
}

/**
 * @brief Add a two-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 * @param edge The edge to be added.
 * @return true If the edge is added successfully.
 * @return false If the edge is not added successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addTwoWayEdge(
    const int from_id, const int to_id, const EdgeType& edge)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = edges_[from_id];
    edges_a_id.insert(std::make_pair(to_id, edge));

    auto& edges_b_id = edges_[to_id];
    edges_b_id.insert(std::make_pair(from_id, edge));

    return true;
}

/**
 * @brief Add a temporary one-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 * @param edge The edge to be added.
 * @return true If the edge is added successfully.
 * @return false If the edge is not added successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addTemporaryOneWayEdge(
    const int from_id, const int to_id, const EdgeType& edge)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = temporary_edges_[from_id];
    edges_a_id.insert(std::make_pair(to_id, edge));

    return true;
}

/**
 * @brief Add a temporary two-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 * @param edge The edge to be added.
 * @return true If the edge is added successfully.
 * @return false If the edge is not added successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::addTemporaryTwoWayEdge(
    const int from_id, const int to_id, const EdgeType& edge)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = temporary_edges_[from_id];
    edges_a_id.insert(std::make_pair(to_id, edge));

    auto& edges_b_id = temporary_edges_[to_id];
    edges_b_id.insert(std::make_pair(from_id, edge));

    return true;
}

/**
 * @brief Delete a one-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 *
 * @return true If the edge is deleted successfully.
 * @return false If the edge is not deleted successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteOneWayEdge(const int from_id,
                                                                           const int to_id)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = edges_[from_id];
    edges_a_id.erase(to_id);

    return true;
}

/**
 * @brief Delete a two-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 *
 * @return true If the edge is deleted successfully.
 * @return false If the edge is not deleted successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteTwoWayEdge(const int from_id,
                                                                           const int to_id)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = edges_[from_id];
    edges_a_id.erase(to_id);

    auto& edges_b_id = edges_[to_id];
    edges_b_id.erase(from_id);

    return true;
}

/**
 * @brief Delete a temporary one-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 *
 * @return true If the edge is deleted successfully.
 * @return false If the edge is not deleted successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteTemporaryOneWayEdge(
    const int from_id, const int to_id)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = temporary_edges_[from_id];
    edges_a_id.erase(to_id);

    return true;
}

/**
 * @brief Delete a temporary two-way edge between two vertices.
 *
 * @param from_id The id of the first vertex.
 * @param to_id The id of the second vertex.
 *
 * @return true If the edge is deleted successfully.
 * @return false If the edge is not deleted successfully.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::deleteTemporaryTwoWayEdge(
    const int from_id, const int to_id)
{
    if (from_id == to_id)
    {
        std::cerr << "Function name: " << __func__ << "from_id is equal to to_id" << endl;
        return false;
    }

    std::unique_lock<std::shared_mutex> lock_vertex(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Function name: " << __func__ << ", Invalid vertex index" << endl;
        return false;
    }

    auto& edges_a_id = temporary_edges_[from_id];
    edges_a_id.erase(to_id);

    auto& edges_b_id = temporary_edges_[to_id];
    edges_b_id.erase(from_id);

    return true;
}

/**
 * @brief Check if a vertex exists in the graph.
 *
 * @param vertex_id The id of the vertex.
 * @return true If the vertex exists.
 * @return false If the vertex does not exist.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::isVertexExisted(const int vertex_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return vertices_.find(vertex_id) != vertices_.end();
}

/**
 * @brief Check if a vertex exists in the graph.
 *
 * @param vertex The vertex to be checked.
 * @return true If the vertex exists.
 * @return false If the vertex does not exist.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::isVertexExisted(
    const VertexType& point) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return points_ids_.find(point.pos_) != points_ids_.end();
}

/**
 * @brief Check if a point exists in the graph.
 *
 * @param point The point to be checked.
 * @return true If the point exists.
 * @return false If the point does not exist.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::isVertexExisted(
    const eigen_utils::Vec3f& point) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return points_ids_.find(point) != points_ids_.end();
}

/**
 * @brief Get the vertex with the given id.
 *
 * @param vertex_id The id of the vertex.
 * @return eigen_utils::Vec3f The point of the vertex.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline eigen_utils::Vec3f
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getVertexPos(const int vertex_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = vertices_.find(vertex_id);
    if (it == vertices_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second.pos_;
}

/**
 * @brief Get a lightweight snapshot for visualization/export use.
 *
 * This avoids copying full edge extra data such as bounding boxes when only
 * geometry and active states are needed by marker generation.
 *
 * @return GraphVisualizationSnapshot Lightweight graph snapshot.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::GraphVisualizationSnapshot
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getGraphVisualizationSnapshot() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    GraphVisualizationSnapshot snapshot;
    snapshot.vertices.reserve(vertices_.size());
    snapshot.edges.reserve(vertices_.size() * 4);
    snapshot.temporary_edges.reserve(temporary_edges_.size() * 4);

    for (const auto& [vertex_id, vertex] : vertices_)
    {
        typename GraphVisualizationSnapshot::VertexEntry entry{
            vertex_id, vertex.pos_, vertex.active_, {}};
        if constexpr (!std::is_void_v<VertexExtraDataType>)
            entry.extra_data = vertex.extra_data_;

        snapshot.vertices.push_back(std::move(entry));
    }

    for (const auto& [from_id, connected_edges] : edges_)
    {
        for (const auto& [to_id, edge_data] : connected_edges)
        {
            snapshot.edges.push_back(
                typename GraphVisualizationSnapshot::EdgeEntry{from_id, to_id, edge_data.active_});
        }
    }

    for (const auto& [from_id, connected_edges] : temporary_edges_)
    {
        for (const auto& [to_id, edge_data] : connected_edges)
        {
            snapshot.temporary_edges.push_back(
                typename GraphVisualizationSnapshot::EdgeEntry{from_id, to_id, edge_data.active_});
        }
    }

    return snapshot;
}

/**
 * @brief Get a copy of the graph.
 *
 * @return std::pair<std::unordered_map<int, VertexType>, std::unordered_map<int,
 * std::unordered_map<int, EdgeType>>>
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::pair<
    std::unordered_map<
        int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType>,
    std::unordered_map<
        int, std::unordered_map<
                 int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>>>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getGraphCopy() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return {vertices_, edges_};
}

/**
 * @brief Get the vertex with the given id.
 *
 * @param vertex_id The id of the vertex.
 * @return VertexType The vertex.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getVertex(const int vertex_id)
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = vertices_.find(vertex_id);
    if (it == vertices_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get the vertex with the given id.
 *
 * @param vertex_id The id of the vertex.
 * @return VertexType The vertex.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getVertex(const int vertex_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = vertices_.find(vertex_id);
    if (it == vertices_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get all vertices in the graph.
 *
 * @return const eigen_utils::Vec_Vec3f The vector of all vertices.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getAllVertices() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return vertices_;
}

/**
 * @brief Get all vertices in the graph (Copy).
 *
 * @return std::unordered_map<int, VertexType> The vector of all vertices.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getAllVerticesCopy() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return vertices_;
}

/**
 * @brief Get all edges in the graph.
 *
 * @return const std::unordered_map<int, std::unordered_map<int, double>> The map of all edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const std::unordered_map<
    int, std::unordered_map<
             int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getAllEdges() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return edges_;
}

/**
 * @brief Get all edges in the graph (Copy).
 *
 * @return std::unordered_map<int, std::unordered_map<int, EdgeType>> The map of all edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::unordered_map<
    int, std::unordered_map<
             int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getAllEdgesCopy() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return edges_;
}

/**
 * @brief Get all temporary edges in the graph.
 *
 * @return const std::unordered_map<int, std::unordered_map<int, double>> The map of all temporary
 * edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const std::unordered_map<
    int, std::unordered_map<
             int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getAllTemporaryEdges() const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    return temporary_edges_;
}

/**
 * @brief Get the edges linked to a vertex.
 *
 * @param point_id The id of the vertex.
 * @return std::unordered_map<int, EdgeType> The map of linked edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getLinkedEdges(const int point_id)
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = edges_.find(point_id);
    if (it == edges_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get the edges linked to a vertex.
 *
 * @param point_id The id of the vertex.
 * @return std::unordered_map<int, EdgeType> The map of linked edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getLinkedEdges(const int point_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = edges_.find(point_id);
    if (it == edges_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get the temporary edges linked to a vertex.
 *
 * @param point_id The id of the vertex.
 * @return std::unordered_map<int, EdgeType> The map of temporary edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getTemporaryLinkedEdges(
    const int point_id)
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = temporary_edges_.find(point_id);
    if (it == temporary_edges_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get the temporary edges linked to a vertex.
 *
 * @param point_id The id of the vertex.
 * @return std::unordered_map<int, EdgeType> The map of temporary edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline const std::unordered_map<
    int, typename IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType>&
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getTemporaryLinkedEdges(
    const int point_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = temporary_edges_.find(point_id);
    if (it == temporary_edges_.end())
        throw std::runtime_error("Invalid vertex index");

    return it->second;
}

/**
 * @brief Get the id of a point in the graph.
 *
 * @param vertex The point to be checked.
 * @return int The id of the point. If the point is not in the graph, return -1.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline int IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getVertexId(
    const VertexType& vertex) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = points_ids_.find(vertex.pos_);
    if (it != points_ids_.end())
        return it->second;
    else
        return -1;
}

/**
 * @brief Get the id of a point in the graph.
 *
 * @param point The point to be checked.
 * @return int The id of the point. If the point is not in the graph, return -1.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline int IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getVertexId(
    const eigen_utils::Vec3f& point) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = points_ids_.find(point);
    if (it != points_ids_.end())
        return it->second;
    else
        return -1;
}

/**
 * @brief Get the number of edges linked to a vertex.
 *
 * @param point_id The id of the vertex.
 * @return int The number of linked edges.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline int IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getLinkedEdgesNum(
    const int point_id) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    auto it = edges_.find(point_id);
    if (it != edges_.end())
        return it->second.size();
    else
        throw std::runtime_error("Invalid vertex index");
}

/**
 * @brief Get the nearest vertices to a given point
 *
 * This function searches the graph for the k nearest vertices to a given 3D point and returns their
 * IDs.
 *
 * @tparam VertexExtraDataType Type of extra data associated with vertices
 * @tparam EdgeExtraDataType Type of extra data associated with edges
 * @param point The input 3D point
 * @param k The number of nearest neighbors to find
 * @param max_distance The maximum distance to search for nearest neighbors
 * @return std::vector<int> A vector containing the IDs of the nearest vertices
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::vector<int> IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getNearestVertex(
    const eigen_utils::Vec3f& point, const int k, const double max_distance)
{
    if (!use_ikdtree_)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    if (graph_ikdtree_->size() == 0)
    {
        std::cerr << "Graph is empty" << std::endl;
        return std::vector<int>();
    }

    PointVector neighbors_vector;
    std::vector<float> distance_vector;
    PointType pt(point.x(), point.y(), point.z());

    graph_ikdtree_->Nearest_Search(pt, k, neighbors_vector, distance_vector, max_distance);

    std::vector<int> vertexs_ids;
    vertexs_ids.reserve(neighbors_vector.size());

    for (const auto& n_pt : neighbors_vector)
    {
        eigen_utils::Vec3f nearest_point(n_pt.x, n_pt.y, n_pt.z);
        auto it = points_ids_.find(nearest_point);
        if (it != points_ids_.end())
        {
            vertexs_ids.push_back(it->second);
        }
        else
        {
            std::cerr << "IkdTreePlanGraph getNearestVertex Fail!" << std::endl;
        }
    }

    return vertexs_ids;
}

/**
 * @brief Get the ids of the vertices within a certain range of a point.
 *
 * This function searches the graph for vertices within a specified range of a given 3D point and
 * returns their IDs. It also populates a vector with the points of the neighboring vertices.
 *
 * @tparam VertexExtraDataType Type of extra data associated with vertices
 * @tparam EdgeExtraDataType Type of extra data associated with edges
 * @param point The point to be checked
 * @param range The range within which to search for vertices
 * @param neighbor_vertexs The vector of points within the range
 * @return std::vector<int> The ids of the vertices within the range
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::vector<int>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getRadiusNeighborVertexsIDs(
    const eigen_utils::Vec3f& point, const double& range, eigen_utils::Vec_Vec3f& neighbor_vertexs)
{
    if (!use_ikdtree_)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    std::vector<int> vertexs_ids;
    neighbor_vertexs.clear();

    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    if (graph_ikdtree_->size() == 0)
    {
        std::cerr << "Graph is empty" << std::endl;
        return vertexs_ids;
    }

    PointVector neighbors_vector;
    graph_ikdtree_->Radius_Search(PointType(point.x(), point.y(), point.z()), range,
                                  neighbors_vector);

    vertexs_ids.reserve(neighbors_vector.size());
    neighbor_vertexs.reserve(neighbors_vector.size());

    for (const auto& item : neighbors_vector)
    {
        eigen_utils::Vec3f neigbor_point(item.x, item.y, item.z);
        auto it = points_ids_.find(neigbor_point);
        if (it != points_ids_.end())
        {
            int id = it->second;
            vertexs_ids.push_back(id);
            neighbor_vertexs.push_back(neigbor_point);
        }
        else
        {
            std::cerr << "IkdTreePlanGraph getRadiusNeighborVertexsIDs Fail!" << std::endl;
        }
    }
    return vertexs_ids;
}

/**
 * @brief Get the ids of the vertices within a bounding box.
 *
 * This function searches the graph for vertices within a specified bounding box and returns their
 * IDs. It also populates a vector with the points of the neighboring vertices.
 *
 * @tparam VertexExtraDataType Type of extra data associated with vertices
 * @tparam EdgeExtraDataType Type of extra data associated with edges
 * @param box_min The minimum corner of the bounding box
 * @param box_max The maximum corner of the bounding box
 * @param neighbor_vertexs The vector of points within the bounding box
 * @return std::vector<int> The ids of the vertices within the bounding box
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline std::vector<int>
IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::getBoxNeighborVertexsIDs(
    const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max,
    eigen_utils::Vec_Vec3f& neighbor_vertexs)
{
    if (!use_ikdtree_)
    {
        throw std::runtime_error(
            "IkdTreePlanGraph::addVertex: kdtree is not enabled, cannot update kdtree.");
    }

    neighbor_vertexs.clear();
    std::vector<int> vertexs_ids;

    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    if (graph_ikdtree_->size() == 0)
    {
        std::cerr << "Graph is empty" << std::endl;
        return vertexs_ids;
    }

    BoxPointType search_box;
    search_box.vertex_min[0] = box_min.x();
    search_box.vertex_min[1] = box_min.y();
    search_box.vertex_min[2] = box_min.z();
    search_box.vertex_max[0] = box_max.x();
    search_box.vertex_max[1] = box_max.y();
    search_box.vertex_max[2] = box_max.z();

    PointVector neighbors_vector;
    graph_ikdtree_->Box_Search(search_box, neighbors_vector);

    vertexs_ids.reserve(neighbors_vector.size());
    neighbor_vertexs.reserve(neighbors_vector.size());

    for (const auto& item : neighbors_vector)
    {
        eigen_utils::Vec3f neighbor_point(item.x, item.y, item.z);
        auto it = points_ids_.find(neighbor_point);
        if (it != points_ids_.end())
        {
            vertexs_ids.emplace_back(it->second);
            neighbor_vertexs.emplace_back(item.x, item.y, item.z);
        }
        else
        {
            std::cerr << "IkdTreePlanGraph getBoxNeighborVertexsIDs Fail!" << std::endl;
        }
    }

    return vertexs_ids;
}

/**
 * @brief Find the shortest path between two points.
 *
 * @param from_id The id of the start point.
 * @param to_id The id of the end point.
 * @param waypoint_ids The ids of the waypoints in the shortest path.
 * @param shortest_path The points of the waypoints in the shortest path.
 * @param cost The cost of the shortest path.
 * @return true The shortest path is found.
 * @return false The shortest path is not found.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::findShortestPath(
    const int from_id, const int to_id, std::vector<int>& waypoint_ids,
    eigen_utils::Vec_Vec3f& shortest_path, double& cost) const
{
    waypoint_ids.clear();
    shortest_path.clear();

    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Start or end point is not in the graph" << std::endl;
        cost = std::numeric_limits<double>::max();
        return false;
    }

    if (from_id == to_id)
    {
        waypoint_ids.push_back(from_id);
        shortest_path.push_back(vertices_.at(from_id).pos_);
        cost = 0.0;
        return true;
    }

    if (!AStarSearchInternal(from_id, to_id, waypoint_ids, cost))
    {
        cost = std::numeric_limits<double>::max();
        return false;
    }

    for (const auto& id : waypoint_ids)
        shortest_path.push_back(vertices_.at(id).pos_);

    return true;
}

template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::findShortestPathWithinBox(
    const int from_id, const int to_id, std::vector<int>& waypoint_ids,
    eigen_utils::Vec_Vec3f& shortest_path, double& cost, const eigen_utils::Vec3f& box_min,
    const eigen_utils::Vec3f& box_max) const
{
    waypoint_ids.clear();
    shortest_path.clear();

    std::shared_lock<std::shared_mutex> lock(graph_mutex_);

    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end())
    {
        std::cerr << "Start or end point is not in the graph" << std::endl;
        cost = std::numeric_limits<double>::max();
        return false;
    }

    if (from_id == to_id)
    {
        waypoint_ids.push_back(from_id);
        shortest_path.push_back(vertices_.at(from_id).pos_);
        cost = 0.0;
        return true;
    }

    if (!AStarSearchWithinBoxInternal(from_id, to_id, waypoint_ids, cost, box_min, box_max))
    {
        cost = std::numeric_limits<double>::max();
        return false;
    }

    for (const auto& id : waypoint_ids)
        shortest_path.push_back(vertices_.at(id).pos_);

    return true;
}

/**
 * @brief Performs Dijkstra's algorithm to find the shortest path between two vertices.
 *
 * This function uses Dijkstra's algorithm to find the shortest path from the start vertex to the
 * end vertex. It returns the path as a vector of vertex IDs and the total cost of the path. If no
 * path is found, it returns false.
 *
 * @tparam VertexExtraDataType The type of extra data stored in each vertex.
 * @tparam EdgeExtraDataType The type of extra data stored in each edge.
 * @param start_v_id The ID of the start vertex.
 * @param end_v_id The ID of the end vertex.
 * @param waypoint_ids A vector to store the IDs of the vertices in the shortest path.
 * @param cost A reference to a double to store the total cost of the path.
 * @return true If a path is found.
 * @return false If no path is found.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::DijkstraSearch(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    return DijkstraSearchInternal(start_v_id, end_v_id, waypoint_ids, cost);
}

template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::DijkstraSearchInternal(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost) const
{
    if (vertices_.find(start_v_id) == vertices_.end() ||
        vertices_.find(end_v_id) == vertices_.end())
    {
        std::cerr << "Invalid start or end vertex index" << std::endl;
        return false;
    }

    // Check if the start and end points are the same
    if (start_v_id == end_v_id)
    {
        waypoint_ids.clear();
        waypoint_ids.push_back(start_v_id);
        cost = 0.0;
        return true;
    }

    // Initialize the priority queue
    typedef std::pair<double, int>
        iPair; // Vertices are represented by their index in the graph.vertices list
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>>
        open_set; // Priority queue of vertices

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> backpointers;
    std::unordered_set<int> closed;

    dist[start_v_id] = 0.0;
    open_set.push(std::make_pair(0.0, start_v_id));

    auto processEdges = [&](const auto& edges, const int from_id) {
        if (edges.find(from_id) == edges.end())
            return;
        for (const auto& edge : edges.at(from_id))
        {
            const int to_id = edge.first;
            const double cost = edge.second.cost_;

            if (closed.count(to_id) || !vertices_.at(to_id).active_ || !edge.second.active_)
                continue;

            // If there is a shorter path to to_id through from_id, update the distance of to_id
            double new_cost = dist[from_id] + cost;
            auto it_dist = dist.find(to_id);
            if (it_dist == dist.end() || it_dist->second > new_cost)
            {
                dist[to_id] = new_cost;
                backpointers[to_id] = from_id;
                open_set.push(std::make_pair(new_cost, to_id));
            }
        }
    };

    int u;
    bool found = false;
    while (!open_set.empty()) // Loop until priority queue is empty
    {
        u = open_set.top().second;
        double d = open_set.top().first;
        open_set.pop(); // Pop the minimum distance vertex

        if (closed.count(u))
            continue;
        closed.insert(u);

        // If the end vertex is reached, break
        if (u == end_v_id)
        {
            found = true;
            cost = d;
            break;
        }

        processEdges(edges_, u);

        if (enable_temporary_edges_)
            processEdges(temporary_edges_, u);
    }

    if (!found)
    {
        cost = std::numeric_limits<double>::max();
        return false;
    }

    // Backtrack to find path
    waypoint_ids.clear();
    int current = end_v_id;
    while (current != start_v_id)
    {
        waypoint_ids.push_back(current);
        current = backpointers[current];
    }
    waypoint_ids.push_back(start_v_id);
    std::reverse(waypoint_ids.begin(), waypoint_ids.end());

    return true;
}

/**
 * @brief Perform A* search to find the shortest path from the start vertex to the end vertex.
 *
 * This function uses the A* algorithm to find the shortest path between two vertices in the graph.
 * It returns the path as a sequence of vertex IDs and the total cost of the path.
 *
 * @tparam VertexExtraDataType The type of extra data associated with each vertex.
 * @tparam EdgeExtraDataType The type of extra data associated with each edge.
 * @param start_v_id The ID of the start vertex.
 * @param end_v_id The ID of the end vertex.
 * @param waypoint_ids A vector to store the IDs of the vertices in the path.
 * @param cost A reference to a double to store the total cost of the path.
 * @return true If a path is found.
 * @return false If no path is found.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::AStarSearch(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    return AStarSearchInternal(start_v_id, end_v_id, waypoint_ids, cost);
}

template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::AStarSearchInternal(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost) const
{
    if (vertices_.find(start_v_id) == vertices_.end() ||
        vertices_.find(end_v_id) == vertices_.end())
    {
        std::cerr << "Invalid start or end vertex index" << std::endl;
        return false;
    }

    // Check if the start and end points are the same
    if (start_v_id == end_v_id)
    {
        waypoint_ids.clear();
        waypoint_ids.push_back(start_v_id);
        cost = 0.0;
        return true;
    }

    const VertexType& start_vertex = vertices_.at(start_v_id);
    const VertexType& end_vertex = vertices_.at(end_v_id);

    typedef std::pair<double, int>
        iPair; // Vertices are represented by their index in the graph.vertices list
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>>
        pq; // Priority queue of vertices

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> backpointers;
    std::unordered_set<int> closed;

    // Add the start vertex
    dist[start_v_id] = 0;
    double start_estimation = (start_vertex.pos_ - end_vertex.pos_).norm();
    pq.push(std::make_pair(start_estimation, start_v_id));

    auto processEdges = [&](const auto& edges, const int from_id) {
        if (edges.find(from_id) == edges.end())
            return;
        for (const auto& edge : edges.at(from_id))
        {
            const int to_id = edge.first;
            const double cost = edge.second.cost_;
            const VertexType& to_vertex = vertices_.at(to_id);

            if (closed.count(to_id) || !to_vertex.active_ || !edge.second.active_)
                continue;

            // If there is a shorter path to to_id through from_id, update the distance of to_id
            double new_cost = dist[from_id] + cost;
            auto it_dist = dist.find(to_id);
            if (it_dist == dist.end() || it_dist->second > new_cost)
            {
                dist[to_id] = new_cost;
                double estimation = new_cost + (to_vertex.pos_ - end_vertex.pos_).norm();
                backpointers[to_id] = from_id;
                pq.push(std::make_pair(estimation, to_id));
            }
        }
    };

    int u;
    bool found = false;
    while (!pq.empty()) // Loop until priority queue is empty
    {
        u = pq.top().second;
        pq.pop(); // Pop the minimum distance vertex

        if (closed.count(u))
            continue;
        closed.insert(u);

        // If the end vertex is reached, break
        if (u == end_v_id)
        {
            found = true;
            cost = dist[end_v_id];
            break;
        }

        processEdges(edges_, u);

        if (enable_temporary_edges_)
            processEdges(temporary_edges_, u);
    }

    if (!found)
    {
        cost = std::numeric_limits<double>::max();
        return false;
    }

    // Backtrack to find path
    waypoint_ids.clear();
    int current = end_v_id;
    while (current != start_v_id)
    {
        waypoint_ids.push_back(current);
        current = backpointers[current];
    }
    waypoint_ids.push_back(start_v_id);
    std::reverse(waypoint_ids.begin(), waypoint_ids.end());

    return true;
}

/**
 * @brief Perform A* search within a given bounding box to find the shortest path from the start
 * vertex to the end vertex.
 *
 * This function uses the A* algorithm to find the shortest path between two vertices within a
 * specified bounding box. It returns the path as a sequence of vertex IDs and the total cost of the
 * path.
 *
 * @tparam VertexExtraDataType The type of extra data associated with each vertex.
 * @tparam EdgeExtraDataType The type of extra data associated with each edge.
 * @param start_v_id The ID of the start vertex.
 * @param end_v_id The ID of the end vertex.
 * @param waypoint_ids A vector to store the IDs of the vertices in the path.
 * @param cost A reference to a double to store the total cost of the path.
 * @param box_min The minimum coordinates of the bounding box.
 * @param box_max The maximum coordinates of the bounding box.
 * @return true If a path is found.
 * @return false If no path is found.
 */
template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::AStarSearchWithinBox(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost,
    const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max) const
{
    std::shared_lock<std::shared_mutex> lock(graph_mutex_);
    return AStarSearchWithinBoxInternal(start_v_id, end_v_id, waypoint_ids, cost, box_min, box_max);
}

template <typename VertexExtraDataType, typename EdgeExtraDataType>
inline bool IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::AStarSearchWithinBoxInternal(
    const int start_v_id, const int end_v_id, std::vector<int>& waypoint_ids, double& cost,
    const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max) const
{
    const VertexType& start_vertex = vertices_.at(start_v_id);
    const VertexType& end_vertex = vertices_.at(end_v_id);

    // Check if the start and end points are in the box
    if (!process_utils::ProcessUtils::isInBox(start_vertex.pos_, box_min, box_max) ||
        !process_utils::ProcessUtils::isInBox(end_vertex.pos_, box_min, box_max))
    {
        std::cerr << "Start or end point is not in the box" << std::endl;
        cost = std::numeric_limits<double>::max();
        return false;
    }

    // Check if the start and end points are the same
    if (start_v_id == end_v_id)
    {
        waypoint_ids.clear();
        waypoint_ids.push_back(start_v_id);
        cost = 0.0;
        return true;
    }

    typedef std::pair<double, int>
        iPair; // Vertices are represented by their index in the graph.vertices list
    std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>>
        pq; // Priority queue of vertices

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> backpointers;
    std::unordered_set<int> closed;

    // Add the start vertex
    dist[start_v_id] = 0;
    double start_estimation = (start_vertex.pos_ - end_vertex.pos_).norm();
    pq.push(std::make_pair(start_estimation, start_v_id));

    auto processEdges = [&](const auto& edges, const int from_id) {
        if (edges.find(from_id) == edges.end())
            return;
        for (const auto& edge : edges.at(from_id))
        {
            const int to_id = edge.first;
            const double cost = edge.second.cost_;
            const VertexType& to_vertex = vertices_.at(to_id);

            if (closed.count(to_id) || !to_vertex.active_ || !edge.second.active_ ||
                !process_utils::ProcessUtils::isInBox(to_vertex.pos_, box_min, box_max))
                continue;

            // If there is a shorter path to to_id through from_id, update the distance of to_id
            double new_cost = dist[from_id] + cost;
            auto it_dist = dist.find(to_id);
            if (it_dist == dist.end() || it_dist->second > new_cost)
            {
                dist[to_id] = new_cost;
                double estimation = new_cost + (to_vertex.pos_ - end_vertex.pos_).norm();
                backpointers[to_id] = from_id;
                pq.push(std::make_pair(estimation, to_id));
            }
        }
    };

    int u;
    bool found = false;
    while (!pq.empty()) // Loop until priority queue is empty
    {
        u = pq.top().second;
        pq.pop(); // Pop the minimum distance vertex

        if (closed.count(u))
            continue;
        closed.insert(u);

        // If the end vertex is reached, break
        if (u == end_v_id)
        {
            found = true;
            cost = dist[end_v_id];
            break;
        }

        processEdges(edges_, u);

        if (enable_temporary_edges_)
            processEdges(temporary_edges_, u);
    }

    if (!found)
    {
        cost = std::numeric_limits<double>::max();
        return false;
    }

    // Backtrack to find path
    waypoint_ids.clear();
    int current = end_v_id;
    while (current != start_v_id)
    {
        waypoint_ids.push_back(current);
        current = backpointers[current];
    }
    waypoint_ids.push_back(start_v_id);
    std::reverse(waypoint_ids.begin(), waypoint_ids.end());

    return true;
}
} // namespace map_process

#endif // _IKDTREE_PLAN_GRAPH_
