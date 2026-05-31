/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-09-19 12:05:48
 * @LastEditTime: 2026-02-04 20:32:52
 * @Description:
 */

#ifndef WHOLE_STATE_ROAD_MAP_H
#define WHOLE_STATE_ROAD_MAP_H

#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "map_process/core/map_process_params.h"
#include "plan_env/sdf_map.h"
#include "process_utils/process_utils.h"
#include "utils/AABB.hpp"
#include "utils/plan_graph_ikdtree.h"

namespace map_process
{

struct WSVertexExtraData
{
    enum class VertexSource : int8_t
    {
        FIXED,
        SAMPLE
    };

    enum class VertexState : int8_t
    {
        UNKNOWN,
        FREE
    };

    enum class VertexType : int8_t
    {
        COMMON,
        CURRENT_POSITION,
        VIEWPOINT,
        REGION_CENTER
    };

    VertexSource vertex_source_;    // 0: fixed vertex, 1: sample vertex
    VertexState cur_vertex_state_;  // 0: unknown vertex, 1: free vertex
    VertexState last_vertex_state_; // 0: unknown vertex, 1: free vertex
    VertexType vertex_type_; // 0: common, 1: current position, 2: viewpoint, 3: region center

    WSVertexExtraData()
        : vertex_source_(VertexSource::SAMPLE), cur_vertex_state_(VertexState::UNKNOWN),
          last_vertex_state_(VertexState::UNKNOWN), vertex_type_(VertexType::COMMON)
    {
    }
    WSVertexExtraData(const VertexSource vertex_source, const VertexState vertex_state,
                      const VertexType vertex_type = VertexType::COMMON)
        : vertex_source_(vertex_source), cur_vertex_state_(vertex_state),
          last_vertex_state_(vertex_state), vertex_type_(vertex_type)
    {
    }
};

struct WSEdgeExtraData
{
    process_utils::CubeBox bbox_;
    bool overlap_frontiers_;
    bool cross_unknown_;

    WSEdgeExtraData(const process_utils::CubeBox& bbox, const bool overlap_frontiers,
                    const bool cross_unknown)
        : bbox_(bbox), overlap_frontiers_(overlap_frontiers), cross_unknown_(cross_unknown)
    {
    }
};

class WSRoadMap
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<WSRoadMap>;

    using VertexExtraDataType = WSVertexExtraData;
    using EdgeExtraDataType = WSEdgeExtraData;
    using VertexType = IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::VertexType;
    using EdgeType = IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType>::EdgeType;

    WSRoadMap() = default;
    WSRoadMap(ros::NodeHandle& nh, fast_planner::SDFMap::Ptr sdf_map, const int planner_dim = 3);
    ~WSRoadMap() = default;

    // Setters
    void setPlannerDim(const int planner_dim);
    void setInitialPosition(const eigen_utils::Vec3f& initial_position);
    void setCurrentPosition(const eigen_utils::Vec3f& current_position);
    void setSamplePointsBound(const eigen_utils::Vec3d& max_bound,
                              const eigen_utils::Vec3d& min_bound);

    void insertValidRegionBoxes(const std::vector<process_utils::CubeBox>& boxes);

    // Getters
    int getVertexId(const eigen_utils::Vec3f& point);
    bool isVertexExisted(const eigen_utils::Vec3f& point) const;
    VertexType& getVertex(const int vertex_id);
    int getVertexNum() const;
    std::unordered_map<int, EdgeType>& getLinkedEdges(const int vertex_id);

    bool isManuallyUpdatedVertex(const VertexType& vertex) const;

    //  Update operations
    void addFixedPointsToGraph(const eigen_utils::Vec_Vec3f& fixed_points,
                               const eigen_utils::Vec3f resolution);
    void updateExistingVerticesStateAndEdgesConnection(
        const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max,
        const std::vector<process_utils::CubeBox>& frontiers_boxes);
    void growingTopoGraphBySamplePointsWithinBox(const eigen_utils::Vec3f& box_min,
                                                 const eigen_utils::Vec3f& box_max);
    void checkAndEraseIsolatedVerticesWithinBox(const eigen_utils::Vec3f& box_min,
                                                const eigen_utils::Vec3f& box_max);
    void inactivateUnknownStateVerticesInBox(const eigen_utils::Vec3f& box_min,
                                             const eigen_utils::Vec3f& box_max);
    void activateUnknownStateVerticesInBox(const eigen_utils::Vec3f& box_min,
                                           const eigen_utils::Vec3f& box_max);

    // Insert and delete operations (Mannually)
    std::pair<int, bool> insertVertex(const VertexType& vertex);
    void deleteVertex(const int& vertex_id);
    bool addTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id, const EdgeType& edge);
    bool deleteTwoWayEdge(const int& start_vertex_id, const int& end_vertex_id);

    //  Search operations
    bool nearestSearch(const eigen_utils::Vec3f& point, eigen_utils::Vec3f& nearest_point,
                       int& nearest_point_id);
    bool nearestSearch(const eigen_utils::Vec3f& point, const int k,
                       eigen_utils::Vec_Vec3f& neighbor_vertexs,
                       std::vector<int>& neighbor_vertex_ids,
                       const double max_distance = std::numeric_limits<double>::max());
    bool nearRangeSearch(const eigen_utils::Vec3f& point, const double range,
                         eigen_utils::Vec_Vec3f& neighbor_vertexs,
                         std::vector<int>& neighbor_vertex_ids);
    bool BoxNeighborSearch(const eigen_utils::Vec3f& box_min, const eigen_utils::Vec3f& box_max,
                           eigen_utils::Vec_Vec3f& neighbor_vertexs,
                           std::vector<int>& neighbor_vertex_ids);

    bool findNearestValidPointInGraph(
        const eigen_utils::Vec3f& point, const bool optimistic, eigen_utils::Vec3f& nearest_point,
        int& nearest_point_id, eigen_utils::Vec_Vec3f& path, double& path_cost,
        const double neighbors_search_range = 8.0,
        const std::optional<process_utils::CubeBox>& bounding_box = std::nullopt) const;
    bool findNearestValidPointsInGraph(
        const eigen_utils::Vec3f& point, const bool optimistic, eigen_utils::Vec_Vec3f& near_points,
        std::vector<int>& near_point_ids, std::vector<eigen_utils::Vec_Vec3f>& paths,
        std::vector<double>& path_costs, const double neighbors_search_range = 8.0,
        const std::optional<process_utils::CubeBox>& bounding_box = std::nullopt) const;
    bool findShortestPath(const eigen_utils::Vec3f& start_point,
                          const eigen_utils::Vec3f& end_point, const bool optimistic,
                          eigen_utils::Vec_Vec3f& shortest_path, double& cost) const;
    bool findShortestPathWithinBox(const eigen_utils::Vec3f& start_point,
                                   const eigen_utils::Vec3f& end_point,
                                   const process_utils::CubeBox& search_box, const bool optimistic,
                                   eigen_utils::Vec_Vec3f& shortest_path, double& cost) const;

    // Visualization
    template <typename PointListType>
    visualization_msgs::MarkerArray generatePointsMarkers(const PointListType& points,
                                                          const eigen_utils::Vec4d& color,
                                                          const eigen_utils::Vec3d& scale) const;
    void generateRoadGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                  const double& vertices_scale, const double& edges_scale,
                                  const eigen_utils::Vec4d& vertices_rgba,
                                  const eigen_utils::Vec4d& edges_rgba) const;

  private:
    void loadParamsFromROS(const ros::NodeHandle& nh);

    void initTopoGraph(const eigen_utils::Vec3f& origin_position);
    void updateGraphVerticesStrictFreeConnections(const std::vector<int>& vertex_ids);
    void updateGraphVerticesOptimisticConnections(const std::vector<int>& vertex_ids);

    bool findCollisionFreeStraightPath(const eigen_utils::Vec3f& point,
                                       const eigen_utils::Vec_Vec3f& nearest_points,
                                       const bool optimistic, eigen_utils::Vec3f& key_point,
                                       eigen_utils::Vec_Vec3f& path, double& path_cost) const;
    bool findCollisionFreeStraightPaths(const eigen_utils::Vec3f& point,
                                        const eigen_utils::Vec_Vec3f& nearest_points,
                                        const bool optimistic, eigen_utils::Vec_Vec3f& key_points,
                                        std::vector<eigen_utils::Vec_Vec3f>& paths,
                                        std::vector<double>& path_costs, const int max_n = 1) const;
    bool findDetailedPathToNearestGoal(const eigen_utils::Vec3f& point,
                                       const eigen_utils::Vec_Vec3f& nearest_points,
                                       const bool optimistic, const double search_range,
                                       eigen_utils::Vec3f& key_point, eigen_utils::Vec_Vec3f& path,
                                       double& path_cost) const;

    /** @brief Search KD-tree for nearby vertices and filter by state + bounding box.
     *  Returns the filtered candidate vertices as eigen vectors. */
    eigen_utils::Vec_Vec3f
    searchNearestFilteredVertices(const eigen_utils::Vec3f& point, const bool optimistic,
                                  int max_candidates, const double neighbors_search_range,
                                  const std::optional<process_utils::CubeBox>& bounding_box) const;

    bool findShortestPathImpl(
        const eigen_utils::Vec3f& start_point, const eigen_utils::Vec3f& end_point,
        const bool optimistic, eigen_utils::Vec_Vec3f& shortest_path, double& cost,
        const std::optional<process_utils::CubeBox>& bounding_box = std::nullopt) const;

    std::pair<double, eigen_utils::Vec_Vec3f>
    findShortestPathFromStartToAnyGoal(const eigen_utils::Vec3f start,
                                       const eigen_utils::Vec3fSet<3>& goals, const bool optimistic,
                                       const process_utils::CubeBox& search_box) const;

    WSRoadMapParams wsgp_;

    // Core data structures
    IkdTreePlanGraph<VertexExtraDataType, EdgeExtraDataType> graph_;
    fast_planner::SDFMap::Ptr sdf_map_;
    KD_TREE<PointType>::Ptr active_points_ikdtree_;
    KD_TREE<PointType>::Ptr fixed_points_ikdtree_;

    // Valid region tracking
    std::unique_ptr<aabb::Tree> valid_region_aabb_tree_;
    std::unordered_map<int, bool> valid_region_explored_state_;
    int valid_region_count_ = 0;

    // Bounds & state
    eigen_utils::Vec3d samplepoint_max_bound_, samplepoint_min_bound_;
    eigen_utils::Vec3f current_position_;
    int planner_dim_ = 3;
    bool is_graph_initialized_ = false;
    bool is_active_points_ikdtree_built_ = false;
    bool is_fixed_points_ikdtree_built_ = false;

    // Debug — only available in debug builds
    void test();
};
} // namespace map_process

#endif // WHOLE_STATE_ROAD_MAP_H
