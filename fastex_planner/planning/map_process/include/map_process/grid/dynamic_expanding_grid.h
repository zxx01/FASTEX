/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-30 14:20:57
 * @Description:
 */

#ifndef _DYNAMIC_EXPANDING_GRID_
#define _DYNAMIC_EXPANDING_GRID_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "map_process/core/map_process_params.h"
#include "map_process/frontier/frontier_manager.h"
#include "map_process/grid/single_level_grid.h"
#include "map_process/roadmap/history_pos_graph.h"
#include "map_process/searcher/path_searcher.h"
#include "plan_env/sdf_map.h"
#include "process_utils/process_utils.h"
#include "utils/ikd_Tree.h"
#include "utils/plan_graph_ikdtree.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_planner
{
class EDTEnvironment;
} // namespace fast_planner

namespace map_process
{
struct RelevantGridAttributes
{
    GRID_EXPLORE_STATE state_;
    eigen_utils::Vec3i index_;
    eigen_utils::Vec3d centroid_;

    RelevantGridAttributes(const GRID_EXPLORE_STATE& state, const eigen_utils::Vec3i& index,
                           const eigen_utils::Vec3d& centroid)
        : state_(state), index_(index), centroid_(centroid)
    {
    }
};

struct DynamicExpandingGridData
{
    // basic data
    eigen_utils::Vec3d min_now_ = eigen_utils::Vec3d::Zero();
    eigen_utils::Vec3d max_now_ = eigen_utils::Vec3d::Zero();
    eigen_utils::Vec3d current_position_ = eigen_utils::Vec3d::Zero();
    std::unique_ptr<SingleLevelGrid> grid_;

    openvdb::BoolGrid::Ptr active_grid_;
    std::vector<openvdb::Vec3d> reserved_changed_cells_;

    eigen_utils::Vec3iMap<std::vector<pcl::PointXYZI>> label_clouds_;
    eigen_utils::Vec3iMap<float> label_clouds_uncovered_ratio_;
    KD_TREE<pcl::PointXYZ>::Ptr key_pos_kdtree_;

    // relevant grid data
    eigen_utils::Vec3iSet current_relevant_grid_; // store the indices of current relevant grids

    eigen_utils::Vec3dMap<GRID_EXPLORE_STATE, 3>
        vertices_explore_state_; // store the explore state of each grid vertices
    eigen_utils::Vec3dMap<eigen_utils::Vec3i, 3>
        grid_centroids_indices_; // store the grid index of each grid centroid
    eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>
        grid_indices_centroids_; // store the centroids of each grid
    std::vector<RelevantGridAttributes>
        relevant_graph_vertices_; // store the attributes of each relevant grid vertices (used to
                                  // compute cost)
    eigen_utils::Vec3fMap<eigen_utils::Vec3d, 3>
        grid_centroids_match_; // store the double-precision match of each graph centroid

    map_process::IkdTreePlanGraph<void, void>::SharedPtr relevant_graph_;

    // for road map
    std::vector<process_utils::CubeBox> new_extend_grid_boxes_;
    KD_TREE<pcl::PointXYZ>::Ptr exploring_unknown_zone_kdtree_;

    // Paths
    eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3> global_cost_matrix_map_;
    eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<eigen_utils::Vec_Vec3d, 3>, 3> global_paths_;
    eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<eigen_utils::Vec_Vec3d, 3>, 3> local_paths_;
};

class DynamicExpandingGrid
{
  public:
    using RelevantGraphType = map_process::IkdTreePlanGraph<void, void>;

  private:
    using ZoneToAllClusterTopVps =
        std::vector<std::pair<eigen_utils::Vec3d, std::optional<eigen_utils::Vec_Vec3d>>>;
    using ZoneToNearestClusterTopVp =
        std::vector<std::pair<eigen_utils::Vec3d, std::optional<eigen_utils::Vec3d>>>;
    using CentroidPair = std::pair<eigen_utils::Vec3d, eigen_utils::Vec3d>;
    using RelevantEdgeCostMap = std::unordered_map<int, std::unordered_map<int, double>>;

    fast_planner::SDFMap::Ptr sdf_map_;
    map_process::PathSearcher::SharedPtr path_searcher_;
    map_process::WSRoadMap::SharedPtr whole_state_road_map_;
    map_process::FrontierManager::SharedPtr frontier_manager_;
    map_process::HistoryPosGraph::SharedPtr history_pos_graph_;

    // Debug
    ros::Publisher visible_cloud_pub_, label_cloud_pub_, active_grid_pub_,
        first_grid_centroids_pub_, first_grid_neighbor_centroids_pub_, points_group_pub_,
        all_virtual_cloud_pub_, all_vis_virtual_cloud_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cumulative_vis_virtual_cloud_,
        cumulative_virtual_cloud_all_;

    /**
     * @brief Load DynamicExpandingGrid parameters from ROS.
     * @param nh ROS node handle used to fetch parameters.
     */
    void loadParamsFromROS(const ros::NodeHandle& nh);

    /**
     * @brief Query the current active frontier-cluster centroids.
     * @param cluster_centroids Output map from cluster id to centroid position.
     */
    void collectFrontierClusterCentroids(
        std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const;
    /**
     * @brief Convert changed-cell addresses to world coordinates and append them to the buffer.
     * @param changed_cells Changed-cell addresses reported by the map.
     * @param changed_cells_world In-out buffer storing world-space changed cells.
     */
    void appendChangedCells(const std::vector<int>& changed_cells,
                            std::vector<openvdb::Vec3d>& changed_cells_world) const;
    /**
     * @brief Cache the world-space boxes of newly extended grids for downstream updates.
     * @param extended_grid_indices Grid indices that were newly activated by this frame.
     */
    void recordNewExtendGridBoxes(const eigen_utils::Vec3iSet& extended_grid_indices);
    /**
     * @brief Push this frame's updates into the underlying single-level grid.
     * @param changed_cells_world In-out buffer of changed cells in world coordinates.
     * @param cluster_centroids Frontier-cluster centroids associated with the current frame.
     * @param extend_grid_coords Newly extended grid coordinates in OpenVDB index space.
     * @param update_min Minimum corner of the update box.
     * @param update_max Maximum corner of the update box.
     */
    void syncSingleLevelGridData(std::vector<openvdb::Vec3d>& changed_cells_world,
                                 std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
                                 const std::vector<openvdb::math::Coord>& extend_grid_coords,
                                 const eigen_utils::Vec3d& update_min,
                                 const eigen_utils::Vec3d& update_max);
    /**
     * @brief Refresh centroid-to-grid and centroid-to-state lookup tables for relevant grids.
     * @param grid_ids Relevant grid indices to export into cached metadata maps.
     */
    void populateRelevantGridMetadata(const eigen_utils::Vec3iSet& grid_ids);
    /**
     * @brief Update exploration-related per-grid data for all grids touched in this frame.
     * @param update_indices Grid indices scheduled for exploration-info refresh.
     */
    void updateGridExplorationInfoBatch(const eigen_utils::Vec_Vec3i& update_indices);
    /**
     * @brief Build the removed/new centroid sets used to synchronize the relevant graph.
     * @param removed_grid_indices Relevant grids removed from the current frame.
     * @param new_grid_indices Relevant grids newly added in the current frame.
     * @param unchanged_grid_indices Relevant grids preserved across frames.
     * @param last_grid_centroids Centroid snapshot from the previous relevant-grid state.
     * @param update_min Minimum corner of the update box.
     * @param update_max Maximum corner of the update box.
     * @param removed_centroids Output centroid map to erase from the relevant graph.
     * @param new_centroids Output centroid map to insert into the relevant graph.
     */
    void collectChangedRelevantGridCentroids(
        const eigen_utils::Vec3iSet& removed_grid_indices,
        const eigen_utils::Vec3iSet& new_grid_indices,
        const eigen_utils::Vec3iSet& unchanged_grid_indices,
        const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& last_grid_centroids,
        const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max,
        eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& removed_centroids,
        eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& new_centroids) const;
    /**
     * @brief Rebuild the cached relevant-graph vertex attributes from current centroid metadata.
     */
    void rebuildRelevantGraphVertexCache();
    /**
     * @brief Collect the world-space boxes of updated grids for roadmap refresh.
     * @param update_grid_indices Grid indices touched in the current frame.
     * @param refresh_boxes Output refresh boxes covering those grids.
     */
    void collectRefreshBoxes(const eigen_utils::Vec_Vec3i& update_grid_indices,
                             std::vector<process_utils::CubeBox>& refresh_boxes) const;
    /**
     * @brief Collect unknown-zone centers and their linked cluster top viewpoints.
     * @param update_grid_indices Grid indices touched in the current frame.
     * @param zone_to_all_cluster_top_vps Output mapping from zone center to all neighbor-cluster
     * top viewpoints.
     * @param zone_to_nearest_cluster_top_vp Output mapping from zone center to nearest
     * neighbor-cluster top viewpoint.
     */
    void collectUnknownZoneRoadmapTargets(
        const eigen_utils::Vec_Vec3i& update_grid_indices,
        ZoneToAllClusterTopVps& zone_to_all_cluster_top_vps,
        ZoneToNearestClusterTopVp& zone_to_nearest_cluster_top_vp) const;
    /**
     * @brief Remove outdated unknown-zone vertices from roadmap, history graph, and kd-tree.
     * @param refresh_boxes Boxes whose contents must be removed before reinsertion.
     */
    void removeStaleUnknownZoneVertices(const std::vector<process_utils::CubeBox>& refresh_boxes);
    /**
     * @brief Insert unknown-zone vertices and edges into the workspace roadmap.
     * @param zone_to_nearest_cluster_top_vp Mapping from zone center to nearest cluster top
     * viewpoint.
     * @param added_zone_points Output point set newly inserted into the unknown-zone kd-tree.
     */
    void insertUnknownZoneVerticesToRoadMap(
        const ZoneToNearestClusterTopVp& zone_to_nearest_cluster_top_vp,
        KD_TREE<pcl::PointXYZ>::PointVector& added_zone_points);
    /**
     * @brief Insert unknown-zone to cluster-top-vp connectivity into the history graph.
     * @param zone_to_all_cluster_top_vps Mapping from zone center to all linked cluster top
     * viewpoints.
     */
    void insertUnknownZoneVerticesToHistoryGraph(
        const ZoneToAllClusterTopVps& zone_to_all_cluster_top_vps);
    /**
     * @brief Update the kd-tree that indexes active unknown-zone centers.
     * @param added_zone_points Newly inserted unknown-zone points.
     */
    void updateUnknownZoneKdTree(KD_TREE<pcl::PointXYZ>::PointVector& added_zone_points);
    /**
     * @brief Collect centroid connected-components from the current relevant graph.
     * @param centroid_clusters Output connected components in centroid space.
     */
    void
    collectRelevantCentroidClusters(std::vector<eigen_utils::Vec_Vec3d>& centroid_clusters) const;
    /**
     * @brief Keep only centroids in the exploring state for each connected component.
     * @param centroid_clusters Input connected components.
     * @param exploring_centroid_clusters Output exploring-only centroid components.
     */
    void collectExploringCentroidClusters(
        const std::vector<eigen_utils::Vec_Vec3d>& centroid_clusters,
        std::vector<eigen_utils::Vec_Vec3d>& exploring_centroid_clusters) const;
    /**
     * @brief Build all centroid pairs between different exploring connected components.
     * @param exploring_centroid_clusters Exploring centroid components.
     * @param centroid_pairs Output centroid pairs to evaluate as temporary edges.
     */
    void buildInterClusterCentroidPairs(
        const std::vector<eigen_utils::Vec_Vec3d>& exploring_centroid_clusters,
        std::vector<CentroidPair>& centroid_pairs) const;
    /**
     * @brief Compute temporary-edge distances for candidate centroid pairs.
     * @param centroid_pairs Candidate centroid pairs.
     * @param global_dist_map Output pairwise distance map for valid temporary edges.
     */
    void computeTemporaryEdgeDistances(
        const std::vector<CentroidPair>& centroid_pairs,
        eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3>& global_dist_map) const;
    /**
     * @brief Insert temporary edges into the relevant graph from a centroid distance map.
     * @param global_dist_map Pairwise centroid distance map for valid temporary edges.
     */
    void insertTemporaryEdgesToRelevantGraph(
        const eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3>& global_dist_map);
    /**
     * @brief Collect grids whose enduring edges need to be refreshed in the current frame.
     * @param update_min Minimum corner of the update box.
     * @param update_max Maximum corner of the update box.
     * @param new_grid_indices Newly added relevant grids.
     * @param consider_grid_indices Output relevant grids whose enduring edges should be updated.
     */
    void
    collectRelevantGridsForEnduringEdgeUpdate(const eigen_utils::Vec3d& update_min,
                                              const eigen_utils::Vec3d& update_max,
                                              const eigen_utils::Vec3iSet& new_grid_indices,
                                              eigen_utils::Vec3iSet& consider_grid_indices) const;
    /**
     * @brief Compute the candidate enduring-edge cost between two relevant centroids.
     * @param from_centroid Source centroid.
     * @param to_centroid Target centroid.
     * @param box_search_margin Margin used to expand the local path-search box.
     * @return Positive cost if a valid edge exists; otherwise `-1.0`.
     */
    double computeEnduringEdgeCost(const eigen_utils::Vec3d& from_centroid,
                                   const eigen_utils::Vec3d& to_centroid,
                                   const eigen_utils::Vec3d& box_search_margin) const;
    /**
     * @brief Collect enduring-edge costs between centroids inside the same relevant grid.
     * @param grid_index Relevant grid whose internal centroid chain should be connected.
     * @param box_search_margin Margin used to expand the local path-search box.
     * @param edge_costs Output edge-cost cache keyed by relevant-graph vertex ids.
     */
    void collectIntraGridEnduringEdges(const eigen_utils::Vec3i& grid_index,
                                       const eigen_utils::Vec3d& box_search_margin,
                                       RelevantEdgeCostMap& edge_costs) const;
    /**
     * @brief Collect enduring-edge costs between centroids in neighboring relevant grids.
     * @param grid_index Source relevant grid.
     * @param current_grid_indices All current relevant grids.
     * @param box_search_margin Margin used to expand the local path-search box.
     * @param edge_costs Output edge-cost cache keyed by relevant-graph vertex ids.
     */
    void collectNeighborGridEnduringEdges(const eigen_utils::Vec3i& grid_index,
                                          const eigen_utils::Vec3iSet& current_grid_indices,
                                          const eigen_utils::Vec3d& box_search_margin,
                                          RelevantEdgeCostMap& edge_costs) const;
    /**
     * @brief Insert valid enduring edges into the relevant graph from a vertex-id cost cache.
     * @param edge_costs Candidate enduring-edge costs keyed by relevant-graph vertex ids.
     */
    void insertEnduringEdgesToRelevantGraph(const RelevantEdgeCostMap& edge_costs);

    // Extend Grid
    template <typename PointT>
    void processSensorCloud(const typename pcl::PointCloud<PointT>::Ptr& real_cloud,
                            const typename pcl::PointCloud<PointT>::Ptr& virtual_cloud,
                            eigen_utils::Vec3iMap<std::vector<PointT>>& label_clouds,
                            eigen_utils::Vec3iSet& real_cloud_located_grid_indices,
                            eigen_utils::Vec3iSet& extended_grid_indices,
                            std::vector<openvdb::math::Coord>& extend_grid_coords);

    template <typename PointT>
    void processLabelCloud(const eigen_utils::Vec_Vec3d& cumulated_pos,
                           const eigen_utils::Vec3iSet& real_cloud_located_grid_indices,
                           eigen_utils::Vec3iMap<std::vector<PointT>>& label_clouds,
                           eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio,
                           const double covered_range = 15.0, const double keypos_interval = 2.0);

    // Relevant Grid
    void updateRelevantGrid();

    void updateRelevantGridGraphVertices(
        const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& removed_vertices,
        const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& new_vertices);
    void updateRelevantGridGraphEnduringEdges(const eigen_utils::Vec3d& update_min,
                                              const eigen_utils::Vec3d& update_max,
                                              const eigen_utils::Vec3iSet& new_grid_indices);
    void eraseIsolatedUnexploredGrid();
    void updateRelevantGridGraphTemporaryEdges();

    // --- Relevant-graph update helpers ---

    /**
     * @brief Refresh per-grid exploration info and sync unknown-zone vertices
     *        to the workspace roadmap and history graph.
     * @param update_indices Grid indices touched by the current frame.
     */
    void updateExplorationInfoAndRoadmap(const eigen_utils::Vec_Vec3i& update_indices);

    /**
     * @brief BFS from a start centroid through the relevant graph to collect a
     *        connected component.
     * @param start_centroid  Starting centroid in world coordinates.
     * @param[out] component  Collected centroids in this connected component.
     * @param[in,out] voxel_visited  Shared visited set (by voxel index), updated
     *                               with all traversed nodes.
     * @return true if the component contains at least one EXPLORING vertex.
     */
    bool collectConnectedComponent(const eigen_utils::Vec3d& start_centroid,
                                   eigen_utils::Vec_Vec3d& component,
                                   eigen_utils::Vec3iSet& voxel_visited) const;

    // For Local SOP
    void findFirstGridCentroids(const std::vector<int>& grid_indices,
                                eigen_utils::Vec3dSet<3>& first_grid_centroids);
    void findFirstGridNeighborCentroids(const eigen_utils::Vec3dSet<3>& first_grid_centroids,
                                        eigen_utils::Vec3dSet<3>& first_grid_neighbor_centroids);
    void segmentGridIndices(const std::vector<int>& grid_indices,
                            std::vector<std::vector<int>>& grid_indices_groups,
                            std::vector<eigen_utils::Vec_Vec3d>& grid_centroids_groups);

    // Utils
    void analyzeGridChanges(const eigen_utils::Vec3iSet& current_grid,
                            const eigen_utils::Vec3iSet& last_grid,
                            eigen_utils::Vec3iSet& removed_grid, eigen_utils::Vec3iSet& added_grid,
                            eigen_utils::Vec3iSet& unchanged_grid);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<DynamicExpandingGrid>;
    using ConstSharedPtr = std::shared_ptr<const DynamicExpandingGrid>;
    using UniquePtr = std::unique_ptr<DynamicExpandingGrid>;
    using ConstUniquePtr = std::unique_ptr<const DynamicExpandingGrid>;

    using RelevantGraphVertexType = RelevantGraphType::VertexType;
    using RelevantGraphEdgeType = RelevantGraphType::EdgeType;

    DynamicExpandingGrid() = default;
    DynamicExpandingGrid(ros::NodeHandle& nh,
                         const std::shared_ptr<fast_planner::EDTEnvironment>& edt_env);
    ~DynamicExpandingGrid() = default;

    void setCurrentPosition(const eigen_utils::Vec3d& current_position);
    void setPathSearcher(const map_process::PathSearcher::SharedPtr& path_searcher);
    void setWholeStateRoadMap(const map_process::WSRoadMap::SharedPtr& whole_state_road_map);
    void setFrontierManager(const map_process::FrontierManager::SharedPtr& frontier_manager);
    void setHistoryPosGraph(const map_process::HistoryPosGraph::SharedPtr& history_pos_graph);

    // Update Grid Data
    void updateGridData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& real_cloud,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& virtual_cloud,
                        const eigen_utils::Vec_Vec3d& cumulated_pos,
                        const std::vector<int>& changed_cells, const eigen_utils::Vec3d& update_min,
                        const eigen_utils::Vec3d& update_max);

    // Get State Change Grid
    void getStateChangeGridBoxes(std::vector<process_utils::CubeBox>& new_boxes,
                                 std::vector<process_utils::CubeBox>& change_to_explored_boxes,
                                 std::vector<process_utils::CubeBox>& change_from_explored_boxes);

    // Update Relevant Grid Graph
    void updateRelevantGridGraph(const eigen_utils::Vec3d& update_min,
                                 const eigen_utils::Vec3d& update_max);
    void updateZoneAndClusterVerticesOnRoadMap(const eigen_utils::Vec_Vec3i& update_grid_indices);

    // Compute Cost
    void computeGlobalCoverageCostMatrix(const eigen_utils::Vec_Vec3d& position,
                                         const eigen_utils::Vec_Vec3d& velocity,
                                         const std::vector<double>& y1, Eigen::MatrixXd& matrix);

    void computeIncrementalGlobalCoverageCostMatrix(
        const eigen_utils::Vec_Vec3d& position, const eigen_utils::Vec_Vec3d& velocity,
        const std::vector<double>& y1, const std::vector<std::vector<int>>& generalized_indices,
        std::vector<int>& considered_vertices_indices, Eigen::MatrixXd& matrix);

    void computeLocalSOPCostMatrix(const eigen_utils::Vec_Vec3d& cur_pos,
                                   const eigen_utils::Vec_Vec3d& cur_vel,
                                   const std::vector<double>& cur_yaw,
                                   const eigen_utils::Vec_Vec3d& cluster_centroids,
                                   const std::vector<int>& grid_indices,
                                   Eigen::MatrixXd& local_matrix,
                                   std::vector<eigen_utils::Vec_Vec3d>& considered_vertices);

    // Compute Cost
    std::pair<double, eigen_utils::Vec_Vec3d> computeCostAndPathFromDroneToGrid(
        const eigen_utils::Vec3d& from_pos, const eigen_utils::Vec3d& to_pos,
        const eigen_utils::Vec3d& velocity, const double y1, const double inf_cost = 1e3);
    std::pair<double, eigen_utils::Vec_Vec3d>
    computeCostAndPathFromGridToGrid(const eigen_utils::Vec3d& from_pos,
                                     const eigen_utils::Vec3d& to_pos, const double inf_cost = 1e3);

    // Find Path
    // Getters
    const DynamicExpandingGridParams& getParams() const;
    std::vector<RelevantGridAttributes> getRelevantGridVertexSnapshot() const;
    const RelevantGraphType& getRelevantGraphReadonly() const;
    bool getCachedGlobalPath(const eigen_utils::Vec3d& from, const eigen_utils::Vec3d& to,
                             eigen_utils::Vec_Vec3d& path) const;
    bool getCachedGlobalCost(const eigen_utils::Vec3d& from, const eigen_utils::Vec3d& to,
                             double& cost) const;
    bool getCachedLocalPath(const eigen_utils::Vec3d& from, const eigen_utils::Vec3d& to,
                            eigen_utils::Vec_Vec3d& path) const;
    bool getRelevantGridAttributeByIndex(int idx, RelevantGridAttributes& attr) const;

    eigen_utils::Vec_Vec3i getGridIndicesInRange(const eigen_utils::Vec3d& min,
                                                 const eigen_utils::Vec3d& max) const;
    void getGridTour(const eigen_utils::Vec3d& current_position, const std::vector<int>& indices,
                     eigen_utils::Vec_Vec3d& key_points, eigen_utils::Vec_Vec3d& path) const;
    void getRelevantFrontierClusters(const std::vector<int>& grid_indices,
                                     std::vector<int>& cluster_indices) const;

    // Markers
    void getAllGridMarkers(const bool ignore_inactive_grid,
                           std::vector<eigen_utils::Vec_Vec3d>& pts1_vec,
                           std::vector<eigen_utils::Vec_Vec3d>& pts2_vec) const;
    void getRelevantGridMarkers(std::vector<eigen_utils::Vec_Vec3d>& pts1_vec,
                                std::vector<eigen_utils::Vec_Vec3d>& pts2_vec) const;
    void getRelevantGraphMarkers(visualization_msgs::MarkerArray& graph_markers,
                                 const double& vertices_scale, const double& edges_scale,
                                 const eigen_utils::Vec_Vec4d& vertices_rgba,
                                 const eigen_utils::Vec_Vec4d& edges_rgba) const;

    void publishActiveVoxels(ros::Publisher& marker_pub, openvdb::BoolGrid::ConstPtr grid,
                             const eigen_utils::Vec3d& voxel_size) const;

    template <typename PointsType>
    void publishPointsMarker(ros::Publisher& marker_pub, const PointsType& points,
                             const eigen_utils::Vec4f& color, const double scale,
                             const std::string& frame_id, const std::string& ns,
                             const int id) const;

  private:
    DynamicExpandingGridParams params_;
    DynamicExpandingGridData data_;
};

} // namespace map_process

#endif // _DYNAMIC_EXPANDING_GRID_
