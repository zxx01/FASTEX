/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 23:32:30
 * @LastEditTime: 2026-02-04 22:41:35
 * @Description:
 */

#ifndef _FRONTIER_MANAGER_H_
#define _FRONTIER_MANAGER_H_

#include <list>
#include <memory>
#include <shared_mutex>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "common_utils/eigen_utils.h"
#include "map_process/roadmap/history_pos_graph.h"
#include "map_process/core/map_process_params.h"
#include "map_process/searcher/path_searcher.h"
#include "plan_env/raycast.h"
#include "plan_env/sdf_map.h"
#include "utils/hash_grid.hpp"

namespace map_process
{
/**
 * @brief The struct to store the frontier unit information
 *
 */
struct FrontierUnit
{
    bool clustered_;
    eigen_utils::Vec3d pos_;
    eigen_utils::Vec3d normal_;

    FrontierUnit() = default;
    FrontierUnit(const bool& clustered, const eigen_utils::Vec3d& pos,
                 const eigen_utils::Vec3d& normal)
        : clustered_(clustered), pos_(pos), normal_(normal) {};
};

/**
 * @brief Viewpoint to cover a frontier cluster
 *
 */
struct FrontierViewpoint
{
    eigen_utils::Vec3d pos_;
    eigen_utils::Vec3d normal_;
    double visib_score_ = 0.0;

    FrontierViewpoint() = default;
    FrontierViewpoint(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& normal,
                      const double& visib_score)
        : pos_(pos), normal_(normal), visib_score_(visib_score) {};
};

/**
 * @brief The struct to store the information of the frontier cluster
 *
 */
struct FrontierClusterInfo
{
    int id_ = -1;
    int top_vp_roadmap_id_ = -1;
    bool is_dormant_ = false;
    eigen_utils::Vec3d centroid_;
    eigen_utils::Vec3d normal_;
    eigen_utils::Vec3d box_min_, box_max_;
    eigen_utils::Vec_Vec3d frontiers_, frontiers_ds_;
    eigen_utils::Vec_Vec3d normals_, normals_ds_;
    std::vector<FrontierViewpoint> viewpoints_;

    int initial_frontiers_count_ = 0;

    FrontierClusterInfo() = default;
    FrontierClusterInfo(const FrontierClusterInfo&) = default;
    FrontierClusterInfo(FrontierClusterInfo&&) noexcept = default;
    FrontierClusterInfo& operator=(const FrontierClusterInfo&) = default;
    FrontierClusterInfo& operator=(FrontierClusterInfo&&) noexcept = default;
    ~FrontierClusterInfo() = default;
};

struct FrontierClusterSummary
{
    int id_;
    eigen_utils::Vec3d centroid_;
    eigen_utils::Vec3d box_min_;
    eigen_utils::Vec3d box_max_;

    FrontierClusterSummary() : id_(-1) {};
    FrontierClusterSummary(const int id, const eigen_utils::Vec3d& centroid,
                           const eigen_utils::Vec3d& box_min, const eigen_utils::Vec3d& box_max)
        : id_(id), centroid_(centroid), box_min_(box_min), box_max_(box_max) {};
};

class FrontierManager
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<FrontierManager>;
    using UniquePtr = std::unique_ptr<FrontierManager>;

    FrontierManager() {};
    FrontierManager(ros::NodeHandle& nh, const fast_planner::SDFMap::Ptr& sdf_map);
    ~FrontierManager() {};

    // Initialization
    void setCurrentPosition(const eigen_utils::Vec3d& current_position);
    void setPathSearcher(const PathSearcher::SharedPtr path_searcher)
    {
        path_searcher_ = path_searcher;
    };
    void setWholeStateRoadMap(const WSRoadMap::SharedPtr whole_state_road_map)
    {
        whole_state_road_map_ = whole_state_road_map;
    };
    void setHistoryPosGraph(const HistoryPosGraph::SharedPtr history_pos_graph)
    {
        history_pos_graph_ = history_pos_graph;
    };

    // Frontiers
    void searchFrontiers(const std::vector<int>& changed_cells, const eigen_utils::Vec3d& bmin,
                         const eigen_utils::Vec3d& bmax);
    void acquireChangedCells(std::vector<int>& changed_cells);
    void updateGlobalClusters();
    void eraseFrontiers(const eigen_utils::Vec_Vec3i& frontiers_indices);

    // Viewpoints
    void computeKeyViewpointsForClustersWithVisibleScore2();

    // Insert the new Top Viewpoints into the WSRoadMap
    void updateTopViewpointsInWSRoadMap(const eigen_utils::Vec3d& plan_position);

    // Checkers
    bool isFrontier(const eigen_utils::Vec3i& index, const int& dim = 3);
    bool isFrontier(const eigen_utils::Vec3d& pos, const int& dim = 3);
    bool isFrontierCovered();

    // Getters
    void getActiveFrontierClusterBoxesWithinRange(
        const process_utils::CubeBox& confined_box,
        std::vector<process_utils::CubeBox>& active_cluster_boxes) const;
    void getActiveFrontierClusterSummaries(
        std::vector<FrontierClusterSummary>& active_cluster_summaries) const;

    void getGlobalSimpleFrontiers(eigen_utils::Vec_Vec3d& frontiers) const;
    void getLocalSimpleFrontiers(const double range, eigen_utils::Vec_Vec3d& frontiers) const;
    void getGlobalClusteredFrontiers(std::vector<eigen_utils::Vec_Vec3d>& clustered_frontiers,
                                     const bool ignore_dormant) const;
    void getGlobalClusteredCentroids(eigen_utils::Vec_Vec3d& clustered_centroids,
                                     const bool ignore_dormant) const;
    void
    getGlobalClusteredCentroids(std::unordered_map<int, eigen_utils::Vec3d>& clustered_centroids,
                                const bool ignore_dormant) const;
    void getGlobalClusteredNormals(eigen_utils::Vec_Vec3d& clustered_normals,
                                   const bool ignore_dormant) const;
    void getGlobalClusteredViewpoints(std::vector<eigen_utils::Vec_Vec3d>& clustered_viewpoints,
                                      const bool ignore_dormant) const;
    void getGlobalClusteredTopViewpoints(
        std::vector<std::pair<eigen_utils::Vec3d, double>>& clustered_top_viewpoints,
        const bool ignore_dormant) const;
    void getRemovedActiveClustersIndices(std::vector<int>& removed_cluster_indices) const;
    void getClusterFrontiers(const int& cluster_id, eigen_utils::Vec_Vec3d& frontiers) const;
    void getViewpointsForClusterIndices(const std::vector<int>& cluster_indices,
                                        std::vector<eigen_utils::Vec_Vec3d>& viewpoints,
                                        const int vp_num = -1) const;

    template <typename Func>
    void getSubDataFromFrontierInfo(Func extractor, const bool ignore_dormant) const
    {
        {
            std::shared_lock<std::shared_mutex> lock(active_frontier_clusters_info_mutex_);
            for (const auto& cluster_ptr : active_frontier_clusters_info_ptr_)
            {
                if (cluster_ptr)
                {
                    extractor(*cluster_ptr);
                }
            }
        }

        if (!ignore_dormant)
        {
            std::shared_lock<std::shared_mutex> lock(dormant_frontier_clusters_info_mutex_);
            for (const auto& cluster_ptr : dormant_frontier_clusters_info_ptr_)
            {
                if (cluster_ptr)
                {
                    extractor(*cluster_ptr);
                }
            }
        }
    }

    // Visualization
    void pubFrontierStatistics();
    void pubSimpleGlobalFrontiersMarkers();

    fast_planner::SDFMap::Ptr sdf_map_;
    PathSearcher::SharedPtr path_searcher_;
    WSRoadMap::SharedPtr whole_state_road_map_;
    HistoryPosGraph::SharedPtr history_pos_graph_;

    ros::Publisher SDFMap_frontier_statistics_pub_, global_frontiers_pub_;

  private:
    void loadParamsFromROS(const ros::NodeHandle& nh);

    // Search frontiers
    void findNewLocalFrontiers(const std::vector<int>& changed_cells);
    void updatePreviousClusters(const eigen_utils::Vec3d& bmin, const eigen_utils::Vec3d& bmax);
    void updateDiscreteFrontiers(const eigen_utils::Vec3d& bmin, const eigen_utils::Vec3d& bmax);
    void computeAndStoreFrontierNormals(
        eigen_utils::Vec3iMap<eigen_utils::Vec3d>& local_frontier_normals_map,
        const eigen_utils::Vec3iSet& active_indices, const int nbr_d);
    void smoothFrontiersNormals(const eigen_utils::Vec3iMap<eigen_utils::Vec3d>& normals_map,
                                eigen_utils::Vec3iMap<eigen_utils::Vec3d>& smooth_normals_map,
                                const int kernel_size = 3);
    void updateGlobalFrontiers(
        const std::list<std::shared_ptr<FrontierClusterInfo>>& frontier_clusters_info);

    // Split the frontiers into clusters
    void clusterFrontiersByRegionGrowing(
        std::list<std::shared_ptr<FrontierClusterInfo>>& local_clusters);

    // Update the clusters information
    void updateClustersInfo(FrontierClusterInfo& splits);
    void downsampleClusters(FrontierClusterInfo& frontier_clusters_info, const double voxel_size);

    // Compute the viewpoint for the clusters
    void generateViewpointsForClusters(FrontierClusterInfo& cluster);
    void generateCylinderSamplingPoints(eigen_utils::Vec_Vec3d& points, double r_min, double r_max,
                                        double z_min, double z_max, double step_z0, double step_r0,
                                        double angle_step0, double k_z = 0.0, double k_r = 0.0,
                                        double k_theta = 0.0);
    void generateViewpointsWithDownsampledFrontiersAndNormals(
        const eigen_utils::Vec_Vec3d& frontiers, const eigen_utils::Vec_Vec3d& normals,
        const double min_dist, const double max_dist, const int step_num,
        eigen_utils::Vec_Vec3d& sample_points, eigen_utils::Vec_Vec3d& sample_normals);
    void transformPoints(const eigen_utils::Vec_Vec3d& points_raw,
                         eigen_utils::Vec_Vec3d& points_transformed,
                         const eigen_utils::Vec3d& translation,
                         const eigen_utils::Vec3d& initial_direction,
                         const eigen_utils::Vec3d& direction);
    int computeVisibleScore(FrontierViewpoint& vp, const std::vector<FrontierUnit>& frontiers,
                            std::vector<bool>& visible_frontiers);
    bool tryUseClusterCentroidAsViewpoint(FrontierClusterInfo& cluster);

    double computeAdaptiveThreshold(const std::vector<FrontierViewpoint>& viewpoints,
                                    const double k = 0.5);
    std::vector<size_t> thresholdFilterViewpoints(const std::vector<FrontierViewpoint>& viewpoints,
                                                  const double threshold);
    double computeAverageDistance(const std::vector<FrontierViewpoint>& viewpoints);
    std::vector<size_t>
    nonMaximumSuppression(const std::vector<FrontierViewpoint>& candidate_viewpoints,
                          const std::vector<size_t>& score_filtered_indices,
                          const double distance_hreshold, const size_t max_viewpoint_num);

    // Utils
    bool isInFov(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& obj) const;
    bool isNeighborUnknown(const eigen_utils::Vec3i& index, const int& dim = 3) const;
    bool isSafe(const eigen_utils::Vec3i& idx, const double min_clearance = -1.0) const;
    bool isSafe(const eigen_utils::Vec3d& pos, const double min_clearance = -1.0) const;

    // Visualization
    void generateMarkerArray(visualization_msgs::Marker& frontier_cell_markers,
                             const eigen_utils::Vec_Vec3d& frontier_cells,
                             const eigen_utils::Vec4d& rgba) const;
    void
    generateNormalsMarkers(visualization_msgs::Marker& normal_markers,
                           const eigen_utils::Vec3iMap<eigen_utils::Vec3d> frontier_normals_map,
                           const eigen_utils::Vec4d& rgba) const;

    template <typename InputType, typename OutputType, typename Func>
    void parallelProcess(const std::vector<InputType>& input_vec,
                         std::vector<OutputType>& output_vec, Func process_func, int chunk_size,
                         int num_threads);

    FrontierParams frontier_params_;

    eigen_utils::Vec3d current_position_;

    std::unordered_set<int> simple_known_cells_;

    // global frontier grid used to store frontier scores: the active state indicates the frontier
    // cells, and the value indicates the frontier score
    utils::HashGridManager<eigen_utils::Vec3iSet>::UniquePtr global_frontier_hash_grid_;
    eigen_utils::Vec3iMap<FrontierUnit> global_frontier_map_;

    // Clusters information
    std::vector<int> removed_cluster_ids_;
    std::vector<std::pair<int, eigen_utils::Vec3d>> removed_cluster_topvp_rm_ids_;
    int new_cluster_start_index_;

    std::list<std::shared_ptr<FrontierClusterInfo>> active_frontier_clusters_info_ptr_,
        dormant_frontier_clusters_info_ptr_, local_new_frontier_clusters_info_ptr_;

    mutable std::shared_mutex active_frontier_clusters_info_mutex_,
        dormant_frontier_clusters_info_mutex_, local_new_frontier_clusters_info_mutex_;

    mutable std::shared_mutex global_frontier_map_mutex_;

    eigen_utils::Vec3iMap<eigen_utils::Vec3d> local_frontier_normals_map_;

    // Utils
    std::unique_ptr<RayCaster> raycaster_;

  private:
    // For fast frontier checking, storage and processing
    struct FastFrontierGrid
    {
        // use unordered_set to store active frontier indices
        eigen_utils::Vec3iSet active_indices_;
        eigen_utils::Vec3iSet visited_indices_;

        // buffer pool to avoid repeated allocation
        std::queue<eigen_utils::Vec3i> bfs_queue_;

        // batch add
        void addBatch(const eigen_utils::Vec_Vec3i& indices)
        {
            active_indices_.insert(indices.begin(), indices.end());
        }

        // fast lookup
        inline bool isActive(const eigen_utils::Vec3i& idx) const
        {
            return active_indices_.count(idx) > 0;
        }

        inline bool isVisited(const eigen_utils::Vec3i& idx) const
        {
            return visited_indices_.count(idx) > 0;
        }

        // clear and reuse memory
        void clear()
        {
            active_indices_.clear();
            visited_indices_.clear();
        }

        // get all indices (avoid copy)
        const eigen_utils::Vec3iSet& getActiveIndices() const { return active_indices_; }
    };

    FastFrontierGrid local_frontier_grid_;

    // cached object pool
    struct CachePool
    {
        eigen_utils::Vec_Vec3d vec3d_buffer_;
        eigen_utils::Vec_Vec3i vec3i_buffer_;

        void clear()
        {
            vec3d_buffer_.clear();
            vec3i_buffer_.clear();
        }
    };

    CachePool cache_pool_;
};

} // namespace map_process

#endif // _FRONTIER_MANAGER_H_
