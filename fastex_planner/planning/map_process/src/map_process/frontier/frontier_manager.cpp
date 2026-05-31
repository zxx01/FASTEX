/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-04 10:18:55
 * @LastEditTime: 2026-04-30 14:30:02
 * @Description:
 */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fastex_msgs/frontierStatistics.h"
#include "map_process/frontier/frontier_manager.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace map_process
{
FrontierManager::FrontierManager(ros::NodeHandle& nh, const fast_planner::SDFMap::Ptr& sdf_map)
{
    sdf_map_ = sdf_map;
    sdf_map_->enableChangeDetection();

    loadParamsFromROS(nh);

    // compute grid offset
    const double grid_resolution = sdf_map_->getResolution();
    global_frontier_hash_grid_ = std::make_unique<utils::HashGridManager<eigen_utils::Vec3iSet>>(
        eigen_utils::Vec3d(0, 0, 0), 10.0 * grid_resolution);

    // Set up raycaster
    raycaster_ = std::make_unique<RayCaster>();
    eigen_utils::Vec3d size;
    eigen_utils::Vec3d origin;
    sdf_map_->getRegion(origin, size);
    raycaster_->setParams(grid_resolution, origin);

    // Publishers
    SDFMap_frontier_statistics_pub_ =
        nh.advertise<fastex_msgs::frontierStatistics>("/data_log_manager_node/map_info", 2);
    global_frontiers_pub_ =
        nh.advertise<visualization_msgs::Marker>("/topo_planner/global_frontier_marker", 10);
}

/**
 * @brief load params from ROS
 *
 */
void FrontierManager::loadParamsFromROS(const ros::NodeHandle& nh)
{
    bool is_param_load = true;

    is_param_load &=
        nh.param("FrontierManager/frame_id", frontier_params_.frame_id_, std::string("world"));
    is_param_load &= nh.param("FrontierManager/min_visible_voxel_num",
                              frontier_params_.min_visible_voxel_num_, 10);
    is_param_load &= nh.param("FrontierManager/min_cluster_voxel_num",
                              frontier_params_.min_cluster_voxel_num_, 20);
    is_param_load &=
        nh.param("FrontierManager/max_cluster_radius", frontier_params_.max_cluster_radius_, 5.0f);
    is_param_load &= nh.param("FrontierManager/frontier_filter_ratio",
                              frontier_params_.frontier_filter_ratio_, 2.0f);
    is_param_load &=
        nh.param("FrontierManager/upper_fov_angle", frontier_params_.upper_fov_angle_, 52.0f);
    is_param_load &=
        nh.param("FrontierManager/lower_fov_angle", frontier_params_.lower_fov_angle_, -7.0f);
    is_param_load &= nh.param("FrontierManager/min_view_finish_fraction",
                              frontier_params_.min_view_finish_fraction_, 0.5f);
    is_param_load &= nh.param("FrontierManager/frontier_clear_threshold",
                              frontier_params_.frontier_clear_threshold_, 0.0f);
    is_param_load &= nh.param("FrontierManager/min_candidate_clearance",
                              frontier_params_.min_candidate_clearance_, 1.0f);

    is_param_load &=
        nh.param("FrontierManager/candidate_rmin", frontier_params_.candidate_rmin_, 0.0);
    is_param_load &=
        nh.param("FrontierManager/candidate_rmax", frontier_params_.candidate_rmax_, 3.0);
    is_param_load &=
        nh.param("FrontierManager/candidate_hmin", frontier_params_.candidate_hmin_, 1.0);
    is_param_load &=
        nh.param("FrontierManager/candidate_hmax", frontier_params_.candidate_hmax_, 3.0);
    is_param_load &=
        nh.param("FrontierManager/candidate_rstep", frontier_params_.candidate_rstep_, 0.5);
    is_param_load &=
        nh.param("FrontierManager/candidate_hstep", frontier_params_.candidate_hstep_, 1.0);
    is_param_load &= nh.param("FrontierManager/candidate_theta_step",
                              frontier_params_.candidate_theta_step_, M_PI / 6);
    is_param_load &= nh.param("FrontierManager/candidate_kr", frontier_params_.candidate_kr_, 0.0);
    is_param_load &= nh.param("FrontierManager/candidate_kh", frontier_params_.candidate_kh_, 0.0);
    is_param_load &=
        nh.param("FrontierManager/candidate_ktheta", frontier_params_.candidate_ktheta_, 0.0);

    is_param_load &= nh.param("FrontierManager/refined_viewpoint_num",
                              frontier_params_.refined_viewpoint_num_, 10);

    if (!is_param_load)
        ROS_ERROR("FrontierManager params load failed!");

    Eigen::Vector3d bmin, bmax;
    sdf_map_->getBox(bmin, bmax);
    frontier_params_.frontier_cutoff_min_ = bmin(2);
    frontier_params_.frontier_cutoff_max_ = bmax(2);
}

/**
 * @brief set current position
 *
 * @param current_position the current position
 */
void FrontierManager::setCurrentPosition(const eigen_utils::Vec3d& current_position)
{
    current_position_ = current_position;
}

/**
 * @brief search frontiers
 *
 * @param changed_cells The changed cells that used to search frontiers
 * @param bmin The minimum point of the perception range
 * @param bmax The maximum point of the perception range
 */
void FrontierManager::searchFrontiers(const std::vector<int>& changed_cells,
                                      const eigen_utils::Vec3d& bmin,
                                      const eigen_utils::Vec3d& bmax)
{
    time_utils::Timer::Ptr timer_search_frontiers =
        std::make_shared<time_utils::Timer>("searchFrontiers");
    timer_search_frontiers->start();

    local_new_frontier_clusters_info_ptr_.clear();

    // local frontier grid used to cluster frontiers: the active state indicates the frontier
    // cells, and the value indicates if the frontier is clustered
    local_frontier_grid_.clear();
    cache_pool_.clear();

    // 2. extract newly added local frontiers
    time_utils::Timer::Ptr timer_find_new_frontiers =
        std::make_shared<time_utils::Timer>("findNewLocalFrontiers");
    timer_find_new_frontiers->start();

    findNewLocalFrontiers(changed_cells);

    timer_find_new_frontiers->stop(false, "ms");

    // 3. detect intersection of existing frontier clusters with the sensing range; break up
    // intersecting clusters,
    //    discard parts that are no longer frontiers (and remove corresponding global frontiers),
    //    merge remainder into local frontiers
    time_utils::Timer::Ptr timer_update_previous_clusters =
        std::make_shared<time_utils::Timer>("updatePreviousClusters");
    timer_update_previous_clusters->start();

    updatePreviousClusters(bmin, bmax);

    timer_update_previous_clusters->stop(false, "ms");

    // 4. update isolated frontier states: remove those no longer valid, retain those still
    // frontiers
    time_utils::Timer::Ptr timer_update_discrete_frontiers =
        std::make_shared<time_utils::Timer>("updateDiscreteFrontiers");
    timer_update_discrete_frontiers->start();

    updateDiscreteFrontiers(bmin, bmax);

    timer_update_discrete_frontiers->stop(false, "ms");

    // 5. cluster local frontiers
    time_utils::Timer::Ptr timer_cluster_frontiers =
        std::make_shared<time_utils::Timer>("clusterFrontiers");
    timer_cluster_frontiers->start();

    std::list<std::shared_ptr<FrontierClusterInfo>> frontier_clusters_info_temp;
    clusterFrontiersByRegionGrowing(frontier_clusters_info_temp);

    timer_cluster_frontiers->stop(false, "ms");

    // 6. downsample each cluster
    time_utils::Timer::Ptr timer_downsample_clusters =
        std::make_shared<time_utils::Timer>("downsampleClusters");
    timer_downsample_clusters->start();

    for (auto& cluster : frontier_clusters_info_temp)
        downsampleClusters(*cluster,
                           sdf_map_->getResolution() * frontier_params_.frontier_filter_ratio_);

    timer_downsample_clusters->stop(false, "ms");

    // 7. update global frontiers
    time_utils::Timer::Ptr timer_update_global_frontiers =
        std::make_shared<time_utils::Timer>("updateGlobalFrontiers");
    timer_update_global_frontiers->start();

    updateGlobalFrontiers(frontier_clusters_info_temp);

    timer_update_global_frontiers->stop(false, "ms");

    // 8. if a cluster center falls on an obstacle, move it to the nearest free point;
    // if no free point is found within the search radius, move center to the cluster's first
    // frontier
    time_utils::Timer::Ptr timer_search_free_center =
        std::make_shared<time_utils::Timer>("searchFreeClusterCenter");
    timer_search_free_center->start();

    eigen_utils::Vec3d search_box_size(8.0, 8.0, 8.0);
    for (auto& cluster : frontier_clusters_info_temp)
    {
        if (sdf_map_->isInflatedOccupied(cluster->centroid_))
        {
            eigen_utils::Vec3d nearest_free;
            bool is_modified =
                sdf_map_->bfsNearestFree(search_box_size, cluster->centroid_, nearest_free);

            if (is_modified)
            {
                cluster->centroid_ = nearest_free;
            }
            else
                cluster->centroid_ = cluster->frontiers_.front();
        }
    }

    timer_search_free_center->stop(false, "ms");

    {
        std::unique_lock<std::shared_mutex> local_lock(local_new_frontier_clusters_info_mutex_);
        local_new_frontier_clusters_info_ptr_.swap(frontier_clusters_info_temp);
    }

    timer_search_frontiers->stop(false, "ms");
}

/**
 * @brief acquire changed cells from sdf_map_
 *
 */
void FrontierManager::acquireChangedCells(std::vector<int>& changed_cells)
{
    sdf_map_->change_mutex_.lock();
    changed_cells.assign(sdf_map_->changesBegin(), sdf_map_->changesEnd());
    sdf_map_->resetChangeDetection();
    sdf_map_->change_mutex_.unlock();

    simple_known_cells_.insert(changed_cells.begin(), changed_cells.end());
}

void FrontierManager::findNewLocalFrontiers(const std::vector<int>& changed_cells)
{
    const double sq_resolution = sdf_map_->getResolution() * sdf_map_->getResolution();

    // pre-allocate capacity
    cache_pool_.vec3i_buffer_.clear();
    cache_pool_.vec3i_buffer_.reserve(changed_cells.size() / 10); // estimate frontier ratio

    eigen_utils::Vec3d point_pos;
    eigen_utils::Vec3i point_index;

    // batch-check frontiers
    for (const int& changed_cell : changed_cells)
    {
        point_pos = sdf_map_->AddressToPos(changed_cell);

        // fast pre-check
        if (point_pos.z() < frontier_params_.frontier_cutoff_min_ ||
            point_pos.z() > frontier_params_.frontier_cutoff_max_ ||
            (point_pos - current_position_).squaredNorm() <= sq_resolution)
            continue;

        point_index = sdf_map_->AddressToIndex(changed_cell);

        if (sdf_map_->isInBox(point_index) && isFrontier(point_index, 3))
            cache_pool_.vec3i_buffer_.push_back(point_index);
    }

    // batch-add to grid
    local_frontier_grid_.addBatch(cache_pool_.vec3i_buffer_);
}

void FrontierManager::updatePreviousClusters(const eigen_utils::Vec3d& bmin,
                                             const eigen_utils::Vec3d& bmax)
{
    // reuse buffer
    eigen_utils::Vec_Vec3i& reserved_indices = cache_pool_.vec3i_buffer_;
    eigen_utils::Vec_Vec3d& reserved_frontiers = cache_pool_.vec3d_buffer_;
    eigen_utils::Vec_Vec3i erased_indices;
    eigen_utils::Vec_Vec3i cluster_valid_indices;
    eigen_utils::Vec_Vec3i cluster_invalid_indices;
    reserved_indices.clear();
    reserved_frontiers.clear();
    erased_indices.reserve(5000);
    cluster_valid_indices.reserve(5000);
    cluster_invalid_indices.reserve(5000);

    removed_cluster_ids_.clear();
    removed_cluster_topvp_rm_ids_.clear();

    auto computeClearRatio = [](const FrontierClusterInfo& cluster,
                                const size_t reserved_frontier_num) -> double {
        if (cluster.initial_frontiers_count_ <= 0)
            return 0.0;

        return static_cast<double>(cluster.initial_frontiers_count_ - reserved_frontier_num) /
               cluster.initial_frontiers_count_;
    };

    auto cacheRemovedActiveCluster = [&](const std::shared_ptr<FrontierClusterInfo>& cluster,
                                         const int cluster_index) {
        const eigen_utils::Vec3d removal_anchor =
            cluster->viewpoints_.empty() ? cluster->centroid_ : cluster->viewpoints_.front().pos_;
        removed_cluster_topvp_rm_ids_.emplace_back(cluster->top_vp_roadmap_id_, removal_anchor);
        removed_cluster_ids_.push_back(cluster_index);
    };

    auto shouldRebuildActiveCluster = [&](const FrontierClusterInfo& cluster,
                                          const size_t reserved_frontier_num) -> bool {
        const double clear_ratio = computeClearRatio(cluster, reserved_frontier_num);
        return clear_ratio >= frontier_params_.frontier_clear_threshold_;
    };

    const bool always_rebuild_active_clusters = frontier_params_.frontier_clear_threshold_ <= 0.0f;

    auto processClustersFrontiers = [&](auto& clusters, std::shared_mutex& mutex,
                                        const bool rebuild_active_clusters) {
        std::unique_lock<std::shared_mutex> lock(mutex);

        int rmv_idx = 0;
        for (auto iter = clusters.begin(); iter != clusters.end();)
        {
            auto& cluster = *iter;

            // bounding box check
            if (!process_utils::ProcessUtils::isOverlapped(cluster->box_min_, cluster->box_max_,
                                                           bmin, bmax))
            {
                ++iter;
                ++rmv_idx;
                continue;
            }

            cluster_valid_indices.clear();
            cluster_invalid_indices.clear();
            cluster_valid_indices.reserve(cluster->frontiers_.size());
            cluster_invalid_indices.reserve(cluster->frontiers_.size());

            const bool need_reserved_frontiers =
                rebuild_active_clusters && !always_rebuild_active_clusters;
            if (need_reserved_frontiers)
            {
                reserved_frontiers.clear();
                reserved_frontiers.reserve(cluster->frontiers_.size());
            }

            // batch-process frontiers
            eigen_utils::Vec3i index;
            for (const auto& ft : cluster->frontiers_)
            {
                sdf_map_->posToIndex(ft, index);
                if (isFrontier(index, 3))
                {
                    if (need_reserved_frontiers)
                        reserved_frontiers.push_back(ft);
                    cluster_valid_indices.push_back(index);
                }
                else
                {
                    cluster_invalid_indices.push_back(index);
                }
            }

            const bool erase_cluster =
                !need_reserved_frontiers ||
                shouldRebuildActiveCluster(*cluster, cluster_valid_indices.size());

            if (erase_cluster)
            {
                if (rebuild_active_clusters)
                    cacheRemovedActiveCluster(cluster, rmv_idx);

                // retain still-valid frontiers for subsequent local re-clustering
                reserved_indices.insert(reserved_indices.end(), cluster_valid_indices.begin(),
                                        cluster_valid_indices.end());
                erased_indices.insert(erased_indices.end(), cluster_valid_indices.begin(),
                                      cluster_valid_indices.end());
                erased_indices.insert(erased_indices.end(), cluster_invalid_indices.begin(),
                                      cluster_invalid_indices.end());

                iter = clusters.erase(iter);
            }
            else
            {
                // keep the current active cluster; only remove invalid frontiers
                // Use swap to avoid copying the filtered frontier list back into the cluster.
                cluster->frontiers_.swap(reserved_frontiers);
                erased_indices.insert(erased_indices.end(), cluster_invalid_indices.begin(),
                                      cluster_invalid_indices.end());
                ++iter;
            }

            ++rmv_idx;
        }
    };

    processClustersFrontiers(active_frontier_clusters_info_ptr_,
                             active_frontier_clusters_info_mutex_, true);
    processClustersFrontiers(dormant_frontier_clusters_info_ptr_,
                             dormant_frontier_clusters_info_mutex_, false);

    eraseFrontiers(erased_indices);

    // batch-add retained frontiers
    local_frontier_grid_.addBatch(reserved_indices);
}

void FrontierManager::updateDiscreteFrontiers(const eigen_utils::Vec3d& bmin,
                                              const eigen_utils::Vec3d& bmax)
{
    // use cache to avoid repeated allocation
    eigen_utils::Vec_Vec3i& reserved_indices = cache_pool_.vec3i_buffer_;
    eigen_utils::Vec_Vec3i erased_frontiers_indices;
    reserved_indices.clear();
    erased_frontiers_indices.reserve(1000);

    // expand search range
    eigen_utils::Vec3d inflate_margin_local(frontier_params_.max_cluster_radius_ / 2.0,
                                            frontier_params_.max_cluster_radius_ / 2.0,
                                            frontier_params_.max_cluster_radius_ / 2.0);

    // search in global frontier hash grid
    std::vector<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3iSet>> all_frontiers =
        global_frontier_hash_grid_->boxSearch(bmin - inflate_margin_local,
                                              bmax + inflate_margin_local);

    // iterate over all found frontiers
    {
        std::shared_lock<std::shared_mutex> lock(global_frontier_map_mutex_);
        for (const auto& [grid_idx, fts] : all_frontiers)
        {
            for (const auto& ft_index : fts)
            {
                auto iter = global_frontier_map_.find(ft_index);
                if (iter == global_frontier_map_.end() || iter->second.clustered_)
                    continue;

                // check if it is still a valid frontier
                if (isFrontier(ft_index, 3))
                    reserved_indices.push_back(ft_index);

                // mark for deletion (regardless of frontier validity)
                erased_frontiers_indices.push_back(ft_index);
            }
        }
    }

    // batch-delete frontiers
    eraseFrontiers(erased_frontiers_indices);

    // batch-add retained frontiers to local grid
    local_frontier_grid_.addBatch(reserved_indices);
}

void FrontierManager::computeAndStoreFrontierNormals(
    eigen_utils::Vec3iMap<eigen_utils::Vec3d>& local_frontier_normals_map,
    const eigen_utils::Vec3iSet& active_indices, const int nbr_d)
{
    // convert to vector for parallel indexed access
    std::vector<eigen_utils::Vec3i> idx_vec(active_indices.begin(), active_indices.end());
    std::vector<eigen_utils::Vec3d> normals_out;

    // define normal vector computation function
    auto calculateNormal3D = [this, nbr_d](const eigen_utils::Vec3i& ijk) -> eigen_utils::Vec3d {
        eigen_utils::Vec3d normal(0.0, 0.0, 0.0);

        // iterate over neighboring voxels
        for (int dx = -nbr_d; dx <= nbr_d; ++dx)
            for (int dy = -nbr_d; dy <= nbr_d; ++dy)
                for (int dz = -nbr_d; dz <= nbr_d; ++dz)
                {
                    eigen_utils::Vec3i neighbor(ijk.x() + dx, ijk.y() + dy, ijk.z() + dz);

                    // If the neighbor voxel is free, add its direction to the normal
                    if (sdf_map_->isInflatedFree(neighbor))
                        normal += eigen_utils::Vec3d(dx, dy, dz);
                    else if (sdf_map_->isFree(neighbor))
                        normal += eigen_utils::Vec3d(dx, dy, dz) * 0.5;

                    // else if (sdf_map_->isInflatedUnknown(neighbor))
                    //   normal -= eigen_utils::Vec3d(dx, dy, dz) * 0.5;
                }

        // normalize the normal vector
        if (normal.squaredNorm() > 0.0)
            normal.normalize();

        return normal;
    };

    // parallel normal vector computation
    parallelProcess(idx_vec, normals_out, calculateNormal3D, 64, 8);

    // merge results into map
    local_frontier_normals_map.clear();
    local_frontier_normals_map.reserve(idx_vec.size());
    for (size_t i = 0; i < idx_vec.size(); ++i)
        local_frontier_normals_map.insert_or_assign(idx_vec[i], std::move(normals_out[i]));
}

/**
 * @brief Smooth the normals of frontiers using a kernel.
 *
 * This function smooths the normals of frontiers by averaging the normals within a specified
 * kernel size. The smoothed normals are stored in the provided smooth_normals_map.
 *
 * @param normals_map A map containing the original normals of the frontiers.
 * @param smooth_normals_map A map to store the smoothed normals of the frontiers.
 * @param kernel_size The size of the kernel used for smoothing.
 */
void FrontierManager::smoothFrontiersNormals(
    const eigen_utils::Vec3iMap<eigen_utils::Vec3d>& normals_map,
    eigen_utils::Vec3iMap<eigen_utils::Vec3d>& smooth_normals_map, const int kernel_size)
{
    smooth_normals_map.clear();

    std::vector<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3d>> normals_vec(normals_map.begin(),
                                                                               normals_map.end());
    std::vector<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3d>> smooth_normals_vec(
        normals_vec.size());

    const int radius = kernel_size / 2;

    // define smoothing function
    auto smoothSingleNormal =
        [&normals_map, radius](const std::pair<eigen_utils::Vec3i, eigen_utils::Vec3d>& item)
        -> std::pair<eigen_utils::Vec3i, eigen_utils::Vec3d> {
        const eigen_utils::Vec3i& idx = item.first;
        eigen_utils::Vec3d smooth_normal = eigen_utils::Vec3d::Zero();

        for (int dx = -radius; dx <= radius; ++dx)
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dz = -radius; dz <= radius; ++dz)
                {
                    eigen_utils::Vec3i neighbor = idx + eigen_utils::Vec3i(dx, dy, dz);
                    auto it = normals_map.find(neighbor);
                    if (it != normals_map.end())
                        smooth_normal += it->second;
                }

        if (smooth_normal.squaredNorm() > 0)
            smooth_normal.normalize();

        return {idx, smooth_normal};
    };

    parallelProcess(normals_vec, smooth_normals_vec, smoothSingleNormal, 64, 8);

    // merge results into map
    smooth_normals_map.reserve(smooth_normals_vec.size());
    for (auto& [idx, normal] : smooth_normals_vec)
        smooth_normals_map.insert_or_assign(idx, std::move(normal));
}

/**
 * @brief Downsample frontier clusters
 *
 * This function downsamples the frontier clusters by grouping points into voxels and selecting
 * the point closest to the centroid of each voxel.
 *
 * @param frontier_clusters_info The information of the frontier clusters to be downsampled
 * @param voxel_size The size of the voxel used for downsampling
 */
void FrontierManager::downsampleClusters(FrontierClusterInfo& frontier_clusters_info,
                                         const double voxel_size)
{
    // Define voxel data structure
    struct VoxelData
    {
        eigen_utils::Vec3d centroid_sum = eigen_utils::Vec3d::Zero(); // Sum of centroids
        int num_points = 0;                                           // Number of points
        std::vector<int> indices;                                     // List of point indices
    };

    eigen_utils::Vec3iMap<VoxelData> voxel_map;

    const auto& frontiers = frontier_clusters_info.frontiers_;
    const auto& normals = frontier_clusters_info.normals_;

    for (size_t i = 0; i < frontiers.size(); ++i)
    {
        const eigen_utils::Vec3d& point = frontiers[i];

        // Calculate voxel index
        eigen_utils::Vec3i voxel_idx;
        voxel_idx[0] = static_cast<int>(std::floor(point[0] / voxel_size));
        voxel_idx[1] = static_cast<int>(std::floor(point[1] / voxel_size));
        voxel_idx[2] = static_cast<int>(std::floor(point[2] / voxel_size));

        // Update voxel data
        auto& voxel = voxel_map[voxel_idx];
        voxel.centroid_sum += point;
        voxel.num_points += 1;
        voxel.indices.push_back(static_cast<int>(i));
    }

    // Create downsampled point set
    eigen_utils::Vec_Vec3d downsampled_points, downsampled_normals;
    downsampled_points.reserve(voxel_map.size());
    downsampled_normals.reserve(voxel_map.size());

    // Iterate through each voxel and select the original point closest to the centroid
    for (const auto& item : voxel_map)
    {
        const VoxelData& voxel = item.second;
        // Calculate centroid
        eigen_utils::Vec3d centroid = voxel.centroid_sum / static_cast<double>(voxel.num_points);

        // Find the original point closest to the centroid within the voxel
        double min_distance_sq = std::numeric_limits<double>::max();
        int closest_point_idx = -1;

        for (const int idx : voxel.indices)
        {
            const eigen_utils::Vec3d& point = frontiers[idx];
            double distance_sq = (point - centroid).squaredNorm();
            if (distance_sq < min_distance_sq)
            {
                min_distance_sq = distance_sq;
                closest_point_idx = idx;
            }
        }

        // Add the found point to the downsampled point set
        if (closest_point_idx != -1)
        {
            downsampled_points.emplace_back(frontiers[closest_point_idx]);
            downsampled_normals.emplace_back(normals[closest_point_idx]);
        }
    }

    frontier_clusters_info.frontiers_ds_ = std::move(downsampled_points);
    frontier_clusters_info.normals_ds_ = std::move(downsampled_normals);
}

void FrontierManager::updateGlobalFrontiers(
    const std::list<std::shared_ptr<FrontierClusterInfo>>& frontier_clusters_info)
{
    std::unique_lock<std::shared_mutex> lock(global_frontier_map_mutex_);

    // collect all clustered indices (for marking clustered state)
    eigen_utils::Vec3iSet clustered_indices;
    clustered_indices.clear();

    // add clustered frontiers to the global index set and global map first (prioritize clustered
    // points)
    eigen_utils::Vec3i ft_idx;
    for (const auto& cluster : frontier_clusters_info)
    {
        for (const auto& ft_pos : cluster->frontiers_)
        {
            sdf_map_->posToIndex(ft_pos, ft_idx);
            clustered_indices.insert(ft_idx);

            // add index to global hash grid (by position cell)
            global_frontier_hash_grid_
                ->getOrCreateGridData(global_frontier_hash_grid_->getGridIndex(ft_pos))
                .insert(ft_idx);

            // prefer local normal cache; fall back to cluster normal
            eigen_utils::Vec3d ft_normal = cluster->normal_;
            auto itn = local_frontier_normals_map_.find(ft_idx);
            if (itn != local_frontier_normals_map_.end())
                ft_normal = itn->second;

            // update global map (clustered point, clustered = true)
            global_frontier_map_[ft_idx] = FrontierUnit(true, ft_pos, ft_normal);
        }
    }

    // iterate over all active indices in the local grid (including unclustered frontiers), update
    // global containers
    for (const auto& idx : local_frontier_grid_.getActiveIndices())
    {
        if (clustered_indices.count(idx) > 0)
            continue; // skip already-processed clustered points

        eigen_utils::Vec3d pos;
        sdf_map_->indexToPos(idx, pos);

        eigen_utils::Vec3d normal = eigen_utils::Vec3d::Zero();
        auto itn = local_frontier_normals_map_.find(idx);
        if (itn != local_frontier_normals_map_.end())
            normal = itn->second;

        // ensure the index exists in the hash grid
        global_frontier_hash_grid_
            ->getOrCreateGridData(global_frontier_hash_grid_->getGridIndex(pos))
            .insert(idx);

        // update or insert global frontier cell (unclustered, clustered = false)
        global_frontier_map_[idx] = FrontierUnit(false, pos, normal);
    }
}

/**
 * @brief update global clusters info
 *
 */
void FrontierManager::updateGlobalClusters()
{
    std::unique_lock<std::shared_mutex> local_lock(local_new_frontier_clusters_info_mutex_);
    std::unique_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
    std::unique_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);

    new_cluster_start_index_ = active_frontier_clusters_info_ptr_.size();

    for (const auto& cluster : local_new_frontier_clusters_info_ptr_)
    {
        if (cluster->is_dormant_)
            dormant_frontier_clusters_info_ptr_.push_back(cluster);
        else
            active_frontier_clusters_info_ptr_.push_back(cluster);
    }

    int i = 0;
    for (auto& cluster : active_frontier_clusters_info_ptr_)
        cluster->id_ = i++;

    i = 0;
    for (auto& cluster : dormant_frontier_clusters_info_ptr_)
        cluster->id_ = i++;
}

/**
 * @brief Erases the specified frontiers from the global frontier map and the KD-tree.
 *
 * This function takes a list of frontier indices, finds the corresponding frontiers
 * in the global frontier map, and removes them. It also deletes the points from the
 * KD-tree to ensure consistency.
 *
 * @param frontiers_indices A vector of 3D indices representing the frontiers to be erased.
 */
void FrontierManager::eraseFrontiers(const eigen_utils::Vec_Vec3i& frontiers_indices)
{
    std::unique_lock<std::shared_mutex> lock(global_frontier_map_mutex_);
    for (const eigen_utils::Vec3i& index : frontiers_indices)
    {
        auto iter = global_frontier_map_.find(index);
        if (iter != global_frontier_map_.end())
        {
            global_frontier_hash_grid_
                ->getOrCreateGridData(global_frontier_hash_grid_->getGridIndex(iter->second.pos_))
                .erase(index);
            global_frontier_map_.erase(iter);
        }
    }
}

void FrontierManager::getActiveFrontierClusterBoxesWithinRange(
    const process_utils::CubeBox& confined_box,
    std::vector<process_utils::CubeBox>& active_cluster_boxes) const
{
    std::shared_lock<std::shared_mutex> lock(active_frontier_clusters_info_mutex_);
    active_cluster_boxes.clear();
    active_cluster_boxes.reserve(active_frontier_clusters_info_ptr_.size());
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
    {
        if (cluster &&
            process_utils::ProcessUtils::isOverlapped(cluster->box_min_, cluster->box_max_,
                                                      confined_box.min_, confined_box.max_))
        {
            active_cluster_boxes.emplace_back(cluster->box_min_, cluster->box_max_);
        }
    }
}

void FrontierManager::getActiveFrontierClusterSummaries(
    std::vector<FrontierClusterSummary>& active_cluster_summaries) const
{
    std::shared_lock<std::shared_mutex> lock(active_frontier_clusters_info_mutex_);
    active_cluster_summaries.clear();
    active_cluster_summaries.reserve(active_frontier_clusters_info_ptr_.size());
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
    {
        if (cluster)
        {
            active_cluster_summaries.emplace_back(cluster->id_, cluster->centroid_,
                                                  cluster->box_min_, cluster->box_max_);
        }
    }
}

/**
 * @brief get the simple global frontiers
 *
 * @param frontiers the vector of frontiers
 */
void FrontierManager::getGlobalSimpleFrontiers(eigen_utils::Vec_Vec3d& frontiers) const
{
    std::shared_lock<std::shared_mutex> lock(global_frontier_map_mutex_);
    frontiers.clear();
    for (const auto& [index, frontier] : global_frontier_map_)
    {
        frontiers.push_back(frontier.pos_);
    }
}

/**
 * @brief get the simple local frontiers
 *
 * @param range the range of the frontiers
 * @param frontiers the vector of frontiers
 */
void FrontierManager::getLocalSimpleFrontiers(const double range,
                                              eigen_utils::Vec_Vec3d& frontiers) const
{
    std::shared_lock<std::shared_mutex> lock(global_frontier_map_mutex_);
    frontiers.clear();

    eigen_utils::Vec3d bmin = current_position_ - eigen_utils::Vec3d(range, range, range);
    eigen_utils::Vec3d bmax = current_position_ + eigen_utils::Vec3d(range, range, range);

    std::vector<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3iSet>> results =
        global_frontier_hash_grid_->boxSearch(bmin, bmax);

    for (const auto& result : results)
    {
        for (const auto& index : result.second)
        {
            auto iter = global_frontier_map_.find(index);
            if (iter != global_frontier_map_.end())
                frontiers.push_back(iter->second.pos_);
        }
    }
}

/**
 * @brief get the clustered frontiers from the frontier clusters info
 *
 * @param clustered_frontiers
 */
void FrontierManager::getGlobalClusteredFrontiers(
    std::vector<eigen_utils::Vec_Vec3d>& clustered_frontiers, const bool ignore_dormant) const
{
    std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
        clustered_frontiers.push_back(cluster->frontiers_);
    active_lock.unlock();

    if (!ignore_dormant)
    {
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
            clustered_frontiers.push_back(cluster->frontiers_);
    }
}

/**
 * @brief get the clustered centroids from the frontier clusters info
 *
 * @param clustered_centroids the vector of clustered centroids
 */
void FrontierManager::getGlobalClusteredCentroids(eigen_utils::Vec_Vec3d& clustered_centroids,
                                                  const bool ignore_dormant) const
{
    std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
        clustered_centroids.push_back(cluster->centroid_);
    active_lock.unlock();

    if (!ignore_dormant)
    {
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
            clustered_centroids.push_back(cluster->centroid_);
    }
}

/**
 * @brief get the clustered centroids from the frontier clusters info
 *
 * @param clustered_centroids the map of clustered index and centroids
 * @param ignore_dormant  whether to ignore the dormant clusters
 */
void FrontierManager::getGlobalClusteredCentroids(
    std::unordered_map<int, eigen_utils::Vec3d>& clustered_centroids,
    const bool ignore_dormant) const
{
    {
        std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
        clustered_centroids.reserve(clustered_centroids.size() +
                                    active_frontier_clusters_info_ptr_.size());
        for (const auto& cluster : active_frontier_clusters_info_ptr_)
        {
            clustered_centroids.emplace(cluster->id_, cluster->centroid_);
        }
    }

    if (!ignore_dormant)
    {
        int last_id = clustered_centroids.size();
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        clustered_centroids.reserve(clustered_centroids.size() +
                                    dormant_frontier_clusters_info_ptr_.size());
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
        {
            clustered_centroids.emplace(last_id + cluster->id_, cluster->centroid_);
        }
    }
}

/**
 * @brief get the clustered normals from the frontier clusters info
 *
 * @param clustered_normals the vector of clustered normals
 */
void FrontierManager::getGlobalClusteredNormals(eigen_utils::Vec_Vec3d& clustered_normals,
                                                const bool ignore_dormant) const
{
    {
        std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
        for (const auto& cluster : active_frontier_clusters_info_ptr_)
            clustered_normals.push_back(cluster->normal_);
    }

    if (!ignore_dormant)
    {
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
            clustered_normals.push_back(cluster->normal_);
    }
}

/**
 * @brief get the clustered viewpoints from the frontier clusters info
 *
 * @param clustered_viewpoints the vector of clustered viewpoints
 * @param ignore_dormant whether to ignore the dormant clusters
 */
void FrontierManager::getGlobalClusteredViewpoints(
    std::vector<eigen_utils::Vec_Vec3d>& clustered_viewpoints, const bool ignore_dormant) const
{
    std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
    {
        eigen_utils::Vec_Vec3d viewpoints;
        for (const auto& viewpoint : cluster->viewpoints_)
            viewpoints.push_back(viewpoint.pos_);
        clustered_viewpoints.push_back(viewpoints);
    }
    active_lock.unlock();

    if (!ignore_dormant)
    {
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
        {
            eigen_utils::Vec_Vec3d viewpoints;
            for (const auto& viewpoint : cluster->viewpoints_)
                viewpoints.push_back(viewpoint.pos_);
            clustered_viewpoints.push_back(viewpoints);
        }
    }
}

/**
 * @brief get the clustered top viewpoints from the frontier clusters info
 *
 * @param clustered_top_viewpoints the vector of clustered top viewpoints
 * @param ignore_dormant whether to ignore the dormant clusters
 */
void FrontierManager::getGlobalClusteredTopViewpoints(
    std::vector<std::pair<eigen_utils::Vec3d, double>>& clustered_top_viewpoints,
    const bool ignore_dormant) const
{
    {
        std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
        clustered_top_viewpoints.reserve(clustered_top_viewpoints.size() +
                                         active_frontier_clusters_info_ptr_.size());
        for (const auto& cluster : active_frontier_clusters_info_ptr_)
        {
            if (!cluster->viewpoints_.empty())
            {
                clustered_top_viewpoints.emplace_back(cluster->viewpoints_.front().pos_,
                                                      cluster->viewpoints_.front().visib_score_);
            }
        }
    }

    if (!ignore_dormant)
    {
        std::shared_lock<std::shared_mutex> dormant_lock(dormant_frontier_clusters_info_mutex_);
        clustered_top_viewpoints.reserve(clustered_top_viewpoints.size() +
                                         dormant_frontier_clusters_info_ptr_.size());
        for (const auto& cluster : dormant_frontier_clusters_info_ptr_)
        {
            if (!cluster->viewpoints_.empty())
            {
                clustered_top_viewpoints.emplace_back(cluster->viewpoints_.front().pos_,
                                                      cluster->viewpoints_.front().visib_score_);
            }
        }
    }
}

/**
 * @brief get the removed active clusters indices
 *
 * @param removed_cluster_indices
 */
void FrontierManager::getRemovedActiveClustersIndices(
    std::vector<int>& removed_cluster_indices) const
{
    removed_cluster_indices = removed_cluster_ids_;
}

void FrontierManager::getClusterFrontiers(const int& cluster_id,
                                          eigen_utils::Vec_Vec3d& frontiers) const
{
    std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);
    for (const auto& cluster : active_frontier_clusters_info_ptr_)
    {
        if (cluster->id_ == cluster_id)
        {
            frontiers = cluster->frontiers_;
            break;
        }
    }
}

/**
 * @brief Get viewpoints for the given cluster indices.
 *
 * This function retrieves the viewpoints associated with the specified cluster indices.
 * It sorts the cluster indices, locks the active frontier clusters info, and then iterates
 * through the clusters to find and collect the viewpoints for the given indices.
 *
 * @param cluster_indices A vector of cluster indices for which viewpoints are to be retrieved.
 * @param viewpoints A vector of vectors of 3D points where the viewpoints will be stored.
 * @param vp_num The number of viewpoints to retrieve for each cluster.
 */
void FrontierManager::getViewpointsForClusterIndices(
    const std::vector<int>& cluster_indices, std::vector<eigen_utils::Vec_Vec3d>& viewpoints,
    const int vp_num) const
{
    // Record the original position of each index
    std::vector<std::pair<int, int>> indexed_cluster_indices;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        indexed_cluster_indices.emplace_back(cluster_indices[i], i);
    }

    // Sort by index
    std::sort(indexed_cluster_indices.begin(), indexed_cluster_indices.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                  return a.first < b.first;
              });

    std::shared_lock<std::shared_mutex> active_lock(active_frontier_clusters_info_mutex_);

    auto cluster_it = active_frontier_clusters_info_ptr_.begin();
    auto index_it = indexed_cluster_indices.begin();

    // Temporary storage for viewpoints
    std::vector<eigen_utils::Vec_Vec3d> temp_viewpoints(cluster_indices.size());

    while (cluster_it != active_frontier_clusters_info_ptr_.end() &&
           index_it != indexed_cluster_indices.end())
    {
        if ((*cluster_it)->id_ < index_it->first)
        {
            ++cluster_it;
        }
        else if ((*cluster_it)->id_ > index_it->first)
        {
            ++index_it;
        }
        else
        {
            eigen_utils::Vec_Vec3d cluster_viewpoints;
            for (const auto& viewpoint : (*cluster_it)->viewpoints_)
            {
                cluster_viewpoints.push_back(viewpoint.pos_);

                if (vp_num > 0 && static_cast<int>(cluster_viewpoints.size()) >= vp_num)
                    break;
            }
            temp_viewpoints[index_it->second] = cluster_viewpoints;

            ++cluster_it;
            ++index_it;
        }
    }

    // Restore original order
    viewpoints = std::move(temp_viewpoints);
}

void FrontierManager::clusterFrontiersByRegionGrowing(
    std::list<std::shared_ptr<FrontierClusterInfo>>& local_clusters)
{
    const auto cluster_rg_start_time = std::chrono::steady_clock::now();
    static const std::array<eigen_utils::Vec3i, 26> kNeighborOffsets = {
        eigen_utils::Vec3i(-1, -1, -1), eigen_utils::Vec3i(-1, -1, 0),
        eigen_utils::Vec3i(-1, -1, 1),  eigen_utils::Vec3i(-1, 0, -1),
        eigen_utils::Vec3i(-1, 0, 0),   eigen_utils::Vec3i(-1, 0, 1),
        eigen_utils::Vec3i(-1, 1, -1),  eigen_utils::Vec3i(-1, 1, 0),
        eigen_utils::Vec3i(-1, 1, 1),   eigen_utils::Vec3i(0, -1, -1),
        eigen_utils::Vec3i(0, -1, 0),   eigen_utils::Vec3i(0, -1, 1),
        eigen_utils::Vec3i(0, 0, -1),   eigen_utils::Vec3i(0, 0, 1),
        eigen_utils::Vec3i(0, 1, -1),   eigen_utils::Vec3i(0, 1, 0),
        eigen_utils::Vec3i(0, 1, 1),    eigen_utils::Vec3i(1, -1, -1),
        eigen_utils::Vec3i(1, -1, 0),   eigen_utils::Vec3i(1, -1, 1),
        eigen_utils::Vec3i(1, 0, -1),   eigen_utils::Vec3i(1, 0, 0),
        eigen_utils::Vec3i(1, 0, 1),    eigen_utils::Vec3i(1, 1, -1),
        eigen_utils::Vec3i(1, 1, 0),    eigen_utils::Vec3i(1, 1, 1)};

    struct LocalFrontierNode
    {
        eigen_utils::Vec3i idx;
        eigen_utils::Vec3d pos;
        eigen_utils::Vec3d normal;
    };

    local_frontier_normals_map_.clear();

    // 1. batch compute normals
    const int nbr_d = 2;
    const auto& active_indices = local_frontier_grid_.getActiveIndices();
    computeAndStoreFrontierNormals(local_frontier_normals_map_, active_indices, nbr_d);

    // smooth normals
    eigen_utils::Vec3iMap<eigen_utils::Vec3d> smooth_normals_map;
    smoothFrontiersNormals(local_frontier_normals_map_, smooth_normals_map, 3);
    local_frontier_normals_map_.swap(smooth_normals_map);

    const double cos_threshold = std::cos(M_PI / 3);
    const auto& frontier_normals_map = local_frontier_normals_map_;

    std::vector<LocalFrontierNode> frontier_nodes;
    frontier_nodes.reserve(active_indices.size());
    eigen_utils::Vec3iMap<int> node_id_by_index;
    node_id_by_index.reserve(active_indices.size());

    for (const auto& idx : active_indices)
    {
        auto normal_it = frontier_normals_map.find(idx);
        if (normal_it == frontier_normals_map.end())
            continue;

        LocalFrontierNode node;
        node.idx = idx;
        sdf_map_->indexToPos(idx, node.pos);
        node.normal = normal_it->second.normalized();

        const int node_id = static_cast<int>(frontier_nodes.size());
        frontier_nodes.push_back(std::move(node));
        node_id_by_index.emplace(idx, node_id);
    }

    if (frontier_nodes.empty())
        return;

    auto getNodeId = [&](const eigen_utils::Vec3i& idx) -> int {
        auto node_it = node_id_by_index.find(idx);
        return node_it == node_id_by_index.end() ? -1 : node_it->second;
    };

    std::vector<uint8_t> visited(frontier_nodes.size(), 0);
    std::vector<int> bfs_queue;
    bfs_queue.reserve(frontier_nodes.size());

    for (size_t start_node_id = 0; start_node_id < frontier_nodes.size(); ++start_node_id)
    {
        if (visited[start_node_id] != 0)
            continue;

        eigen_utils::Vec_Vec3d cluster, normals;
        cluster.reserve(100);
        normals.reserve(100);

        eigen_utils::Vec3d centroid = eigen_utils::Vec3d::Zero();
        eigen_utils::Vec3d min_coord, max_coord;
        const LocalFrontierNode* start_node = &frontier_nodes[start_node_id];

        visited[start_node_id] = 1;
        bfs_queue.clear();
        bfs_queue.push_back(static_cast<int>(start_node_id));

        cluster.push_back(start_node->pos);
        centroid = start_node->pos;
        min_coord = max_coord = start_node->pos;

        eigen_utils::Vec3d avg_normal = start_node->normal;
        normals.push_back(avg_normal);

        // BFS expansion
        size_t queue_head = 0;
        while (queue_head < bfs_queue.size())
        {
            const LocalFrontierNode& current_node = frontier_nodes[bfs_queue[queue_head++]];

            for (const auto& offset : kNeighborOffsets)
            {
                const eigen_utils::Vec3i nbr = current_node.idx + offset;
                const int nbr_node_id = getNodeId(nbr);
                if (nbr_node_id < 0)
                    continue;
                if (visited[nbr_node_id] != 0)
                    continue;

                const LocalFrontierNode& nbr_node = frontier_nodes[nbr_node_id];
                const eigen_utils::Vec3d& xyz_eigen = nbr_node.pos;
                const eigen_utils::Vec3d& cur_normal = nbr_node.normal;

                if (avg_normal.dot(cur_normal) < cos_threshold)
                    continue;

                const eigen_utils::Vec3d candidate_min = min_coord.cwiseMin(xyz_eigen);
                const eigen_utils::Vec3d candidate_max = max_coord.cwiseMax(xyz_eigen);

                double max_dist = (candidate_max - candidate_min).cwiseAbs().maxCoeff();
                if (max_dist > frontier_params_.max_cluster_radius_ &&
                    static_cast<int>(cluster.size()) >= frontier_params_.min_cluster_voxel_num_)
                {
                    // create cluster
                    auto cluster_ptr = std::make_shared<FrontierClusterInfo>();
                    cluster_ptr->frontiers_.swap(cluster);
                    cluster_ptr->normals_.swap(normals);
                    cluster_ptr->centroid_ =
                        centroid / static_cast<double>(cluster_ptr->frontiers_.size());
                    cluster_ptr->box_min_ = min_coord;
                    cluster_ptr->box_max_ = max_coord;
                    cluster_ptr->normal_ = avg_normal;
                    cluster_ptr->initial_frontiers_count_ = cluster_ptr->frontiers_.size();
                    local_clusters.push_back(cluster_ptr);

                    visited[nbr_node_id] = 1;

                    // clear queue and start a new cluster
                    bfs_queue.clear();
                    bfs_queue.push_back(nbr_node_id);
                    queue_head = 0;

                    cluster.clear();
                    normals.clear();
                    cluster.push_back(xyz_eigen);
                    normals.push_back(cur_normal);
                    centroid = xyz_eigen;
                    min_coord = max_coord = xyz_eigen;
                    avg_normal = cur_normal;
                    break;
                }

                visited[nbr_node_id] = 1;
                bfs_queue.push_back(nbr_node_id);
                cluster.push_back(xyz_eigen);
                normals.push_back(cur_normal);
                centroid += xyz_eigen;
                min_coord = candidate_min;
                max_coord = candidate_max;
                avg_normal = (avg_normal * (cluster.size() - 1) + cur_normal).normalized();
            }
        }

        if (static_cast<int>(cluster.size()) >= frontier_params_.min_cluster_voxel_num_)
        {
            auto cluster_ptr = std::make_shared<FrontierClusterInfo>();
            cluster_ptr->frontiers_.swap(cluster);
            cluster_ptr->normals_.swap(normals);
            cluster_ptr->centroid_ = centroid / static_cast<double>(cluster_ptr->frontiers_.size());
            cluster_ptr->box_min_ = min_coord;
            cluster_ptr->box_max_ = max_coord;
            cluster_ptr->normal_ = avg_normal;
            cluster_ptr->initial_frontiers_count_ = cluster_ptr->frontiers_.size();
            local_clusters.push_back(cluster_ptr);
        }
    }

    const auto cluster_rg_end_time = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(cluster_rg_end_time - cluster_rg_start_time)
            .count();
    ROS_DEBUG_STREAM("clusterFrontiersByRegionGrowing finished in "
                     << elapsed_ms << " ms, active_indices=" << active_indices.size()
                     << ", frontier_nodes=" << frontier_nodes.size()
                     << ", generated_clusters=" << local_clusters.size());
}

/**
 * @brief Update the information of a frontier cluster.
 *
 * This function updates the centroid, bounding box, and normal of the given frontier cluster.
 * It also determines whether the cluster is dormant based on the orientation of its normal.
 *
 * @param cluster The cluster of frontiers that needs to be updated.
 *                The 'frontiers_' attribute of the cluster should be updated before calling
 * this function.
 */
void FrontierManager::updateClustersInfo(FrontierClusterInfo& cluster)
{
    if (cluster.frontiers_.empty())
        return;

    const size_t n = cluster.frontiers_.size();

    Eigen::MatrixXd data =
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
            cluster.frontiers_[0].data(), n, 3);

    cluster.centroid_ = data.colwise().mean().transpose();
    cluster.box_min_ = data.colwise().minCoeff().transpose();
    cluster.box_max_ = data.colwise().maxCoeff().transpose();

    // Center the data around the centroid
    data.rowwise() -= cluster.centroid_.transpose();

    // Compute the normal of the cluster using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(data, Eigen::ComputeFullV);
    int svd_rank = svd.rank();
    if (svd_rank > 1)
    {
        eigen_utils::Vec3d v1 = svd.matrixV().col(2);
        cluster.normal_ = v1.dot(current_position_ - cluster.centroid_) > 0 ? v1 : -v1;
        double pitch =
            std::atan2(cluster.normal_.z(), std::hypot(cluster.normal_.x(), cluster.normal_.y()));
        cluster.is_dormant_ = (std::abs(pitch) > M_PI / 3);
    }
    else // Line cluster
    {
        // set the cluster as dormant
        cluster.is_dormant_ = true;
    }
}

/**
 * @brief Check if an object is within the field of view
 *
 * This function determines whether a given object is within the field of view of the robot
 * based on the robot's current position and the object's position. It calculates the angle
 * between the robot's position and the object and checks if it falls within the specified
 * field of view angles.
 *
 * @param pos The current position of the robot
 * @param obj The position of the object to check
 * @return true If the object is within the field of view
 * @return false If the object is not within the field of view
 */
bool FrontierManager::isInFov(const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& obj) const
{
    eigen_utils::Vec3d diff = obj - pos;                                // 3D vector difference
    double norm = std::sqrt(diff.x() * diff.x() + diff.y() * diff.y()); // horizontal distance
    if (norm == 0.0)
        return false;

    double tan_theta = diff.z() / norm; // compute tangent of the elevation angle

    // compute tangents of FOV upper and lower limits
    double lower_tan_theta = std::tan(frontier_params_.lower_fov_angle_ * M_PI / 180.0);
    double upper_tan_theta = std::tan(frontier_params_.upper_fov_angle_ * M_PI / 180.0);

    return tan_theta > lower_tan_theta && tan_theta < upper_tan_theta;
}

/**
 * @brief check if the neighbor of the cell in sdf_map_ is unknown
 *
 * @param index the index of the cell
 * @param dim the dimension of the check
 * @return true
 * @return false
 */
bool FrontierManager::isNeighborUnknown(const eigen_utils::Vec3i& index, const int& dim) const
{
    eigen_utils::Vec_Vec3i nbrs;
    if (dim == 2)
        nbrs = process_utils::ProcessUtils::fourNeighbors(index);
    else if (dim == 3)
        nbrs = process_utils::ProcessUtils::sixNeighbors(index);

    for (const auto& nbr : nbrs)
    {
        if (sdf_map_->isUnknown(nbr))
            return true;
    }

    return false;
}

/**
 * @brief Check if a given cell is safe based on minimum clearance.
 *
 * This function checks whether a given cell is safe by verifying its occupancy status and the
 * occupancy status of its neighboring cells within a specified clearance distance. A cell is
 * considered safe if it and its neighbors are free of obstacles.
 *
 * @param idx The index of the cell to be checked.
 * @param min_clearance The minimum clearance distance required for the cell to be considered
 * safe.
 * @return true If the cell is safe.
 * @return false If the cell is not safe.
 */
bool FrontierManager::isSafe(const eigen_utils::Vec3i& idx, const double min_clearance) const
{
    // Check if the current cell is occupied
    if (!sdf_map_->isInflatedFree(idx))
        return false;

    // Calculate the number of cells to check based on the minimum clearance
    double check_dist = min_clearance > 0 ? min_clearance : sdf_map_->getResolution();
    int check_num = std::ceil(check_dist / sdf_map_->getResolution());

    // Iterate over the neighboring cells within the clearance distance
    for (int i = -check_num; i <= check_num; ++i)
        for (int j = -check_num; j <= check_num; ++j)
            for (int k = -check_num; k <= check_num; ++k)
            {
                eigen_utils::Vec3i nbr = idx + eigen_utils::Vec3i(i, j, k);
                if (!sdf_map_->isInflatedFree(nbr))
                    return false;
            }

    return true;
}

/**
 * @brief Check if a given position is safe
 *
 * This function checks if the given position is safe by converting
 * the position to an index in the SDF map and then checking the safety
 * status at that index with a specified minimum clearance.
 *
 * @param pos The position to check
 * @param min_clearance The minimum clearance required for the position to be considered safe
 * (default is -1.0)
 * @return true if the position is safe
 * @return false if the position is not safe
 */
bool FrontierManager::isSafe(const eigen_utils::Vec3d& pos, const double min_clearance) const
{
    eigen_utils::Vec3i idx;
    sdf_map_->posToIndex(pos, idx);
    return isSafe(idx, min_clearance);
}

/**
 * @brief check if the cell in sdf_map_ is frontier
 *
 * @param index the index of the cell
 * @param dim the dimension of the check
 * @return true
 * @return false
 */
bool FrontierManager::isFrontier(const eigen_utils::Vec3i& index, const int& dim)
{
    return sdf_map_->isInflatedFree(index) && isNeighborUnknown(index, dim);
}

/**
 * @brief check if the cell in sdf_map_ is frontier
 *
 * @param pos the position of the cell
 * @param dim the dimension of the check
 * @return true
 * @return false
 */
bool FrontierManager::isFrontier(const eigen_utils::Vec3d& pos, const int& dim)
{
    eigen_utils::Vec3i index;
    sdf_map_->posToIndex(pos, index);
    return sdf_map_->isInflatedFree(index) && isNeighborUnknown(index, dim);
}

/**
 * @brief Check if any cluster is covered.
 *
 * This function checks if the active and dormant frontier clusters have enough changes,
 * which indicates that the frontiers are covered.
 *
 * @return true If any of the active or dormant frontier clusters is covered.
 * @return false Otherwise.
 */
bool FrontierManager::isFrontierCovered()
{
    eigen_utils::Vec3d update_min, update_max;
    sdf_map_->getUpdatedBox(update_min, update_max, false);

    auto checkChanges =
        [&](const std::list<std::shared_ptr<FrontierClusterInfo>>& clusters) -> bool {
        for (const auto& cluster : clusters)
        {
            if (!process_utils::ProcessUtils::isOverlapped(cluster->box_min_, cluster->box_max_,
                                                           update_min, update_max))
                continue;

            const int change_thresh = static_cast<int>(frontier_params_.min_view_finish_fraction_ *
                                                       cluster->frontiers_.size());
            int change_num = 0;
            for (const eigen_utils::Vec3d& cell : cluster->frontiers_)
            {
                if (!isFrontier(cell, 3) && ++change_num >= change_thresh)
                    return true;
            }
        }
        return false;
    };

    return checkChanges(active_frontier_clusters_info_ptr_) ||
           checkChanges(dormant_frontier_clusters_info_ptr_);
}

/**
 * @brief Parallel process a vector of inputs using multiple threads.
 *
 * This function takes a vector of input data, applies a processing function to each element
 * in parallel using multiple threads, and stores the results in an output vector.
 *
 * @tparam InputType The type of the input vector elements.
 * @tparam OutputType The type of the output vector elements.
 * @tparam Func The type of the processing function.
 * @param input_vec The input vector to process.
 * @param output_vec The output vector to store the results.
 * @param process_func The processing function to apply to each element.
 * @param chunk_size The size of the chunks to process in parallel.
 * @param num_threads The number of threads to use for parallel processing.
 */
template <typename InputType, typename OutputType, typename Func>
void FrontierManager::parallelProcess(const std::vector<InputType>& input_vec,
                                      std::vector<OutputType>& output_vec, Func process_func,
                                      int chunk_size, int num_threads)
{
    output_vec.resize(input_vec.size());

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic, chunk_size)                                             \
    num_threads(std::min(num_threads, omp_get_max_threads()))
#endif
    for (size_t i = 0; i < input_vec.size(); ++i)
    {
        output_vec[i] = process_func(input_vec[i]);
    }
}

} // namespace map_process
