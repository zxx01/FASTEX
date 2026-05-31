/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-30 20:57:41
 * @Description:
 */

#include <omp.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "map_process/grid/dynamic_expanding_grid.h"
#include "plan_env/edt_environment.h"
#include "time_utils/time_utils.h"
#include "utils/HPR.h"
#include <vis_utils/marker_utils.h>

namespace map_process
{
/**
 * @brief Construct a new DynamicExpandingGrid object.
 *
 * @param initial_resolution the initial grid resolution
 */
DynamicExpandingGrid::DynamicExpandingGrid(
    ros::NodeHandle& nh, const std::shared_ptr<fast_planner::EDTEnvironment>& edt_env)
{
    // Initialize some basic modules: the sdf map, relevant graph and path finder
    sdf_map_ = edt_env->sdf_map_;
    data_.relevant_graph_ = std::make_shared<map_process::IkdTreePlanGraph<void, void>>();
    data_.relevant_graph_->enableTemporaryEdges();

    // Load DynamicExpandingGrid params from ROS
    loadParamsFromROS(nh);
    params_.is_min_max_set_ = !params_.min_set_.isApprox(params_.max_set_);

    // Get the origin offset from the sdf map to occupancy_grid_ in GridData
    eigen_utils::Vec3d origin(0., 0., 0.);
    eigen_utils::Vec3d origin_xyz;
    eigen_utils::Vec3i origin_idx;
    sdf_map_->posToIndex(origin, origin_idx);
    sdf_map_->indexToPos(origin_idx, origin_xyz, 0.0);
    double cell_resolution = sdf_map_->getResolution();
    openvdb::Vec3d cell_offset = toVdb(origin_xyz);

    // Initialize the dynamic expanding grid.
    eigen_utils::Vec3d grid_resolution = params_.initial_resolution_;

    if (params_.is_min_max_set_)
    {
        data_.grid_ = std::make_unique<SingleLevelGrid>(
            grid_resolution, cell_resolution, cell_offset, params_.min_set_, params_.max_set_);
    }
    else
    {
        data_.grid_ =
            std::make_unique<SingleLevelGrid>(grid_resolution, cell_resolution, cell_offset);
    }
    data_.grid_->setSDFMap(sdf_map_);

    // Initialize the active grid
    data_.active_grid_ = openvdb::BoolGrid::create(false);
    vdb_utils::VDBUtil::setVoxelSize(*data_.active_grid_, toVdb(grid_resolution));

    // Initialize the key position kdtree
    data_.key_pos_kdtree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>(0.3, 0.6, 0.2);
    data_.exploring_unknown_zone_kdtree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>(0.3, 0.6, 0.2);

    // Debug
    visible_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning_vis/visible_cloud", 100);
    label_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planning_vis/label_cloud", 100);
    active_grid_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("/planning_vis/active_grid", 100);

    first_grid_centroids_pub_ =
        nh.advertise<visualization_msgs::Marker>("/planning_vis/first_grid_centroids", 100);
    first_grid_neighbor_centroids_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/planning_vis/first_grid_neighbor_centroids", 100);
    points_group_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/points_group", 100);

    cumulative_virtual_cloud_all_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    cumulative_vis_virtual_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    all_virtual_cloud_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>("/planning_vis/all_virtual_cloud", 100);
    all_vis_virtual_cloud_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>("/planning_vis/all_vis_virtual_cloud", 100);
}

/**
 * @brief Load DynamicExpandingGrid params from ROS
 * @param nh  the node handle of ROS
 *
 */
void DynamicExpandingGrid::loadParamsFromROS(const ros::NodeHandle& nh)
{
    bool is_param_load = true;

    is_param_load &=
        nh.param("DynamicExpandingGrid/frame_id", params_.frame_id_, std::string("world"));
    is_param_load &= nh.param("DynamicExpandingGrid/initial_grid_resolution_x",
                              params_.initial_resolution_[0], 16.0);
    is_param_load &= nh.param("DynamicExpandingGrid/initial_grid_resolution_y",
                              params_.initial_resolution_[1], 16.0);
    is_param_load &= nh.param("DynamicExpandingGrid/initial_grid_resolution_z",
                              params_.initial_resolution_[2], 16.0);
    is_param_load &= nh.param("DynamicExpandingGrid/min_set_x", params_.min_set_[0], -100.0);
    is_param_load &= nh.param("DynamicExpandingGrid/min_set_y", params_.min_set_[1], -100.0);
    is_param_load &= nh.param("DynamicExpandingGrid/min_set_z", params_.min_set_[2], -0.5);
    is_param_load &= nh.param("DynamicExpandingGrid/max_set_x", params_.max_set_[0], 100.0);
    is_param_load &= nh.param("DynamicExpandingGrid/max_set_y", params_.max_set_[1], 100.0);
    is_param_load &= nh.param("DynamicExpandingGrid/max_set_z", params_.max_set_[2], 20.0);

    is_param_load &= nh.param("exploration/straight_max_dist", params_.max_straight_dist_, 12.0);

    if (!is_param_load)
        ROS_ERROR("DynamicExpandingGrid params load failed!");
}

/**
 * @brief Query the current active frontier-cluster centroids.
 * @param cluster_centroids Output map from cluster id to centroid position.
 */
void DynamicExpandingGrid::collectFrontierClusterCentroids(
    std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const
{
    cluster_centroids.clear();
    frontier_manager_->getGlobalClusteredCentroids(cluster_centroids, true);
}

/**
 * @brief Convert changed-cell addresses into world coordinates and append them to the buffer.
 * @param changed_cells Changed-cell addresses reported by the map.
 * @param changed_cells_world In-out buffer storing world-space changed cells.
 */
void DynamicExpandingGrid::appendChangedCells(
    const std::vector<int>& changed_cells, std::vector<openvdb::Vec3d>& changed_cells_world) const
{
    changed_cells_world.reserve(changed_cells_world.size() + changed_cells.size());
    for (const int cell : changed_cells)
    {
        changed_cells_world.push_back(toVdb(sdf_map_->AddressToPos(cell)));
    }
}

/**
 * @brief Cache the world-space boxes of grids newly extended in the current update.
 * @param extended_grid_indices Grid indices that were newly activated by this frame.
 */
void DynamicExpandingGrid::recordNewExtendGridBoxes(
    const eigen_utils::Vec3iSet& extended_grid_indices)
{
    if (extended_grid_indices.empty())
        return;

    const auto& grid = *data_.grid_;
    for (const auto& index : extended_grid_indices)
    {
        openvdb::BBoxd bbox;
        grid.getGridBBoxd(toVdbCoord(index), bbox);
        data_.new_extend_grid_boxes_.emplace_back(fromVdb(bbox.min()), fromVdb(bbox.max()));
    }
}

/**
 * @brief Push this frame's cell, frontier, and extension updates into the single-level grid.
 * @param changed_cells_world In-out buffer of changed cells in world coordinates.
 * @param cluster_centroids Frontier-cluster centroids associated with the current frame.
 * @param extend_grid_coords Newly extended grid coordinates in OpenVDB index space.
 * @param update_min Minimum corner of the update box.
 * @param update_max Maximum corner of the update box.
 */
void DynamicExpandingGrid::syncSingleLevelGridData(
    std::vector<openvdb::Vec3d>& changed_cells_world,
    std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
    const std::vector<openvdb::math::Coord>& extend_grid_coords,
    const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max)
{
    auto& grid = *data_.grid_;
    grid.extendGridDataByGridIndices(extend_grid_coords, true);
    grid.getSingleLevelGridRange(data_.min_now_, data_.max_now_);

    eigen_utils::Vec_Vec3i extra_coords;
    extra_coords.reserve(extend_grid_coords.size());
    for (const auto& coord : extend_grid_coords)
        extra_coords.emplace_back(fromVdbCoord(coord));

    grid.updateGridData(changed_cells_world, cluster_centroids, data_.label_clouds_uncovered_ratio_,
                        extra_coords, update_min, update_max);
}

/**
 * @brief Refresh cached centroid-index and centroid-state mappings for relevant grids.
 * @param grid_ids Relevant grid indices to export into cached metadata maps.
 */
void DynamicExpandingGrid::populateRelevantGridMetadata(const eigen_utils::Vec3iSet& grid_ids)
{
    GRID_EXPLORE_STATE state;
    eigen_utils::Vec_Vec3d centroids;

    auto& slgrid = *data_.grid_;
    for (const auto& id : grid_ids)
    {
        slgrid.getGridExploreState(id, state);
        slgrid.getGridCentroids(id, centroids);

        for (const auto& centroid : centroids)
        {
            data_.grid_centroids_indices_[centroid] = id;
            data_.vertices_explore_state_[centroid] = state;
        }
        data_.grid_indices_centroids_[id] = centroids;
    }
}

/**
 * @brief Update exploration-related per-grid data for all grids touched in this frame.
 * @param update_indices Grid indices scheduled for exploration-info refresh.
 */
void DynamicExpandingGrid::updateGridExplorationInfoBatch(
    const eigen_utils::Vec_Vec3i& update_indices)
{
#ifdef USE_OPENMP
    const int max_threads = omp_get_max_threads();
#pragma omp parallel for num_threads(max_threads) schedule(dynamic)
#endif // USE_OPENMP
    for (size_t i = 0; i < update_indices.size(); ++i)
    {
        data_.grid_->updateGridExplorationInfo(update_indices[i]);
    }
}

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
void DynamicExpandingGrid::collectChangedRelevantGridCentroids(
    const eigen_utils::Vec3iSet& removed_grid_indices,
    const eigen_utils::Vec3iSet& new_grid_indices,
    const eigen_utils::Vec3iSet& unchanged_grid_indices,
    const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& last_grid_centroids,
    const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max,
    eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& removed_centroids,
    eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& new_centroids) const
{
    auto selectCentroids = [](const eigen_utils::Vec3iSet& grid_indices,
                              const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& source_centroids,
                              eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& target_centroids) {
        for (const auto& grid_index : grid_indices)
            target_centroids[grid_index] = source_centroids.at(grid_index);
    };

    selectCentroids(removed_grid_indices, last_grid_centroids, removed_centroids);
    selectCentroids(new_grid_indices, data_.grid_indices_centroids_, new_centroids);

    eigen_utils::Vec3d grid_min_box, grid_max_box;
    for (const auto& grid_index : unchanged_grid_indices)
    {
        data_.grid_->getGridMinMaxBox(grid_index, grid_min_box, grid_max_box);
        if (!process_utils::ProcessUtils::isOverlapped(grid_min_box, grid_max_box, update_min,
                                                       update_max))
            continue;

        removed_centroids[grid_index] = last_grid_centroids.at(grid_index);
        new_centroids[grid_index] = data_.grid_indices_centroids_.at(grid_index);
    }
}

/**
 * @brief Rebuild the cached relevant-graph vertex attributes from current centroid metadata.
 */
void DynamicExpandingGrid::rebuildRelevantGraphVertexCache()
{
    data_.relevant_graph_vertices_.clear();
    for (const auto& [grid_index, centroids] : data_.grid_indices_centroids_)
    {
        for (const auto& centroid : centroids)
        {
            const auto state = data_.vertices_explore_state_.at(centroid);
            data_.relevant_graph_vertices_.emplace_back(state, grid_index, centroid);
        }
    }
}

/**
 * @brief Collect the world-space boxes of updated grids for roadmap refresh.
 * @param update_grid_indices Grid indices touched in the current frame.
 * @param refresh_boxes Output refresh boxes covering those grids.
 */
void DynamicExpandingGrid::collectRefreshBoxes(
    const eigen_utils::Vec_Vec3i& update_grid_indices,
    std::vector<process_utils::CubeBox>& refresh_boxes) const
{
    refresh_boxes.clear();
    refresh_boxes.reserve(update_grid_indices.size());

    eigen_utils::Vec3d box_min, box_max;
    for (const auto& grid_index : update_grid_indices)
    {
        data_.grid_->getGridMinMaxBox(grid_index, box_min, box_max);
        refresh_boxes.emplace_back(box_min, box_max);
    }
}

/**
 * @brief Collect unknown-zone centers and their linked cluster top viewpoints.
 * @param update_grid_indices Grid indices touched in the current frame.
 * @param zone_to_all_cluster_top_vps Output mapping from zone center to all neighbor-cluster top
 * viewpoints.
 * @param zone_to_nearest_cluster_top_vp Output mapping from zone center to nearest
 * neighbor-cluster top viewpoint.
 */
void DynamicExpandingGrid::collectUnknownZoneRoadmapTargets(
    const eigen_utils::Vec_Vec3i& update_grid_indices,
    ZoneToAllClusterTopVps& zone_to_all_cluster_top_vps,
    ZoneToNearestClusterTopVp& zone_to_nearest_cluster_top_vp) const
{
    zone_to_all_cluster_top_vps.clear();
    zone_to_nearest_cluster_top_vp.clear();

    std::vector<std::pair<eigen_utils::Vec3d, double>> clustered_top_viewpoints;
    frontier_manager_->getGlobalClusteredTopViewpoints(clustered_top_viewpoints, true);

    const SingleLevelGrid& slgrid = *data_.grid_;
    GRID_EXPLORE_STATE state;
    eigen_utils::Vec_Vec3d zone_centroids;
    std::vector<std::unordered_set<int>> zone_neighbor_cluster_ids;

    for (const auto& grid_index : update_grid_indices)
    {
        slgrid.getGridExploreState(grid_index, state);
        if (state == GRID_EXPLORE_STATE::EXPLORED)
            continue;

        slgrid.getGridCentroids(grid_index, zone_centroids);
        slgrid.getGridUnknownZoneClusterIds(grid_index, zone_neighbor_cluster_ids);

        const bool has_neighbor_cluster = !zone_neighbor_cluster_ids.empty();
        for (size_t i = 0; i < zone_centroids.size(); ++i)
        {
            const auto& zone_centroid = zone_centroids[i];
            if (!has_neighbor_cluster)
            {
                zone_to_nearest_cluster_top_vp.emplace_back(zone_centroid, std::nullopt);
                zone_to_all_cluster_top_vps.emplace_back(zone_centroid, std::nullopt);
                continue;
            }

            const std::unordered_set<int>& z_nbr_ids = zone_neighbor_cluster_ids[i];
            eigen_utils::Vec_Vec3d cluster_top_vps;
            cluster_top_vps.reserve(z_nbr_ids.size());
            for (const int cluster_id : z_nbr_ids)
                cluster_top_vps.push_back(clustered_top_viewpoints[cluster_id].first);

            double min_distance = std::numeric_limits<double>::max();
            eigen_utils::Vec3d nearest_cluster_top_vp;
            for (const auto& top_vp : cluster_top_vps)
            {
                const double distance = (zone_centroid - top_vp).squaredNorm();
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_cluster_top_vp = top_vp;
                }
            }

            zone_to_nearest_cluster_top_vp.emplace_back(zone_centroid, nearest_cluster_top_vp);
            zone_to_all_cluster_top_vps.emplace_back(zone_centroid, std::move(cluster_top_vps));
        }
    }
}

/**
 * @brief Remove outdated unknown-zone vertices from roadmap, history graph, and kd-tree.
 * @param refresh_boxes Boxes whose contents must be removed before reinsertion.
 */
void DynamicExpandingGrid::removeStaleUnknownZoneVertices(
    const std::vector<process_utils::CubeBox>& refresh_boxes)
{
    BoxPointType search_box;
    KD_TREE<pcl::PointXYZ>::PointVector delete_points, points_result;

    for (const auto& box : refresh_boxes)
    {
        for (int i = 0; i < 3; ++i)
        {
            search_box.vertex_min[i] = box.min_[i];
            search_box.vertex_max[i] = box.max_[i];
        }

        data_.exploring_unknown_zone_kdtree_->Box_Search(search_box, points_result);
        delete_points.insert(delete_points.end(), points_result.begin(), points_result.end());
    }

    if (delete_points.empty())
        return;

    for (const auto& point : delete_points)
    {
        whole_state_road_map_->deleteVertex(
            whole_state_road_map_->getVertexId(eigen_utils::Vec3f(point.x, point.y, point.z)));
        history_pos_graph_->deleteVertex(
            history_pos_graph_->getVertexId(eigen_utils::Vec3f(point.x, point.y, point.z)));
    }

    data_.exploring_unknown_zone_kdtree_->Delete_Points(delete_points);
}

/**
 * @brief Insert unknown-zone vertices and edges into the workspace roadmap.
 * @param zone_to_nearest_cluster_top_vp Mapping from zone center to nearest cluster top viewpoint.
 * @param added_zone_points Output point set newly inserted into the unknown-zone kd-tree.
 */
void DynamicExpandingGrid::insertUnknownZoneVerticesToRoadMap(
    const ZoneToNearestClusterTopVp& zone_to_nearest_cluster_top_vp,
    KD_TREE<pcl::PointXYZ>::PointVector& added_zone_points)
{
    added_zone_points.clear();

    eigen_utils::Vec3d nearest_cluster_tvp_d;
    eigen_utils::Vec3f zone_centroid_f, nearest_cluster_tvp_f;
    eigen_utils::Vec_Vec3f zone_near_pts_on_graph;
    std::vector<int> zone_near_pt_ids_on_graph;
    std::vector<eigen_utils::Vec_Vec3f> paths_temp;
    std::vector<double> path_costs;
    const double neighbors_search_range = 8.0;

    for (const auto& [zone_centroid_d, nearest_cluster_tvp] : zone_to_nearest_cluster_top_vp)
    {
        zone_centroid_f = zone_centroid_d.cast<float>();

        int zone_centroid_vertex_id = whole_state_road_map_->getVertexId(zone_centroid_f);
        bool zone_success = zone_centroid_vertex_id >= 0;
        const bool zone_already_exist = zone_success;

        if (!zone_success)
        {
            zone_success = whole_state_road_map_->findNearestValidPointsInGraph(
                zone_centroid_f, true, zone_near_pts_on_graph, zone_near_pt_ids_on_graph,
                paths_temp, path_costs, neighbors_search_range);
        }

        if (!zone_success)
            continue;

        if (!zone_already_exist)
        {
            map_process::WSRoadMap::VertexExtraDataType vertex_extra_data(
                map_process::WSRoadMap::VertexExtraDataType::VertexSource::SAMPLE,
                map_process::WSRoadMap::VertexExtraDataType::VertexState::UNKNOWN,
                map_process::WSRoadMap::VertexExtraDataType::VertexType::REGION_CENTER);
            map_process::WSRoadMap::VertexType vertex(zone_centroid_f, true, vertex_extra_data);

            std::tie(zone_centroid_vertex_id, std::ignore) =
                whole_state_road_map_->insertVertex(vertex);

            map_process::WSRoadMap::EdgeExtraDataType edge_extra_data(process_utils::CubeBox(),
                                                                      false, false);
            for (size_t i = 0; i < zone_near_pt_ids_on_graph.size(); ++i)
            {
                map_process::WSRoadMap::EdgeType edge(path_costs[i], true, edge_extra_data);
                whole_state_road_map_->addTwoWayEdge(zone_centroid_vertex_id,
                                                     zone_near_pt_ids_on_graph[i], edge);
            }
        }

        if (nearest_cluster_tvp.has_value())
        {
            nearest_cluster_tvp_d = nearest_cluster_tvp.value();
            nearest_cluster_tvp_f = nearest_cluster_tvp_d.cast<float>();
            const int cluster_tvp_id = whole_state_road_map_->getVertexId(nearest_cluster_tvp_f);

            if (cluster_tvp_id != zone_centroid_vertex_id)
            {
                eigen_utils::Vec_Vec3d pathd;
                double dist;
                map_process::PATH_SEARCH_RESULT result = path_searcher_->searchFinePath(
                    zone_centroid_d, nearest_cluster_tvp_d, pathd, dist, -1.0, true);

                if (result == map_process::PATH_SEARCH_RESULT::FAIL)
                    dist = (zone_centroid_d - nearest_cluster_tvp_d).norm() * 2;

                map_process::WSRoadMap::EdgeExtraDataType edge_extra_data(process_utils::CubeBox(),
                                                                          false, false);
                map_process::WSRoadMap::EdgeType edge(dist, true, edge_extra_data);
                whole_state_road_map_->addTwoWayEdge(zone_centroid_vertex_id, cluster_tvp_id, edge);
            }
        }

        if (!zone_already_exist)
            added_zone_points.push_back(
                pcl::PointXYZ(zone_centroid_f.x(), zone_centroid_f.y(), zone_centroid_f.z()));
    }
}

/**
 * @brief Insert unknown-zone to cluster-top-vp connectivity into the history graph.
 * @param zone_to_all_cluster_top_vps Mapping from zone center to all linked cluster top
 * viewpoints.
 */
void DynamicExpandingGrid::insertUnknownZoneVerticesToHistoryGraph(
    const ZoneToAllClusterTopVps& zone_to_all_cluster_top_vps)
{
    for (const auto& [zone_centroid_d, cluster_tvp] : zone_to_all_cluster_top_vps)
    {
        if (!cluster_tvp.has_value())
            continue;

        const eigen_utils::Vec3f zone_centroid_f = zone_centroid_d.cast<float>();
        if (whole_state_road_map_->getVertexId(zone_centroid_f) < 0)
        {
            ROS_ERROR_STREAM("The unknown zone center is not added to the roadmap.");
            continue;
        }

        auto [zone_id, success] = history_pos_graph_->insertVertex(
            map_process::HistoryPosGraph::GraphVertexType(zone_centroid_f, true));

        for (const auto& cluster_tvp_d : cluster_tvp.value())
        {
            const eigen_utils::Vec3f cluster_tvp_f = cluster_tvp_d.cast<float>();
            const int vp_id = history_pos_graph_->getVertexId(cluster_tvp_f);
            if (vp_id == zone_id)
                continue;

            history_pos_graph_->addTwoWayEdge(zone_id, vp_id,
                                              map_process::HistoryPosGraph::GraphEdgeType(
                                                  (zone_centroid_d - cluster_tvp_d).norm(), true));
        }
    }
}

/**
 * @brief Update the kd-tree that indexes active unknown-zone centers.
 * @param added_zone_points Newly inserted unknown-zone points.
 */
void DynamicExpandingGrid::updateUnknownZoneKdTree(
    KD_TREE<pcl::PointXYZ>::PointVector& added_zone_points)
{
    if (added_zone_points.empty())
        return;

    if (data_.exploring_unknown_zone_kdtree_->size() > 0)
        data_.exploring_unknown_zone_kdtree_->Add_Points(added_zone_points, false);
    else
        data_.exploring_unknown_zone_kdtree_->Build(added_zone_points);
}

/**
 * @brief Collect centroid connected-components from the current relevant graph.
 * @param centroid_clusters Output connected components in centroid space.
 */
void DynamicExpandingGrid::collectRelevantCentroidClusters(
    std::vector<eigen_utils::Vec_Vec3d>& centroid_clusters) const
{
    centroid_clusters.clear();

    std::queue<eigen_utils::Vec3d> centroids_queue;
    eigen_utils::Vec3dSet<3> access_flags;

    for (const auto& [centroid, state] : data_.vertices_explore_state_)
    {
        if (access_flags.find(centroid) != access_flags.end())
            continue;

        eigen_utils::Vec_Vec3d cluster;
        cluster.push_back(centroid);
        centroids_queue.push(centroid);
        access_flags.insert(centroid);

        while (!centroids_queue.empty())
        {
            const eigen_utils::Vec3d cur_centroid = centroids_queue.front();
            centroids_queue.pop();

            const int cur_centroid_id =
                data_.relevant_graph_->getVertexId(cur_centroid.cast<float>());
            if (cur_centroid_id < 0)
            {
                ROS_ERROR("The point is not in the relevant graph!");
                continue;
            }

            const auto& edges = data_.relevant_graph_->getLinkedEdges(cur_centroid_id);
            for (const auto& [nbr_id, edge] : edges)
            {
                const eigen_utils::Vec3d& nbr_centroid =
                    data_.grid_centroids_match_.at(data_.relevant_graph_->getVertexPos(nbr_id));
                if (access_flags.find(nbr_centroid) != access_flags.end())
                    continue;

                cluster.push_back(nbr_centroid);
                centroids_queue.push(nbr_centroid);
                access_flags.insert(nbr_centroid);
            }
        }

        centroid_clusters.push_back(std::move(cluster));
    }
}

/**
 * @brief Keep only centroids in the exploring state for each connected component.
 * @param centroid_clusters Input connected components.
 * @param exploring_centroid_clusters Output exploring-only centroid components.
 */
void DynamicExpandingGrid::collectExploringCentroidClusters(
    const std::vector<eigen_utils::Vec_Vec3d>& centroid_clusters,
    std::vector<eigen_utils::Vec_Vec3d>& exploring_centroid_clusters) const
{
    exploring_centroid_clusters.clear();
    exploring_centroid_clusters.reserve(centroid_clusters.size());

    for (const auto& cluster : centroid_clusters)
    {
        eigen_utils::Vec_Vec3d exploring_cluster;
        for (const auto& centroid : cluster)
        {
            if (data_.vertices_explore_state_.at(centroid) == GRID_EXPLORE_STATE::EXPLORING)
                exploring_cluster.push_back(centroid);
        }
        exploring_centroid_clusters.push_back(std::move(exploring_cluster));
    }
}

/**
 * @brief Build all centroid pairs between different exploring connected components.
 * @param exploring_centroid_clusters Exploring centroid components.
 * @param centroid_pairs Output centroid pairs to evaluate as temporary edges.
 */
void DynamicExpandingGrid::buildInterClusterCentroidPairs(
    const std::vector<eigen_utils::Vec_Vec3d>& exploring_centroid_clusters,
    std::vector<CentroidPair>& centroid_pairs) const
{
    centroid_pairs.clear();

    for (size_t i = 0; i < exploring_centroid_clusters.size(); ++i)
    {
        for (size_t j = i + 1; j < exploring_centroid_clusters.size(); ++j)
        {
            const auto& cluster1 = exploring_centroid_clusters[i];
            const auto& cluster2 = exploring_centroid_clusters[j];

            for (const auto& centroid1 : cluster1)
            {
                for (const auto& centroid2 : cluster2)
                    centroid_pairs.emplace_back(centroid1, centroid2);
            }
        }
    }
}

} // namespace map_process
