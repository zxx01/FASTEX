/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-04 11:33:25
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
void DynamicExpandingGrid::computeTemporaryEdgeDistances(
    const std::vector<CentroidPair>& centroid_pairs,
    eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3>& global_dist_map) const
{
    global_dist_map.clear();

#ifdef USE_OPENMP
    time_utils::Timer::Ptr timer_TE_MT = std::make_shared<time_utils::Timer>(
        "Update Relevant Grid Graph Temporary Edges - Multi-Thread");
    timer_TE_MT->start();

    const int max_threads = omp_get_max_threads();
    std::vector<eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3>> thread_local_maps(
        max_threads);
    std::vector<std::vector<std::string>> thread_local_errors(max_threads);

#pragma omp parallel num_threads(max_threads)
    {
        const int thread_id = omp_get_thread_num();
        auto& thread_local_map = thread_local_maps[thread_id];
        auto& thread_local_error = thread_local_errors[thread_id];

#pragma omp for schedule(dynamic)
        for (size_t idx = 0; idx < centroid_pairs.size(); ++idx)
        {
            eigen_utils::Vec3d from_pos = centroid_pairs[idx].first;
            eigen_utils::Vec3d to_pos = centroid_pairs[idx].second;

            double dist;
            eigen_utils::Vec_Vec3f new_path_f;
            std::vector<int> waypoint_ids;
            int history_pos_graph_from_id = history_pos_graph_->getVertexId(from_pos.cast<float>());
            int history_pos_graph_to_id = history_pos_graph_->getVertexId(to_pos.cast<float>());

            map_process::PATH_SEARCH_RESULT path_result =
                history_pos_graph_->findPath(history_pos_graph_from_id, history_pos_graph_to_id,
                                             waypoint_ids, new_path_f, dist)
                    ? map_process::PATH_SEARCH_RESULT::SUCCESS
                    : map_process::PATH_SEARCH_RESULT::FAIL;
            if (path_result == map_process::PATH_SEARCH_RESULT::FAIL)
                ROS_ERROR("history_pos_graph_: No path between two centroids!");

            if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS && dist < 80.0)
            {
                double new_dist;
                eigen_utils::Vec_Vec3d new_path;
                map_process::PATH_SEARCH_RESULT path_result2 =
                    path_searcher_->searchCoarsePathWithWSRoadMap(
                        from_pos, to_pos, new_path, new_dist, true, params_.max_straight_dist_);

                if (path_result2 == map_process::PATH_SEARCH_RESULT::SUCCESS)
                    dist = new_dist;
            }

            if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
                thread_local_map[from_pos][to_pos] = dist;
            else
                thread_local_error.push_back("No T-path between two centroids between clusters " +
                                             eigen_utils::vec_to_string(from_pos) + " and " +
                                             eigen_utils::vec_to_string(to_pos));
        }
    }

    for (const auto& errors : thread_local_errors)
    {
        for (const auto& error_msg : errors)
            ROS_ERROR_STREAM(error_msg);
    }

    for (const auto& local_map : thread_local_maps)
    {
        for (const auto& [from_pos, to_pos_dist_map] : local_map)
        {
            for (const auto& [to_pos, dist] : to_pos_dist_map)
                global_dist_map[from_pos][to_pos] = dist;
        }
    }

    timer_TE_MT->stop(false, "ms");
#else
    time_utils::Timer::Ptr timer_TE_ST = std::make_shared<time_utils::Timer>(
        "Update Relevant Grid Graph Temporary Edges - Single-Thread");
    timer_TE_ST->start();

    for (size_t idx = 0; idx < centroid_pairs.size(); ++idx)
    {
        eigen_utils::Vec3d from_pos = centroid_pairs[idx].first;
        eigen_utils::Vec3d to_pos = centroid_pairs[idx].second;

        double dist;
        eigen_utils::Vec_Vec3d new_path;
        map_process::PATH_SEARCH_RESULT path_result = path_searcher_->searchCoarsePathWithWSRoadMap(
            from_pos, to_pos, new_path, dist, true, params_.max_straight_dist_);

        if (path_result == map_process::PATH_SEARCH_RESULT::SUCCESS)
            global_dist_map[from_pos][to_pos] = dist;
        else
            ROS_ERROR("No T-path between two centroids!");
    }

    timer_TE_ST->stop(false, "ms");
#endif // USE_OPENMP
}

/**
 * @brief Insert temporary edges into the relevant graph from a centroid distance map.
 * @param global_dist_map Pairwise centroid distance map for valid temporary edges.
 */
void DynamicExpandingGrid::insertTemporaryEdgesToRelevantGraph(
    const eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3>& global_dist_map)
{
    for (const auto& [from_pos, to_pos_dist_map] : global_dist_map)
    {
        for (const auto& [to_pos, dist] : to_pos_dist_map)
        {
            data_.relevant_graph_->addTemporaryTwoWayEdge(
                data_.relevant_graph_->getVertexId(from_pos.cast<float>()),
                data_.relevant_graph_->getVertexId(to_pos.cast<float>()),
                RelevantGraphEdgeType(dist, true));
        }
    }
}

/**
 * @brief Collect grids whose enduring edges need to be refreshed in the current frame.
 * @param update_min Minimum corner of the update box.
 * @param update_max Maximum corner of the update box.
 * @param new_grid_indices Newly added relevant grids.
 * @param consider_grid_indices Output relevant grids whose enduring edges should be updated.
 */
void DynamicExpandingGrid::collectRelevantGridsForEnduringEdgeUpdate(
    const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max,
    const eigen_utils::Vec3iSet& new_grid_indices,
    eigen_utils::Vec3iSet& consider_grid_indices) const
{
    consider_grid_indices = new_grid_indices;

    const eigen_utils::Vec3d update_min_inflated = update_min - params_.initial_resolution_;
    const eigen_utils::Vec3d update_max_inflated = update_max + params_.initial_resolution_;

    eigen_utils::Vec3d grid_min, grid_max;
    for (const auto& grid_index : data_.current_relevant_grid_)
    {
        data_.grid_->getGridMinMaxBox(grid_index, grid_min, grid_max);
        if (process_utils::ProcessUtils::isOverlapped(update_min_inflated, update_max_inflated,
                                                      grid_min, grid_max))
            consider_grid_indices.insert(grid_index);
    }
}

/**
 * @brief Compute the candidate enduring-edge cost between two relevant centroids.
 * @param from_centroid Source centroid.
 * @param to_centroid Target centroid.
 * @param box_search_margin Margin used to expand the local path-search box.
 * @return Positive cost if a valid edge exists; otherwise `-1.0`.
 */
double
DynamicExpandingGrid::computeEnduringEdgeCost(const eigen_utils::Vec3d& from_centroid,
                                              const eigen_utils::Vec3d& to_centroid,
                                              const eigen_utils::Vec3d& box_search_margin) const
{
    eigen_utils::Vec3d search_box_min = from_centroid.cwiseMin(to_centroid) - box_search_margin;
    eigen_utils::Vec3d search_box_max = from_centroid.cwiseMax(to_centroid) + box_search_margin;

    double dist;
    eigen_utils::Vec_Vec3d path;
    const map_process::PATH_SEARCH_RESULT path_result =
        path_searcher_->searchCoarsePathWithWSRoadMapWithinBox(from_centroid, to_centroid, path,
                                                               dist, search_box_min, search_box_max,
                                                               true, params_.max_straight_dist_);

    if (path_result != map_process::PATH_SEARCH_RESULT::SUCCESS)
    {
        ROS_ERROR("No path between two neighbor grid centroids!");
        return -1.0;
    }

    int unexplored_cnt = 0;
    if (data_.vertices_explore_state_.at(from_centroid) == GRID_EXPLORE_STATE::UNEXPLORED)
        ++unexplored_cnt;
    if (data_.vertices_explore_state_.at(to_centroid) == GRID_EXPLORE_STATE::UNEXPLORED)
        ++unexplored_cnt;

    double penalty = 1.0;
    switch (unexplored_cnt)
    {
    case 1:
        penalty = 1.2;
        break;
    case 2:
        penalty = 1.4;
        break;
    default:
        break;
    }

    return dist * penalty;
}

/**
 * @brief Collect enduring-edge costs between centroids inside the same relevant grid.
 * @param grid_index Relevant grid whose internal centroid chain should be connected.
 * @param box_search_margin Margin used to expand the local path-search box.
 * @param edge_costs Output edge-cost cache keyed by relevant-graph vertex ids.
 */
void DynamicExpandingGrid::collectIntraGridEnduringEdges(
    const eigen_utils::Vec3i& grid_index, const eigen_utils::Vec3d& box_search_margin,
    RelevantEdgeCostMap& edge_costs) const
{
    const auto& grid_centroids = data_.grid_indices_centroids_.at(grid_index);
    for (size_t i = 0; i + 1 < grid_centroids.size(); ++i)
    {
        const auto& cur_centroid = grid_centroids[i];
        const auto& nbr_centroid = grid_centroids[i + 1];

        const int cur_vertex_id = data_.relevant_graph_->getVertexId(cur_centroid.cast<float>());
        const int nbr_vertex_id = data_.relevant_graph_->getVertexId(nbr_centroid.cast<float>());
        if (cur_vertex_id < 0 || nbr_vertex_id < 0)
        {
            ROS_ERROR("The vertex is not in the relevant graph!");
            continue;
        }

        const int min_id = std::min(cur_vertex_id, nbr_vertex_id);
        const int max_id = std::max(cur_vertex_id, nbr_vertex_id);
        if (edge_costs.count(min_id) && edge_costs.at(min_id).count(max_id))
            continue;

        edge_costs[min_id][max_id] =
            computeEnduringEdgeCost(cur_centroid, nbr_centroid, box_search_margin);
    }
}

/**
 * @brief Collect enduring-edge costs between centroids in neighboring relevant grids.
 * @param grid_index Source relevant grid.
 * @param current_grid_indices All current relevant grids.
 * @param box_search_margin Margin used to expand the local path-search box.
 * @param edge_costs Output edge-cost cache keyed by relevant-graph vertex ids.
 */
void DynamicExpandingGrid::collectNeighborGridEnduringEdges(
    const eigen_utils::Vec3i& grid_index, const eigen_utils::Vec3iSet& current_grid_indices,
    const eigen_utils::Vec3d& box_search_margin, RelevantEdgeCostMap& edge_costs) const
{
    const auto& cur_grid_centroids = data_.grid_indices_centroids_.at(grid_index);
    const auto nbr_indices = process_utils::ProcessUtils::allNeighbors(grid_index);

    for (const auto& nbr_index : nbr_indices)
    {
        if (current_grid_indices.find(nbr_index) == current_grid_indices.end())
            continue;

        const auto& nbr_grid_centroids = data_.grid_indices_centroids_.at(nbr_index);
        for (const auto& cur_centroid : cur_grid_centroids)
        {
            for (const auto& nbr_centroid : nbr_grid_centroids)
            {
                const int cur_vertex_id =
                    data_.relevant_graph_->getVertexId(cur_centroid.cast<float>());
                const int nbr_vertex_id =
                    data_.relevant_graph_->getVertexId(nbr_centroid.cast<float>());
                if (cur_vertex_id < 0 || nbr_vertex_id < 0)
                {
                    ROS_ERROR("The vertex is not in the relevant graph!");
                    continue;
                }

                const int min_id = std::min(cur_vertex_id, nbr_vertex_id);
                const int max_id = std::max(cur_vertex_id, nbr_vertex_id);
                if (edge_costs.count(min_id) && edge_costs.at(min_id).count(max_id))
                    continue;

                edge_costs[min_id][max_id] =
                    computeEnduringEdgeCost(cur_centroid, nbr_centroid, box_search_margin);
            }
        }
    }
}

/**
 * @brief Insert valid enduring edges into the relevant graph from a vertex-id cost cache.
 * @param edge_costs Candidate enduring-edge costs keyed by relevant-graph vertex ids.
 */
void DynamicExpandingGrid::insertEnduringEdgesToRelevantGraph(const RelevantEdgeCostMap& edge_costs)
{
    for (const auto& [min_id, edge_cost] : edge_costs)
    {
        for (const auto& [max_id, cost] : edge_cost)
        {
            if (cost > 0)
                data_.relevant_graph_->addTwoWayEdge(min_id, max_id,
                                                     RelevantGraphEdgeType(cost, true));
        }
    }
}

/**
 * @brief Set current position of the robot
 *
 * @param current_position  current position of the robot
 */
void DynamicExpandingGrid::setCurrentPosition(const eigen_utils::Vec3d& current_position)
{
    data_.current_position_ = current_position;
}

/**
 * @brief Set path searcher for the dynamic expanding grid
 *
 * @param path_searcher  path searcher for the dynamic expanding grid
 */
void DynamicExpandingGrid::setPathSearcher(
    const map_process::PathSearcher::SharedPtr& path_searcher)
{
    path_searcher_ = path_searcher;
}

void DynamicExpandingGrid::setWholeStateRoadMap(
    const map_process::WSRoadMap::SharedPtr& whole_state_road_map)
{
    whole_state_road_map_ = whole_state_road_map;
}

void DynamicExpandingGrid::setFrontierManager(
    const map_process::FrontierManager::SharedPtr& frontier_manager)
{
    frontier_manager_ = frontier_manager;

    data_.grid_->setFrontierManager(frontier_manager);
}

void DynamicExpandingGrid::setHistoryPosGraph(
    const map_process::HistoryPosGraph::SharedPtr& history_pos_graph)
{
    history_pos_graph_ = history_pos_graph;
}

/**
 * @brief Find the extend grid indices for the DynamicExpandingGrid
 *
 * @param real_cloud The real sensor cloud data
 * @param virtual_cloud The virtual sensor cloud data
 * @param label_clouds The label cloud data
 * @param real_cloud_located_grid_indices The real cloud located grid indices
 * @param extended_grid_indices The extended grid indices
 * @param extend_grid_coords The extended grid coordinates
 *
 */
template <typename PointT>
void DynamicExpandingGrid::processSensorCloud(
    const typename pcl::PointCloud<PointT>::Ptr& real_cloud,
    const typename pcl::PointCloud<PointT>::Ptr& virtual_cloud,
    eigen_utils::Vec3iMap<std::vector<PointT>>& label_clouds,
    eigen_utils::Vec3iSet& real_cloud_located_grid_indices,
    eigen_utils::Vec3iSet& extended_grid_indices,
    std::vector<openvdb::math::Coord>& extend_grid_coords)
{
    // Handle the grid activation
    auto handleGridActivation = [&](const openvdb::math::Coord& coord) -> bool {
        openvdb::BBoxd bbox;
        SingleLevelGrid& grid = *data_.grid_;
        grid.getGridBBoxd(coord, bbox);
        const eigen_utils::Vec3d box_min = fromVdb(bbox.min());
        const eigen_utils::Vec3d box_max = fromVdb(bbox.max());

        // Check if the box is overlapped with the setting min and max range
        if (params_.is_min_max_set_ && !process_utils::ProcessUtils::isOverlapped(
                                           params_.min_set_, params_.max_set_, box_min, box_max))
            return false;

        const eigen_utils::Vec3i grid_index = fromVdbCoord(coord);
        if (!grid.checkGridExisted(grid_index))
            extend_grid_coords.emplace_back(coord);

        return true;
    };

    extend_grid_coords.clear();

    if (real_cloud->empty() && virtual_cloud->empty())
        return;

    // 1.Downsample the sensor cloud
    typename pcl::PointCloud<PointT>::Ptr ds_sensor_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr ds_real_cloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr ds_virtual_cloud(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> visible_voxel_grid;
    visible_voxel_grid.setLeafSize(2.0f, 2.0f, 2.0f);
    visible_voxel_grid.setDownsampleAllData(true);

    visible_voxel_grid.setInputCloud(real_cloud);
    visible_voxel_grid.filter(*ds_real_cloud);

    visible_voxel_grid.setInputCloud(virtual_cloud);
    visible_voxel_grid.filter(*ds_virtual_cloud);

    *ds_sensor_cloud = *ds_real_cloud + *ds_virtual_cloud;

    // 2.Compute the visible cloud by Hidden Point Removal algorithm (HPR) for the sensor cloud
    typename pcl::PointCloud<PointT>::Ptr visible_cloud(new pcl::PointCloud<PointT>);

    utils::HPR<PointT> hpr;
    hpr.setKernalType(utils::KernalType::EXPONENTIAL);
    hpr.setGamma(-0.01);
    hpr.setCenterPoint(data_.current_position_.cast<float>());
    hpr.setInputCloud(ds_sensor_cloud);
    *cumulative_virtual_cloud_all_ += *ds_sensor_cloud;
    visible_voxel_grid.setInputCloud(cumulative_virtual_cloud_all_->makeShared());
    visible_voxel_grid.filter(*cumulative_virtual_cloud_all_);

    time_utils::Timer::Ptr timer_HPR = std::make_shared<time_utils::Timer>("HPR");
    timer_HPR->start();
    hpr.compute(visible_cloud);
    timer_HPR->stop(false, "ms");

    *cumulative_vis_virtual_cloud_ += *visible_cloud;
    visible_voxel_grid.setInputCloud(cumulative_vis_virtual_cloud_->makeShared());
    visible_voxel_grid.filter(*cumulative_vis_virtual_cloud_);

    // 3.Activate the grid cells that the visible cloud points are in
    time_utils::Timer::Ptr timer2 =
        std::make_shared<time_utils::Timer>("Convert Visible Cloud to BoolGrid");
    timer2->start();

    openvdb::BoolGrid::Ptr local_active_grid = openvdb::BoolGrid::create(false);
    local_active_grid->setTransform(data_.active_grid_->transform().copy());

    openvdb::BoolGrid::Accessor local_acc = local_active_grid->getAccessor();
    const openvdb::math::Transform& local_tf(local_active_grid->transform());

    openvdb::math::Coord cur_ijk =
        local_tf.worldToIndexNodeCentered(toVdb(data_.current_position_));
    openvdb::Vec3d origin_xyz = local_tf.worldToIndex(toVdb(data_.current_position_));
    local_acc.setValue(cur_ijk, true);

    for (const auto& pt : visible_cloud->points)
    {
        openvdb::math::Coord end_ijk =
            local_tf.worldToIndexNodeCentered(openvdb::Vec3d(pt.x, pt.y, pt.z));

        // Collect the label cloud data
        if (std::fabs(pt.intensity) < 1e-6 && // real cloud
            process_utils::ProcessUtils::isInBox(eigen_utils::Vec3d(pt.x, pt.y, pt.z),
                                                 params_.min_set_,
                                                 params_.max_set_)) // in the setting range
        {
            label_clouds[fromVdbCoord(end_ijk)].push_back(pt);
        }

        // Activate the grid cells that the visible cloud points are in
        if (!local_acc.isValueOn(end_ijk))
        {
            local_acc.setValue(end_ijk, true);
            real_cloud_located_grid_indices.insert(fromVdbCoord(end_ijk));
        }

        openvdb::Vec3d end_xyz = local_tf.worldToIndex(openvdb::Vec3d(pt.x, pt.y, pt.z));
        openvdb::Vec3d dir = end_xyz - origin_xyz;
        double length = dir.length();
        dir.normalize();
        openvdb::math::Ray<double> ray(origin_xyz, dir);
        openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0.0, length);

        while (dda.time() < dda.maxTime())
        {
            openvdb::math::Coord coord = dda.voxel();
            if (!local_acc.isValueOn(coord))
            {
                local_acc.setValue(coord, true);
            }
            dda.step();
        }
    }

    // 4. Collect the extend grid indices
    openvdb::BoolGrid::Accessor global_acc = data_.active_grid_->getAccessor();
    for (openvdb::BoolGrid::ValueOnCIter iter = local_active_grid->cbeginValueOn(); iter; ++iter)
    {
        const openvdb::math::Coord coord = iter.getCoord();
        if (global_acc.isValueOn(coord))
            continue;

        bool flag = handleGridActivation(coord);

        if (flag)
        {
            global_acc.setValue(coord, true);
            extended_grid_indices.insert(fromVdbCoord(coord));
        }
    }

    timer2->stop(false, "ms");

    // Publish the visible cloud(only for visualization, not necessary for the algorithm)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr visible_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& point : visible_cloud->points)
        {
            pcl::PointXYZI point_i;
            point_i.x = point.x;
            point_i.y = point.y;
            point_i.z = point.z;
            point_i.intensity = point.intensity;
            visible_cloud_i->push_back(point_i);
        }

        sensor_msgs::PointCloud2 visible_cloud_msg;
        pcl::toROSMsg(*visible_cloud_i, visible_cloud_msg);
        visible_cloud_msg.header.stamp = ros::Time::now();
        visible_cloud_msg.header.frame_id = "world";
        visible_cloud_pub_.publish(visible_cloud_msg);

        pcl::toROSMsg(*cumulative_virtual_cloud_all_, visible_cloud_msg);
        visible_cloud_msg.header.stamp = ros::Time::now();
        visible_cloud_msg.header.frame_id = "world";
        all_virtual_cloud_pub_.publish(visible_cloud_msg);

        pcl::toROSMsg(*cumulative_vis_virtual_cloud_, visible_cloud_msg);
        visible_cloud_msg.header.stamp = ros::Time::now();
        visible_cloud_msg.header.frame_id = "world";
        all_vis_virtual_cloud_pub_.publish(visible_cloud_msg);

        auto grid_resolution = data_.active_grid_->transform().voxelSize();
        publishActiveVoxels(
            active_grid_pub_, data_.active_grid_,
            eigen_utils::Vec3d(grid_resolution.x(), grid_resolution.y(), grid_resolution.z()));
    }
}

/**
 * @brief Process the label cloud data.
 *
 * This function processes the label cloud data by constructing a kdtree for the cumulated position
 * data, updating the intensity of the covered points, downsampling the label cloud, and storing the
 * processed data back to the hash structure. It also inserts the cumulated position data to the key
 * position kdtree and publishes the label cloud for visualization.
 *
 * @tparam PointT The type of the point in the point cloud.
 * @param cumulated_pos The cumulated position data.
 * @param real_cloud_located_grid_indices The grid indices where the real cloud is located.
 * @param label_clouds The label clouds data.
 * @param label_clouds_covered_ratio The covered ratio of the label clouds.
 * @param covered_range The range within which the points are considered covered.
 * @param keypos_interval The interval between key positions.
 */
template <typename PointT>
void DynamicExpandingGrid::processLabelCloud(
    const eigen_utils::Vec_Vec3d& cumulated_pos,
    const eigen_utils::Vec3iSet& real_cloud_located_grid_indices,
    eigen_utils::Vec3iMap<std::vector<PointT>>& label_clouds,
    eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio, const double covered_range,
    const double keypos_interval)
{
    if (cumulated_pos.empty())
        return;

    // 1. Construct the kdtree for the cumulated position data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cumulated_pos_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cumulated_pos_cloud->reserve(cumulated_pos.size());

    for (const auto& pos : cumulated_pos)
    {
        pcl::PointXYZ point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();
        cumulated_pos_cloud->push_back(point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> pos_kdtree;
    pos_kdtree.setInputCloud(cumulated_pos_cloud);

    // Lambda function: Iterate through all points in the point cloud, re-evaluate points with
    // intensity less than 1e-3 to determine if they are covered, and if so, set their intensity to
    // 255. Two searches will be performed: one for recent cumulated position points and one for
    // historical key position points.
    auto updateIntensityIfCovered = [&](typename pcl::PointCloud<PointT>::Ptr cloud,
                                        const double threshold = 15.0) -> void {
        pcl::PointXYZ search_point;
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        std::vector<float> nbr_dist;
        KD_TREE<pcl::PointXYZ>::PointVector nbr_points;

        for (auto& pt : cloud->points)
        {
            if (pt.intensity < 1e-3)
            {
                search_point.x = pt.x;
                search_point.y = pt.y;
                search_point.z = pt.z;

                // Search for recent key position points, if the distance is less than 15m, consider
                // it covered
                if (pos_kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch,
                                              pointNKNSquaredDistance) > 0 &&
                    pointNKNSquaredDistance[0] < threshold * threshold)
                {
                    pt.intensity = 255;
                    continue;
                }

                // Search for historical key position points, if the distance is less than 15m,
                // consider it covered
                if (data_.key_pos_kdtree_->size() > 0)
                {
                    data_.key_pos_kdtree_->Nearest_Search(search_point, 1, nbr_points, nbr_dist);
                    if (nbr_dist[0] < threshold * threshold)
                    {
                        pt.intensity = 255;
                    }
                }
            }
        }
    };

    // 2. Extract all label_clouds data corresponding to grid_indices, downsample, update coverage
    // properties based on recent position points and historical key position points, and store back
    // to the hash structure
    typename pcl::PointCloud<PointT>::Ptr label_cloud_temp(new pcl::PointCloud<PointT>);

    // 2.1 Determine the area that needs label cloud processing
    openvdb::BBoxd bbox;
    bbox.min() = openvdb::Vec3d(data_.current_position_.x() - covered_range,
                                data_.current_position_.y() - covered_range,
                                data_.current_position_.z() - 8.0);
    bbox.max() = openvdb::Vec3d(data_.current_position_.x() + covered_range,
                                data_.current_position_.y() + covered_range,
                                data_.current_position_.z() + 8.0);

    openvdb::math::CoordBBox bbox_coord;
    vdb_utils::VDBUtil::convertBBoxdToCoordBox(data_.active_grid_, bbox, false, bbox_coord);

    eigen_utils::Vec3iSet grid_indices = real_cloud_located_grid_indices;
    for (auto iter = bbox_coord.beginXYZ(); iter; ++iter)
    {
        const openvdb::math::Coord coord = *iter;
        grid_indices.insert(fromVdbCoord(coord));
    }

    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(2.0, 2.0, 2.0);

    // 2.2 Iterate through all grid_indices and process the label cloud
    for (const auto& grid_index : grid_indices)
    {
        if (label_clouds.find(grid_index) != label_clouds.end())
        {
            // Copy the label cloud data from the hash structure
            label_cloud_temp->clear();
            label_cloud_temp->reserve(label_clouds[grid_index].size());
            for (const auto& point : label_clouds[grid_index])
                label_cloud_temp->push_back(point);

            // Downsample the label cloud
            voxel_grid.setInputCloud(label_cloud_temp->makeShared());
            voxel_grid.filter(*label_cloud_temp);

            // Update the intensity of the covered points
            updateIntensityIfCovered(label_cloud_temp, covered_range);

            // Store the downsampled label cloud data back to the hash structure
            label_clouds[grid_index].clear();
            label_clouds[grid_index].reserve(label_cloud_temp->size());

            int point_cnt = 0, uncovered_cnt = 0;
            for (const auto& point : label_cloud_temp->points)
            {
                ++point_cnt;
                if (std::fabs(point.intensity) < 1e-6)
                    ++uncovered_cnt;

                label_clouds[grid_index].push_back(point);
            }

            label_clouds_covered_ratio[grid_index] = double(uncovered_cnt) / double(point_cnt);
        }
    }

    // 3. Insert the cumulated position data to the key position kdtree
    pcl::PointXYZ search_point;
    std::vector<float> nbr_dist;
    KD_TREE<pcl::PointXYZ>::PointVector nbr_points;
    KD_TREE<pcl::PointXYZ>::PointVector add_points;
    for (const auto& pos : cumulated_pos)
    {
        search_point.x = pos.x();
        search_point.y = pos.y();
        search_point.z = pos.z();

        if (data_.key_pos_kdtree_->size() == 0)
        {
            add_points.clear();
            add_points.push_back(search_point);
            data_.key_pos_kdtree_->Build(add_points);
        }
        else
        {
            data_.key_pos_kdtree_->Nearest_Search(search_point, 1, nbr_points, nbr_dist);

            // insert the point if the distance is larger than the keypos_interval
            if (nbr_dist[0] > keypos_interval * keypos_interval)
            {
                add_points.clear();
                add_points.push_back(search_point);
                data_.key_pos_kdtree_->Add_Points(add_points, false);
            }
        }
    }

    // Publish the label cloud (only for visualization, not necessary for the algorithm)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr label_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointXYZI point_i;
        for (const auto& pair : label_clouds)
        {
            for (const auto& point : pair.second)
            {
                point_i.x = point.x;
                point_i.y = point.y;
                point_i.z = point.z;
                point_i.intensity = std::fabs(point.intensity) > 1e-6 ? 255 : 0.0;
                label_cloud_i->push_back(point_i);
            }
        }

        sensor_msgs::PointCloud2 label_cloud_msg;
        pcl::toROSMsg(*label_cloud_i, label_cloud_msg);
        label_cloud_msg.header.stamp = ros::Time::now();
        label_cloud_msg.header.frame_id = "world";
        label_cloud_pub_.publish(label_cloud_msg);
    }
}

/**
 * @brief Update DynamicExpandingGrid data
 *
 * @param real_cloud The real sensor cloud data
 * @param virtual_cloud The virtual sensor cloud data
 * @param cumulated_pos The cumulated position data
 * @param changed_cells changed cells that need to be input to the grid
 * @param frontier_clusters frontier clusters that need to be input to the grid
 * @param update_min  the min point of the update box
 * @param update_max  the max point of the update box
 *
 */
void DynamicExpandingGrid::updateGridData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& real_cloud,
                                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& virtual_cloud,
                                          const eigen_utils::Vec_Vec3d& cumulated_pos,
                                          const std::vector<int>& changed_cells,
                                          const eigen_utils::Vec3d& update_min,
                                          const eigen_utils::Vec3d& update_max)
{
    time_utils::Timer::Ptr timer1 = std::make_shared<time_utils::Timer>("Update Grid Data");
    timer1->start();

    std::unordered_map<int, eigen_utils::Vec3d> cluster_centroids_temp;
    collectFrontierClusterCentroids(cluster_centroids_temp);

    // 1. Convert changed_cells to world coordinates and merge deferred cells.
    std::vector<openvdb::Vec3d> changed_cells_temp = data_.reserved_changed_cells_;
    appendChangedCells(changed_cells, changed_cells_temp);

    // 2. Find the grids touched by current sensor data.
    time_utils::Timer::Ptr timer_find_extend_grid =
        std::make_shared<time_utils::Timer>("Find Extend Grid Indices");
    timer_find_extend_grid->start();

    std::vector<openvdb::math::Coord> extend_grid_coords;
    eigen_utils::Vec3iSet real_cloud_located_grid_indices, extended_grid_indices;
    processSensorCloud(real_cloud, virtual_cloud, data_.label_clouds_,
                       real_cloud_located_grid_indices, extended_grid_indices, extend_grid_coords);
    recordNewExtendGridBoxes(extended_grid_indices);

    timer_find_extend_grid->stop(false, "ms");

    // 3. Refresh label-cloud coverage after the active-grid set is known.
    processLabelCloud(cumulated_pos, real_cloud_located_grid_indices, data_.label_clouds_,
                      data_.label_clouds_uncovered_ratio_, 15.0, 2.0);

    // 4. Apply all updates to the single-level grid and keep deferred cells for next round.
    syncSingleLevelGridData(changed_cells_temp, cluster_centroids_temp, extend_grid_coords,
                            update_min, update_max);
    data_.reserved_changed_cells_ = changed_cells_temp;
    timer1->stop(false, "ms");
}

/**
 * @brief Update the relevant grid set.
 *
 */
void DynamicExpandingGrid::updateRelevantGrid()
{
    data_.grid_indices_centroids_.clear();
    data_.grid_centroids_indices_.clear();
    data_.vertices_explore_state_.clear();

    auto& slgrid = *data_.grid_;
    auto& grid_ids = data_.current_relevant_grid_;
    grid_ids.clear();

    slgrid.extractRelevantGridIds(grid_ids);
    populateRelevantGridMetadata(grid_ids);
}

/**
 * @brief Get the state change grid boxes
 *
 * This function retrieves the grid boxes that have changed state. It categorizes the boxes into
 * new boxes, boxes that have changed to explored state, and boxes that have changed from explored
 * state.
 *
 * @param new_boxes A vector to store the new grid boxes
 * @param change_to_explored_boxes A vector to store the grid boxes that have changed to explored
 * state
 * @param change_from_explored_boxes A vector to store the grid boxes that have changed from
 * explored state
 */
void DynamicExpandingGrid::getStateChangeGridBoxes(
    std::vector<process_utils::CubeBox>& new_boxes,
    std::vector<process_utils::CubeBox>& change_to_explored_boxes,
    std::vector<process_utils::CubeBox>& change_from_explored_boxes)
{
    new_boxes.clear();
    change_to_explored_boxes.clear();
    change_from_explored_boxes.clear();

    new_boxes.swap(data_.new_extend_grid_boxes_);

    const SingleLevelGrid& slgrid = *data_.grid_;
    eigen_utils::Vec3iSet to_explored_indices, from_explored_indices;
    slgrid.collectStateTransitionGrids(to_explored_indices, from_explored_indices);

    for (const auto& idx : to_explored_indices)
    {
        process_utils::CubeBox box;
        slgrid.getGridMinMaxBox(idx, box.min_, box.max_);
        change_to_explored_boxes.push_back(box);
    }
    for (const auto& idx : from_explored_indices)
    {
        process_utils::CubeBox box;
        slgrid.getGridMinMaxBox(idx, box.min_, box.max_);
        change_from_explored_boxes.push_back(box);
    }
}

/**
 * @brief Phase 0: refresh exploration info and unknown-zone roadmap vertices.
 */
void DynamicExpandingGrid::updateExplorationInfoAndRoadmap(
    const eigen_utils::Vec_Vec3i& update_indices)
{
    auto timer_info = std::make_shared<time_utils::Timer>("Update Grid Exploration Info");
    timer_info->start();
    updateGridExplorationInfoBatch(update_indices);
    timer_info->stop(false, "ms");

    auto timer_roadmap =
        std::make_shared<time_utils::Timer>("Update Zone And Cluster Vertices On RoadMap");
    timer_roadmap->start();
    updateZoneAndClusterVerticesOnRoadMap(update_indices);
    timer_roadmap->stop(false, "ms");
}

/**
 * @brief BFS from a start centroid to collect a connected component in the
 *        relevant graph.
 * @return true if the component contains at least one EXPLORING vertex.
 */
bool DynamicExpandingGrid::collectConnectedComponent(const eigen_utils::Vec3d& start_centroid,
                                                     eigen_utils::Vec_Vec3d& component,
                                                     eigen_utils::Vec3iSet& voxel_visited) const
{
    component.clear();

    eigen_utils::Vec3i pt_voxel_idx;
    std::queue<eigen_utils::Vec3d> queue;

    sdf_map_->posToIndex(start_centroid, pt_voxel_idx);
    voxel_visited.insert(pt_voxel_idx);
    queue.push(start_centroid);
    component.push_back(start_centroid);

    bool has_exploring =
        data_.vertices_explore_state_.at(start_centroid) == GRID_EXPLORE_STATE::EXPLORING;

    while (!queue.empty())
    {
        const eigen_utils::Vec3d cur = queue.front();
        queue.pop();

        const int cur_id = data_.relevant_graph_->getVertexId(cur.cast<float>());
        if (cur_id < 0)
        {
            ROS_ERROR("The point is not in the relevant graph!");
            continue;
        }

        for (const auto& [nbr_id, edge] : data_.relevant_graph_->getLinkedEdges(cur_id))
        {
            const eigen_utils::Vec3d& nbr =
                data_.grid_centroids_match_.at(data_.relevant_graph_->getVertexPos(nbr_id));

            sdf_map_->posToIndex(nbr, pt_voxel_idx);
            if (voxel_visited.find(pt_voxel_idx) != voxel_visited.end())
                continue;

            voxel_visited.insert(pt_voxel_idx);
            queue.push(nbr);
            component.push_back(nbr);

            if (!has_exploring &&
                data_.vertices_explore_state_.at(nbr) == GRID_EXPLORE_STATE::EXPLORING)
                has_exploring = true;
        }
    }

    return has_exploring;
}

void DynamicExpandingGrid::updateRelevantGridGraph(const eigen_utils::Vec3d& update_min,
                                                   const eigen_utils::Vec3d& update_max)
{
    const eigen_utils::Vec_Vec3i update_indices = data_.grid_->getGridUpdateIds();

    // Phase 0 — Refresh exploration info and unknown-zone roadmap vertices
    updateExplorationInfoAndRoadmap(update_indices);

    // Phase 1 — Determine current relevant grids and compute vertex diff
    eigen_utils::Vec3iSet last_relevant_grid = data_.current_relevant_grid_;
    eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d> last_centroids =
        std::move(data_.grid_indices_centroids_);

    updateRelevantGrid();

    eigen_utils::Vec3iSet removed_grid_indices, new_grid_indices, unchanged_grid_indices;
    analyzeGridChanges(data_.current_relevant_grid_, last_relevant_grid, removed_grid_indices,
                       new_grid_indices, unchanged_grid_indices);

    eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d> removed_centroids, new_centroids;
    collectChangedRelevantGridCentroids(removed_grid_indices, new_grid_indices,
                                        unchanged_grid_indices, last_centroids, update_min,
                                        update_max, removed_centroids, new_centroids);

    // Phase 2 — Sync persistent graph: vertices + enduring edges
    updateRelevantGridGraphVertices(removed_centroids, new_centroids);

    auto timer_enduring =
        std::make_shared<time_utils::Timer>("Update Relevant Grid Graph Enduring Edges");
    timer_enduring->start();
    updateRelevantGridGraphEnduringEdges(update_min, update_max, new_grid_indices);
    timer_enduring->stop(false, "ms");

    // Phase 3 — Remove isolated unexplored components
    auto timer_erase = std::make_shared<time_utils::Timer>("Erase Isolated Unexplored Grid");
    timer_erase->start();
    eraseIsolatedUnexploredGrid();
    timer_erase->stop(false, "ms");

    // Phase 4 — Rebuild vertex caches
    auto timer_cache = std::make_shared<time_utils::Timer>("Update Relevant Graph Vertices Data");
    timer_cache->start();
    rebuildRelevantGraphVertexCache();
    timer_cache->stop(false, "ms");

    // Phase 5 — Add temporary exploring-to-exploring edges
    auto timer_temp =
        std::make_shared<time_utils::Timer>("Update Relevant Grid Graph Temporary Edges");
    timer_temp->start();
    updateRelevantGridGraphTemporaryEdges();
    timer_temp->stop(false, "ms");
}

void DynamicExpandingGrid::updateZoneAndClusterVerticesOnRoadMap(
    const eigen_utils::Vec_Vec3i& update_grid_indices)
{
    std::vector<process_utils::CubeBox> refresh_boxes_all;
    ZoneToAllClusterTopVps zone_to_all_cluster_top_vps;
    ZoneToNearestClusterTopVp zone_to_nearest_cluster_top_vp;
    KD_TREE<pcl::PointXYZ>::PointVector add_points_vec;

    collectRefreshBoxes(update_grid_indices, refresh_boxes_all);
    collectUnknownZoneRoadmapTargets(update_grid_indices, zone_to_all_cluster_top_vps,
                                     zone_to_nearest_cluster_top_vp);
    removeStaleUnknownZoneVertices(refresh_boxes_all);
    insertUnknownZoneVerticesToRoadMap(zone_to_nearest_cluster_top_vp, add_points_vec);
    insertUnknownZoneVerticesToHistoryGraph(zone_to_all_cluster_top_vps);
    updateUnknownZoneKdTree(add_points_vec);
}

void DynamicExpandingGrid::updateRelevantGridGraphVertices(
    const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& removed_vertices,
    const eigen_utils::Vec3iMap<eigen_utils::Vec_Vec3d>& new_vertices)
{
    eigen_utils::Vec3f centroid_f;

    // 1. Delete the erased relevant grid centroids from the relevant graph
    for (const auto& [grid_idx, centroids] : removed_vertices)
        for (const auto& pt : centroids)
        {
            centroid_f = pt.cast<float>();
            data_.grid_centroids_match_.erase(centroid_f);
            data_.relevant_graph_->deleteVertex(data_.relevant_graph_->getVertexId(centroid_f),
                                                true);
        }

    // 2. Add the new relevant grid centroids to the relevant graph
    for (const auto& [grid_idx, centroids] : new_vertices)
        for (const auto& pt : centroids)
        {
            centroid_f = pt.cast<float>();
            data_.grid_centroids_match_[centroid_f] = pt;
            data_.relevant_graph_->addVertex(RelevantGraphVertexType(centroid_f, true), true);
        }
}

void DynamicExpandingGrid::updateRelevantGridGraphEnduringEdges(
    const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max,
    const eigen_utils::Vec3iSet& new_grid_indices)
{
    const eigen_utils::Vec3d box_search_margin(8.0, 8.0, 8.0);
    RelevantEdgeCostMap edge_costs;
    eigen_utils::Vec3iSet consider_grid_indices;

    collectRelevantGridsForEnduringEdgeUpdate(update_min, update_max, new_grid_indices,
                                              consider_grid_indices);

    for (const auto& consider_grid_index : consider_grid_indices)
    {
        collectIntraGridEnduringEdges(consider_grid_index, box_search_margin, edge_costs);
        collectNeighborGridEnduringEdges(consider_grid_index, data_.current_relevant_grid_,
                                         box_search_margin, edge_costs);
    }

    insertEnduringEdgesToRelevantGraph(edge_costs);
}

void DynamicExpandingGrid::updateRelevantGridGraphTemporaryEdges()
{
    data_.relevant_graph_->resetTemporaryEdges();

    std::vector<eigen_utils::Vec_Vec3d> centroid_clusters;
    std::vector<eigen_utils::Vec_Vec3d> exploring_centroid_clusters;
    collectRelevantCentroidClusters(centroid_clusters);

    // 2.If there are more than one exploring centroid clusters,
    //   add the edges between the exploring centroid clusters (exploring grid)
    if (centroid_clusters.size() < 2)
        return;

    collectExploringCentroidClusters(centroid_clusters, exploring_centroid_clusters);

    std::vector<CentroidPair> centroid_pairs;
    buildInterClusterCentroidPairs(exploring_centroid_clusters, centroid_pairs);

    eigen_utils::Vec3dMap<eigen_utils::Vec3dMap<double, 3>, 3> global_dist_map;
    computeTemporaryEdgeDistances(centroid_pairs, global_dist_map);
    insertTemporaryEdgesToRelevantGraph(global_dist_map);
}

void DynamicExpandingGrid::eraseIsolatedUnexploredGrid()
{
    // 1. Collect connected components from the relevant graph and identify
    //    those without any EXPLORING vertex (isolated unexplored).
    eigen_utils::Vec3iSet visited;
    eigen_utils::Vec_Vec3d isolated_centroids;

    eigen_utils::Vec3i pt_voxel_idx;
    for (const auto& [centroid, state] : data_.vertices_explore_state_)
    {
        sdf_map_->posToIndex(centroid, pt_voxel_idx);
        if (visited.find(pt_voxel_idx) != visited.end())
            continue;

        eigen_utils::Vec_Vec3d component;
        const bool has_exploring = collectConnectedComponent(centroid, component, visited);

        if (!has_exploring)
            isolated_centroids.insert(isolated_centroids.end(), component.begin(), component.end());
    }

    // 2. Remove isolated unexplored centroids from metadata and the relevant graph.
    for (const auto& centroid : isolated_centroids)
    {
        auto it = data_.grid_centroids_indices_.find(centroid);
        if (it != data_.grid_centroids_indices_.end())
        {
            const auto grid_index = it->second;
            const eigen_utils::Vec_Vec3d centroids = data_.grid_indices_centroids_.at(grid_index);

            if (centroids.size() > 1)
            {
                eigen_utils::Vec3dSet<3> new_centroids(centroids.begin(), centroids.end());
                new_centroids.erase(centroid);
                data_.grid_indices_centroids_.at(grid_index)
                    .assign(new_centroids.begin(), new_centroids.end());
            }
            else
            {
                data_.current_relevant_grid_.erase(grid_index);
                data_.grid_indices_centroids_.erase(grid_index);
            }
        }

        data_.grid_centroids_indices_.erase(centroid);
        data_.vertices_explore_state_.erase(centroid);
        data_.relevant_graph_->deleteVertex(
            data_.relevant_graph_->getVertexId(centroid.cast<float>()), true);
    }
}

} // namespace map_process
