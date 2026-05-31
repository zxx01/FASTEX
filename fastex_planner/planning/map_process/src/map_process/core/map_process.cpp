/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-01 16:09:34
 * @LastEditTime: 2026-05-31 16:11:38
 * @Description:
 */

#include <thread>

#include <pcl_conversions/pcl_conversions.h>

#include "fastex_msgs/DataLog.h"
#include "file_utils/file_rw.h"
#include "map_process/core/map_process.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"

namespace map_process
{
namespace
{
const struct MapProcessTimerLogFlags
{
    bool update_frontiers_cluster{false};
    bool update_frontiers_viewpoints{false};
    bool update_frontiers_total{false};

    bool update_dynamic_expanding_grid{false};

    bool update_roadmap_fixed_points{false};
    bool update_roadmap_vertex_active_state{false};
    bool update_roadmap_vertex_state_and_edges{false};
    bool update_roadmap_sample_points{false};
    bool update_roadmap_total{false};

    bool update_history_position_graph{false};
    bool update_top_viewpoints_in_wsrm{false};
    bool update_relevant_grid_graph{false};
} kTimerLogFlags;

void publishStageLog(const ros::Publisher& publisher, time_utils::Timer& timer,
                     const int plan_iter_num)
{
    if (plan_iter_num <= 0)
        return;

    fastex_msgs::DataLog data_log_msg;
    data_log_msg.iteration_num = plan_iter_num;
    data_log_msg.start_time = file_utils::formatDouble(timer.getStartTime("us") / 1e6, 6);
    data_log_msg.end_time = file_utils::formatDouble(timer.getStopTime("us") / 1e6, 6);
    publisher.publish(data_log_msg);
}
} // namespace

MapProcess::MapProcess(ros::NodeHandle& nh, const fast_planner::EDTEnvironment::Ptr edt_env)
{
    openvdb::initialize();

    edt_env_ = edt_env;
    sdf_map_ = edt_env->sdf_map_;

    map_data_ = std::make_unique<MapKeyData>();
    map_data_->is_get_first_data_ = false;

    initElements(nh);

    sensor_cloud_buffer_ = std::make_unique<SensorCloudBuffer>();
    point_cloud_virtual_sub_ = nh.subscribe("/map_process/virtual_cloud", 10,
                                            &MapProcess::pointCloudVirtualCallback, this);

    odom_filter_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/map_process/odom", 25));
    point_cloud_real_filter_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, "/map_process/real_cloud", 25));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(100), *point_cloud_real_filter_sub_, *odom_filter_sub_));
    sync_cloud_odom_->registerCallback(
        boost::bind(&MapProcess::realCloudOdomCallback, this, _1, _2));

    // init runtime data publishers
    frontier_log_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/frontier_log", 10);
    dynamic_expanding_grid_log_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/dynamic_expanding_grid_log", 10);
    road_map_log_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/road_map_log", 10);

    // init visualization
    visualization_ = std::make_shared<fast_planner::PlanningVisualization>(nh);

    relevant_graph_marker_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("/planning_vis/relevant_graph_markers", 100);
    free_road_graph_marker_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>("/planning_vis/road_graph_markers", 100);
    whole_road_graph_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/planning_vis/whole_road_graph_markers", 100);
    history_pos_graph_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/planning_vis/history_pos_graph_marker", 100);
}

/**
 * @brief Initialize the elements of the map process
 *
 * @param nh ros node handle
 */
void MapProcess::initElements(ros::NodeHandle& nh)
{
    // init history pos graph
    history_pos_graph_ = std::make_shared<HistoryPosGraph>();

    // init frontier_manager
    frontier_manager_ = std::make_shared<map_process::FrontierManager>(nh, sdf_map_);
    frontier_manager_->setHistoryPosGraph(history_pos_graph_);

    // init DynamicExpandingGrid
    dynamic_expanding_grid_ = std::make_shared<map_process::DynamicExpandingGrid>(nh, edt_env_);
    dynamic_expanding_grid_->setFrontierManager(frontier_manager_);
    dynamic_expanding_grid_->setHistoryPosGraph(history_pos_graph_);

    // init roadmap
    eigen_utils::Vec3d bmin, bmax;
    sdf_map_->getBox(bmin, bmax);

    // init whole state roadmap
    whole_state_road_map_ = std::make_shared<WSRoadMap>(nh, sdf_map_, 3);
    whole_state_road_map_->setSamplePointsBound(bmax, bmin);
    frontier_manager_->setWholeStateRoadMap(whole_state_road_map_);
    dynamic_expanding_grid_->setWholeStateRoadMap(whole_state_road_map_);
}

/**
 * @brief Set the Initial Position of the robot
 *
 * @param initial_position initial position of the robot
 */
void MapProcess::setInitialPosition(const eigen_utils::Vec3d& initial_position)
{
    map_data_->initial_position_ = initial_position;
    whole_state_road_map_->setInitialPosition(initial_position.cast<float>());
}

/**
 * @brief Set the Current Position of the robot
 *
 * @param current_position current position of the robot
 */
void MapProcess::setCurrentPosition(const eigen_utils::Vec3d& current_position)
{
    map_data_->current_position_ = current_position;
    dynamic_expanding_grid_->setCurrentPosition(current_position);
    frontier_manager_->setCurrentPosition(current_position);
    whole_state_road_map_->setCurrentPosition(current_position.cast<float>());
}

/**
 * @brief Update the elements of the map process
 *
 */
bool MapProcess::updateElements(const eigen_utils::Vec3d& plan_position, const int& plan_iter_num)
{
    if (!sdf_map_->isSDFMapGot())
        return false;

    std::unique_lock<std::shared_mutex> lock(update_mutex_);
    MapProcessContext context(plan_position, plan_iter_num);
    runFrontierStage(context);
    runDynamicGridStage(context);
    runRoadmapStage(context);
    runHistoryStage(context);
    runRelevantGraphStage(context);
    return true;
}

void MapProcess::runFrontierStage(MapProcessContext& context)
{
    time_utils::Timer timer("Update Frontiers and Viewpoints");
    timer.start();

    time_utils::Timer timer_ft("Update Frontiers Cluster");
    timer_ft.start();

    frontier_manager_->acquireChangedCells(context.simple_changed_cells);

    if (!map_data_->is_get_first_data_)
    {
        sdf_map_->setChangeCollectionMode(sdf_map_->ONLY_UPDATED);
        map_data_->is_get_first_data_ = true;
    }

    sdf_map_->getUpdatedBox(map_data_->previous_perception_range_min_,
                            map_data_->previous_perception_range_max_, true);

    frontier_manager_->searchFrontiers(context.simple_changed_cells,
                                       map_data_->previous_perception_range_min_,
                                       map_data_->previous_perception_range_max_);
    timer_ft.stop(kTimerLogFlags.update_frontiers_cluster, "ms");

    time_utils::Timer timer_vp("Update Frontiers VPs");
    timer_vp.start();
    frontier_manager_->computeKeyViewpointsForClustersWithVisibleScore2();
    timer_vp.stop(kTimerLogFlags.update_frontiers_viewpoints, "ms");

    frontier_manager_->updateGlobalClusters();
    timer.stop(kTimerLogFlags.update_frontiers_total, "ms");
    publishStageLog(frontier_log_pub_, timer, context.plan_iter_num);
}

void MapProcess::runDynamicGridStage(MapProcessContext& context)
{
    time_utils::Timer timer("Update DynamicExpandingGrid");
    timer.start();

    sensor_cloud_buffer_->takeSnapshot(context.real_cloud_temp, context.virtual_cloud_temp,
                                       context.cumulative_pos_temp);

    context.deg_update_min = map_data_->previous_perception_range_min_ -
                             0.25 * dynamic_expanding_grid_->getParams().initial_resolution_;
    context.deg_update_max = map_data_->previous_perception_range_max_ +
                             0.25 * dynamic_expanding_grid_->getParams().initial_resolution_;

    dynamic_expanding_grid_->updateGridData(
        context.real_cloud_temp, context.virtual_cloud_temp, context.cumulative_pos_temp,
        context.simple_changed_cells, context.deg_update_min, context.deg_update_max);

    timer.stop(kTimerLogFlags.update_dynamic_expanding_grid, "ms");
    publishStageLog(dynamic_expanding_grid_log_pub_, timer, context.plan_iter_num);
}

void MapProcess::runRoadmapStage(MapProcessContext& context)
{
    time_utils::Timer timer("Update Roadmap");
    timer.start();

    time_utils::Timer timer_wsrm_partial("Update whole_state_road_map: FIXED POINTS");
    timer_wsrm_partial.start();

    dynamic_expanding_grid_->getStateChangeGridBoxes(
        context.new_boxes, context.change_to_explored_boxes, context.change_from_explored_boxes);

    if (!context.new_boxes.empty())
    {
        whole_state_road_map_->insertValidRegionBoxes(context.new_boxes);

        eigen_utils::Vec3f interval;
        eigen_utils::Vec_Vec3f samples;
        for (const auto& box : context.new_boxes)
        {
            eigen_utils::Vec_Vec3f sample_pts;
            std::tie(interval, sample_pts) =
                process_utils::ProcessUtils::samplePointsUniformCentered(
                    eigen_utils::Vec3i(2, 2, 2), box.min_.cast<float>(), box.max_.cast<float>());
            samples.insert(samples.end(), sample_pts.begin(), sample_pts.end());
        }

        whole_state_road_map_->addFixedPointsToGraph(samples, interval);
    }
    timer_wsrm_partial.stop(kTimerLogFlags.update_roadmap_fixed_points, "ms");

    timer_wsrm_partial =
        time_utils::Timer("Update whole_state_road_map: Update Vertices Active State");
    timer_wsrm_partial.start();

    for (const auto& box : context.change_to_explored_boxes)
        whole_state_road_map_->inactivateUnknownStateVerticesInBox(box.min_.cast<float>(),
                                                                   box.max_.cast<float>());

    for (const auto& box : context.change_from_explored_boxes)
        whole_state_road_map_->activateUnknownStateVerticesInBox(box.min_.cast<float>(),
                                                                 box.max_.cast<float>());

    timer_wsrm_partial.stop(kTimerLogFlags.update_roadmap_vertex_active_state, "ms");

    timer_wsrm_partial = time_utils::Timer(
        "Update whole_state_road_map: Update Vertices State and Edges Connection");
    timer_wsrm_partial.start();

    eigen_utils::Vec3f roadmap_update_min = map_data_->previous_perception_range_min_.cast<float>();
    eigen_utils::Vec3f roadmap_update_max = map_data_->previous_perception_range_max_.cast<float>();

    eigen_utils::Vec_Vec3f roadmap_pts;
    std::vector<int> roadmap_pt_indices;
    whole_state_road_map_->BoxNeighborSearch(roadmap_update_min, roadmap_update_max, roadmap_pts,
                                             roadmap_pt_indices);

    std::unordered_set<int> visited;
    for (const int pt_idx : roadmap_pt_indices)
    {
        const auto& link_edges = whole_state_road_map_->getLinkedEdges(pt_idx);
        for (const auto& [v_id, e_data] : link_edges)
        {
            if (visited.find(v_id) != visited.end())
                continue;

            visited.insert(v_id);
            const auto& vertex = whole_state_road_map_->getVertex(v_id);
            roadmap_update_min = roadmap_update_min.cwiseMin(vertex.pos_);
            roadmap_update_max = roadmap_update_max.cwiseMax(vertex.pos_);
        }
    }

    frontier_manager_->getActiveFrontierClusterBoxesWithinRange(
        process_utils::CubeBox(roadmap_update_min.cast<double>(),
                               roadmap_update_max.cast<double>()),
        context.frontiers_boxes);

    whole_state_road_map_->updateExistingVerticesStateAndEdgesConnection(
        map_data_->previous_perception_range_min_.cast<float>(),
        map_data_->previous_perception_range_max_.cast<float>(), context.frontiers_boxes);
    timer_wsrm_partial.stop(kTimerLogFlags.update_roadmap_vertex_state_and_edges, "ms");

    timer_wsrm_partial = time_utils::Timer("Update whole_state_road_map: Sample Points");
    timer_wsrm_partial.start();

    const eigen_utils::Vec3f box_margin(4.0, 4.0, 4.0);
    const eigen_utils::Vec3f sample_range_min =
        map_data_->previous_perception_range_min_.cast<float>() - box_margin;
    const eigen_utils::Vec3f sample_range_max =
        map_data_->previous_perception_range_max_.cast<float>() + box_margin;
    whole_state_road_map_->growingTopoGraphBySamplePointsWithinBox(sample_range_min,
                                                                   sample_range_max);
    timer_wsrm_partial.stop(kTimerLogFlags.update_roadmap_sample_points, "ms");

    timer.stop(kTimerLogFlags.update_roadmap_total, "ms");
    publishStageLog(road_map_log_pub_, timer, context.plan_iter_num);
}

void MapProcess::runHistoryStage(MapProcessContext& context) const
{
    time_utils::Timer timer("Update History Position Graph");
    timer.start();
    history_pos_graph_->tryToAddHistoryPos(context.plan_position.cast<float>());
    timer.stop(kTimerLogFlags.update_history_position_graph, "ms");
}

void MapProcess::runRelevantGraphStage(MapProcessContext& context)
{
    time_utils::Timer timer("updateTopViewpointsInWSRoadMap");
    timer.start();
    frontier_manager_->updateTopViewpointsInWSRoadMap(context.plan_position);
    timer.stop(kTimerLogFlags.update_top_viewpoints_in_wsrm, "ms");

    timer = time_utils::Timer("Update Relevant Grid Graph");
    timer.start();
    dynamic_expanding_grid_->updateRelevantGridGraph(context.deg_update_min,
                                                     context.deg_update_max);
    timer.stop(kTimerLogFlags.update_relevant_grid_graph, "ms");
}

/**
 * @brief Visualize the data of the map process
 *
 */
void MapProcess::visualizeData()
{
    // Draw updated box
    eigen_utils::Vec3d bmin_ud = map_data_->previous_perception_range_min_;
    eigen_utils::Vec3d bmax_ud = map_data_->previous_perception_range_max_;
    visualization_->drawBox((bmin_ud + bmax_ud) / 2.0, bmax_ud - bmin_ud,
                            Eigen::Vector4d(0, 1, 0, 0.2), "updated_box", 0, 4);

    // Draw dynamic expanding grid
    std::vector<eigen_utils::Vec_Vec3d> pts1_vec, pts2_vec;

    static size_t last_dynamic_expanding_grid_num = 0;
    dynamic_expanding_grid_->getAllGridMarkers(true, pts1_vec, pts2_vec);
    for (size_t i = 0; i < pts1_vec.size(); ++i)
        visualization_->drawLines(pts1_vec[i], pts2_vec[i], 0.2,
                                  visualization_->getColor(double(i) / pts1_vec.size()),
                                  "dynamic_expanding_grid", i, 7);
    for (size_t i = pts1_vec.size(); i < last_dynamic_expanding_grid_num; ++i)
        visualization_->drawLines({}, {}, 0.1, eigen_utils::Vec4d(0, 0, 0, 1),
                                  "dynamic_expanding_grid", i, 7);
    last_dynamic_expanding_grid_num = pts1_vec.size();

    // Draw relevant grid
    static size_t last_relevant_num = 0;
    dynamic_expanding_grid_->getRelevantGridMarkers(pts1_vec, pts2_vec);
    for (size_t i = 0; i < pts1_vec.size(); ++i)
        visualization_->drawLines(pts1_vec[i], pts2_vec[i], 0.8,
                                  eigen_utils::Vec4d(0.75, 0.75, 0.75, 0.8), "relevant_grid", i, 7);
    for (size_t i = pts1_vec.size(); i < last_relevant_num; ++i)
        visualization_->drawLines({}, {}, 0.1, eigen_utils::Vec4d(0, 0, 0, 1), "relevant_grid", i,
                                  7);
    last_relevant_num = pts1_vec.size();

    // Draw relevant graph
    visualization_msgs::MarkerArray relevant_graph_markers;
    dynamic_expanding_grid_->getRelevantGraphMarkers(
        relevant_graph_markers, 2.0, 0.2,
        {eigen_utils::Vec4d(0, 0, 0, 0.5), eigen_utils::Vec4d(1, 0, 0, 0.5)},
        {eigen_utils::Vec4d(0, 1, 0, 1)});
    relevant_graph_marker_pub_.publish(relevant_graph_markers);

    // Draw frontiers clusters cells, centroids and normals
    static size_t last_ft_num = 0;
    std::vector<eigen_utils::Vec_Vec3d> clustered_frontiers, clustered_viewpoints;
    eigen_utils::Vec_Vec3d clustered_centroids, clustered_normals;
    frontier_manager_->getGlobalClusteredFrontiers(clustered_frontiers, true);
    frontier_manager_->getGlobalClusteredCentroids(clustered_centroids, true);
    frontier_manager_->getGlobalClusteredNormals(clustered_normals, true);
    frontier_manager_->getGlobalClusteredViewpoints(clustered_viewpoints, true);
    for (size_t i = 0; i < clustered_frontiers.size(); ++i)
    {
        visualization_->drawCubes(
            clustered_frontiers[i], 0.4,
            visualization_->getColor(double(i) / clustered_frontiers.size(), 0.8), "frontier", i,
            4);
        visualization_->drawSpheres({clustered_centroids[i]}, 0.3, eigen_utils::Vec4d(0, 0, 1, 1),
                                    "frontier_centroid", i, 4);
        visualization_->drawLines(
            {clustered_centroids[i], clustered_centroids[i] + clustered_normals[i] * 3}, 0.2,
            eigen_utils::Vec4d(1, 0, 0, 1), "frontier_normal", i, 4);
        visualization_->drawSpheres(
            clustered_viewpoints[i], 0.5,
            visualization_->getColor(double(i) / clustered_viewpoints.size()), "points", i, 6);
    }
    for (size_t i = clustered_frontiers.size(); i < last_ft_num; ++i)
    {
        visualization_->drawCubes({}, 0.1, eigen_utils::Vec4d(0, 0, 0, 1), "frontier", i, 4);
        visualization_->drawSpheres({}, 0.3, eigen_utils::Vec4d(0, 0, 1, 1), "frontier_centroid", i,
                                    4);
        visualization_->drawLines({}, 0.2, eigen_utils::Vec4d(1, 0, 0, 1), "frontier_normal", i, 4);
        visualization_->drawSpheres({}, 0.1, eigen_utils::Vec4d(0, 0, 0, 1), "points", i, 6);
    }
    last_ft_num = clustered_frontiers.size();

    frontier_manager_->pubFrontierStatistics();

    // Draw road_map
    visualization_msgs::MarkerArray road_graph_markers;
    whole_state_road_map_->generateRoadGraphMarkers(road_graph_markers, 0.1, 0.05,
                                                    eigen_utils::Vec4d(1, 0.7, 0, 1),
                                                    eigen_utils::Vec4d(1.0, 0.0, 0.7, 0.5));
    whole_road_graph_marker_pub_.publish(road_graph_markers);

    // Draw history pos graph
    visualization_msgs::MarkerArray history_pos_graph_markers;
    history_pos_graph_->generateRoadGraphMarkers(history_pos_graph_markers, 0.2, 0.1,
                                                 eigen_utils::Vec4d(1, 0, 0, 1),
                                                 eigen_utils::Vec4d(0, 0, 1, 0.5));
    history_pos_graph_marker_pub_.publish(history_pos_graph_markers);

    // Draw min-max box
    eigen_utils::Vec3d bmin, bmax;
    sdf_map_->getBox(bmin, bmax);
    eigen_utils::Vec_Vec3d box_pts = process_utils::ProcessUtils::generateBoxCorners(bmin, bmax);
    std::array<int, 24> lines = {0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7,
                                 7, 6, 6, 4, 0, 4, 1, 5, 2, 6, 3, 7};
    eigen_utils::Vec_Vec3d start_pts, end_pts;
    for (size_t i = 0; i < lines.size(); i += 2)
    {
        start_pts.push_back(box_pts[lines[i]]);
        end_pts.push_back(box_pts[lines[i + 1]]);
    }
    visualization_->drawLines(start_pts, end_pts, 0.3, Eigen::Vector4d(0, 0, 0, 0.7), "min_max_box",
                              0, 7);
}

/**
 * @brief Point Cloud Callback function
 *
 * @param msg point cloud message
 */
void MapProcess::pointCloudRealCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert the PointCloud2 message to a PCL point cloud and crop it by the bounding box of the
    // SDF map
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    try
    {
        pcl::fromROSMsg(*msg, *cloud);

    } catch (const pcl::PCLException& e)
    {
        ROS_ERROR("Failed to convert ROS message to PCL point cloud: %s", e.what());
        return;
    }

    sensor_cloud_buffer_->appendRealCloud(cloud);
}

/**
 * @brief Point Cloud Callback function
 *
 * @param msg point cloud message
 */
void MapProcess::pointCloudVirtualCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Convert the PointCloud2 message to a PCL point cloud and crop it by the bounding box of the
    // SDF map
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    try
    {
        pcl::fromROSMsg(*msg, *cloud);

    } catch (const pcl::PCLException& e)
    {
        ROS_ERROR("Failed to convert ROS message to PCL point cloud: %s", e.what());
        return;
    }

    sensor_cloud_buffer_->appendVirtualCloud(cloud);
}

void MapProcess::realCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    // 1.Cumulate the cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    try
    {
        pcl::fromROSMsg(*msg, *cloud);

    } catch (const pcl::PCLException& e)
    {
        ROS_ERROR("Failed to convert ROS message to PCL point cloud: %s", e.what());
        return;
    }

    sensor_cloud_buffer_->appendRealCloud(cloud);
    sensor_cloud_buffer_->appendOdomPosition(eigen_utils::Vec3d(
        odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
}

} // namespace map_process
