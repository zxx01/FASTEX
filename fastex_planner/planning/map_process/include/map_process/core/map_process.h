/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-01 16:09:15
 * @LastEditTime: 2026-05-30 14:34:05
 * @Description:
 */

#ifndef _MAP_PROCESS_H_
#define _MAP_PROCESS_H_

#include <memory>
#include <shared_mutex>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_utils/eigen_utils.h"
#include "map_process/core/map_process_data.h"
#include "map_process/core/sensor_cloud_buffer.h"
#include "map_process/frontier/frontier_manager.h"
#include "map_process/grid/dynamic_expanding_grid.h"
#include "map_process/roadmap/history_pos_graph.h"
#include "map_process/roadmap/whole_state_road_map.h"
#include "plan_env/edt_environment.h"
#include "plan_env/sdf_map.h"
#include "traj_utils/planning_visualization.h"

namespace map_process
{
class MapProcess
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<MapProcess>;
    using UniquePtr = std::unique_ptr<MapProcess>;

    MapProcess() {};
    MapProcess(ros::NodeHandle& nh, const fast_planner::EDTEnvironment::Ptr edt_env);
    ~MapProcess() {};

    void initElements(ros::NodeHandle& nh);

    void setInitialPosition(const eigen_utils::Vec3d& initial_position);
    void setCurrentPosition(const eigen_utils::Vec3d& current_position);
    bool updateElements(const eigen_utils::Vec3d& plan_position, const int& plan_iter_num);

    void visualizeData();

    const fast_planner::SDFMap::Ptr& getSdfMap() const { return sdf_map_; }
    const WSRoadMap::SharedPtr& getWholeStateRoadMap() const { return whole_state_road_map_; }
    const FrontierManager::SharedPtr& getFrontierManager() const { return frontier_manager_; }
    const map_process::DynamicExpandingGrid::SharedPtr& getDynamicExpandingGrid() const
    {
        return dynamic_expanding_grid_;
    }
    const HistoryPosGraph::SharedPtr& getHistoryPosGraph() const { return history_pos_graph_; }

  private:
    struct MapProcessContext
    {
        using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;

        explicit MapProcessContext(const eigen_utils::Vec3d& plan_position_input,
                                   const int plan_iter_num_input)
            : plan_position(plan_position_input), plan_iter_num(plan_iter_num_input),
              real_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>),
              virtual_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>)
        {
        }

        eigen_utils::Vec3d plan_position;
        int plan_iter_num;

        std::vector<int> simple_changed_cells;
        CloudPtr real_cloud_temp;
        CloudPtr virtual_cloud_temp;
        eigen_utils::Vec_Vec3d cumulative_pos_temp;

        eigen_utils::Vec3d deg_update_min;
        eigen_utils::Vec3d deg_update_max;

        std::vector<process_utils::CubeBox> new_boxes;
        std::vector<process_utils::CubeBox> change_to_explored_boxes;
        std::vector<process_utils::CubeBox> change_from_explored_boxes;
        std::vector<process_utils::CubeBox> frontiers_boxes;
    };

    void pointCloudRealCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void pointCloudVirtualCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void realCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const nav_msgs::OdometryConstPtr& odom);
    void runFrontierStage(MapProcessContext& context);
    void runDynamicGridStage(MapProcessContext& context);
    void runRoadmapStage(MapProcessContext& context);
    void runHistoryStage(MapProcessContext& context) const;
    void runRelevantGraphStage(MapProcessContext& context);

    // map process elements
    fast_planner::EDTEnvironment::Ptr edt_env_;
    fast_planner::SDFMap::Ptr sdf_map_;
    WSRoadMap::SharedPtr whole_state_road_map_;
    FrontierManager::SharedPtr frontier_manager_;
    map_process::DynamicExpandingGrid::SharedPtr dynamic_expanding_grid_;
    HistoryPosGraph::SharedPtr history_pos_graph_;

    // data
    std::unique_ptr<MapKeyData> map_data_;

    // For sensor cloud process
    SensorCloudBuffer::UniquePtr sensor_cloud_buffer_;
    ros::Subscriber point_cloud_real_sub_, point_cloud_virtual_sub_;

    using SyncPolicyCloudOdom =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        nav_msgs::Odometry>;
    using SynchronizerCloudOdom =
        std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>>;
    SynchronizerCloudOdom sync_cloud_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
        point_cloud_real_filter_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_filter_sub_;

    // mutex
    mutable std::shared_mutex update_mutex_;

    // Log data
    ros::Publisher frontier_log_pub_, dynamic_expanding_grid_log_pub_, road_map_log_pub_;

    // visualization
    std::shared_ptr<fast_planner::PlanningVisualization> visualization_;
    ros::Publisher free_road_graph_marker_pub_, whole_road_graph_marker_pub_,
        relevant_graph_marker_pub_, history_pos_graph_marker_pub_;
};
} // namespace map_process

#endif // _MAP_PROCESS_H_
