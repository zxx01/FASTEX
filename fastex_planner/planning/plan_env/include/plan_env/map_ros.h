#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <ufomapping/lidar_model.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <string>

using std::shared_ptr;

namespace fast_planner
{
  class SDFMap;

  class MapROS
  {
  public:
    MapROS();
    ~MapROS();
    void setMap(SDFMap *map);
    void init();

  private:
    void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                           const nav_msgs::OdometryConstPtr &odom);
    void updateESDFCallback(const ros::TimerEvent & /*event*/);
    void visCallback(const ros::TimerEvent & /*event*/);

    void publishMapAll();
    void publishMapLocal();
    void publishESDF();
    void publishUpdateRange();
    void publishUnknown();

    SDFMap *map_;
    // synchronize pointcloud and odometry
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry>
        SyncPolicyCloudOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

    ros::NodeHandle node_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerCloudOdom sync_cloud_odom_;

    ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
        update_range_pub_;
    ros::Timer esdf_timer_, vis_timer_;

    // LidarModel::Ptr lidar_;

    std::string frame_id_;
    // msg publication
    double esdf_slice_height_;
    double visualization_truncate_height_, visualization_truncate_low_;
    bool show_esdf_time_;
    bool show_all_map_;

    // data
    // flags of map state
    bool local_updated_, esdf_need_update_;
    double esdf_time_, max_esdf_time_;
    int esdf_num_;

    ros::Time map_start_time_;

    friend SDFMap;
  };
}

#endif
