#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <vis_utils/marker_utils.h>
#include <fstream>

namespace fast_planner
{
  MapROS::MapROS()
  {
  }

  MapROS::~MapROS()
  {
  }

  void MapROS::setMap(SDFMap *map)
  {
    this->map_ = map;
  }

  void MapROS::init()
  {
    node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
    node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
    node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
    node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
    node_.param("map_ros/show_all_map", show_all_map_, false);
    node_.param("map_ros/frame_id", frame_id_, std::string("world"));

    local_updated_ = false;
    esdf_need_update_ = false;
    esdf_time_ = 0.0;
    max_esdf_time_ = 0.0;
    esdf_num_ = 0;

    // lidar_.reset(new LidarModel(30.0, 2.0, -7.0, 52.0, 0.4, 0.0, 360.0));
    // lidar_.reset(new LidarModel(nh.param("UFOMap/no_ray_range", 30),
    //                             2.0, -7.0, 52.0, 0.4, 0.0, 360.0));

    esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
    vis_timer_ = node_.createTimer(ros::Duration(0.1), &MapROS::visCallback, this);

    map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
    map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
    map_local_inflate_pub_ =
        node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
    unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
    esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
    update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

    cloud_sub_.reset(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
    odom_sub_.reset(
        new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/map_ros/odom", 25));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudOdom>(
        MapROS::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&MapROS::cloudOdomCallback, this, _1, _2));

    map_start_time_ = ros::Time::now();
  }

  void MapROS::visCallback(const ros::TimerEvent &e)
  {
    publishMapLocal();
    if (show_all_map_)
    {
      // Limit the frequency of all map
      static double tpass = 0.0;
      tpass += (e.current_real - e.last_real).toSec();
      if (tpass > 0.5)
      {
        publishMapAll();
        tpass = 0.0;
      }
    }
    // publishUnknown();
    publishESDF();

    // publishUpdateRange();
  }

  void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/)
  {
    if (!esdf_need_update_)
      return;

    auto t1 = ros::Time::now();

    map_->updateESDF3d();

    esdf_need_update_ = false;

    auto t2 = ros::Time::now();
    esdf_time_ += (t2 - t1).toSec();
    max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
    esdf_num_++;
    if (show_esdf_time_)
      ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
               max_esdf_time_);
  }

  void MapROS::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg,
                                 const nav_msgs::OdometryConstPtr &odom)
  {
    std::lock_guard<std::mutex> lock(map_->sdf_mutex_);

    Eigen::Vector3d sensor_pos;
    sensor_pos(0) = odom->pose.pose.position.x;
    sensor_pos(1) = odom->pose.pose.position.y;
    sensor_pos(2) = odom->pose.pose.position.z;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *raw_cloud);
    int num = raw_cloud->points.size();

    map_->inputPointCloud(*raw_cloud, num, sensor_pos);

    if (local_updated_)
    {
      map_->clearAndInflateLocalMap();
      esdf_need_update_ = true;
      local_updated_ = false;
    }
  }

  void MapROS::publishMapAll()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
    if (map_->md_->sdf_map_got_)
    {
      for (int x = map_->md_->occupancy_min_i_[0]; x < map_->md_->occupancy_max_i_[0]; ++x)
        for (int y = map_->md_->occupancy_min_i_[1]; y < map_->md_->occupancy_max_i_[1]; ++y)
          for (int z = map_->md_->occupancy_min_i_[2]; z < map_->md_->occupancy_max_i_[2]; ++z)
          {
            if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
            {
              Eigen::Vector3d pos;
              map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
              if (pos(2) > visualization_truncate_height_)
                continue;
              if (pos(2) < visualization_truncate_low_)
                continue;
              pt.x = pos(0);
              pt.y = pos(1);
              pt.z = pos(2);
              cloud1.push_back(pt);
            }
          }
      cloud1.width = cloud1.points.size();
      cloud1.height = 1;
      cloud1.is_dense = true;
      cloud1.header.frame_id = frame_id_;
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud1, cloud_msg);
      map_all_pub_.publish(cloud_msg);
    }
  }

  void MapROS::publishMapLocal()
  {
    // local bounding box only constrains x and y; z keeps the original size
    // only publish occupied cells from the SDF
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    // obtain and clamp the local bounding box indices
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    // for (int z = min_cut(2); z <= max_cut(2); ++z)
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
          {
            // Occupied cells
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            // skip if beyond the truncation range
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
          else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
          {
            // Inflated occupied cells
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud2.push_back(pt);
          }
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;

    cloud2.width = cloud2.points.size();
    cloud2.height = 1;
    cloud2.is_dense = true;
    cloud2.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_local_pub_.publish(cloud_msg);
    pcl::toROSMsg(cloud2, cloud_msg);
    map_local_inflate_pub_.publish(cloud_msg);
  }

  void MapROS::publishUnknown()
  {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(max_cut);
    map_->boundIndex(min_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z)
        {
          if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
          {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > visualization_truncate_height_)
              continue;
            if (pos(2) < visualization_truncate_low_)
              continue;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
          }
        }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    unknown_pub_.publish(cloud_msg);
  }

  void MapROS::publishUpdateRange()
  {
    Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;
    map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
    map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

    cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
    cube_scale = esdf_max_pos - esdf_min_pos;

    auto color = vis_utils::marker_utils::toRosColor(Eigen::Vector4d(1.0, 0.0, 0.0, 0.3));
    auto scale = vis_utils::marker_utils::makeScale(cube_scale(0), cube_scale(1), cube_scale(2));
    mk = vis_utils::marker_utils::makeMarker(frame_id_, "", 0, visualization_msgs::Marker::CUBE,
                                             scale, color);
    vis_utils::marker_utils::appendPoint(mk, cube_pos);

    update_range_pub_.publish(mk);
  }

  void MapROS::publishESDF()
  {
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_,
                                                                            map_->mp_->local_map_margin_);
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
      {
        Eigen::Vector3d pos;
        map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
        pos(2) = esdf_slice_height_;
        dist = map_->getDistance(pos);
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = -0.2;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
      }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    esdf_pub_.publish(cloud_msg);

    // ROS_INFO("pub esdf");
  }
}
