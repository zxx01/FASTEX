#include <mutex>

#include <pcl/filters/voxel_grid.h>

#include "map_process/core/sensor_cloud_buffer.h"

namespace map_process
{
SensorCloudBuffer::SensorCloudBuffer()
{
    cumulative_real_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cumulative_virtual_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void SensorCloudBuffer::appendCloud(const CloudPtr& source, const float intensity, CloudPtr& target)
{
    *target += *source;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(target);
    sor.setLeafSize(0.5f, 0.5f, 0.5f);
    sor.filter(*target);

    for (auto& pt : target->points)
        pt.intensity = intensity;
}

void SensorCloudBuffer::appendRealCloud(const CloudPtr& cloud)
{
    std::unique_lock<std::shared_mutex> lock(cumulative_real_cloud_mutex_);
    appendCloud(cloud, 0.0f, cumulative_real_cloud_);
}

void SensorCloudBuffer::appendVirtualCloud(const CloudPtr& cloud)
{
    std::unique_lock<std::shared_mutex> lock(cumulative_virtual_cloud_mutex_);
    appendCloud(cloud, -1.0f, cumulative_virtual_cloud_);
}

void SensorCloudBuffer::appendOdomPosition(const eigen_utils::Vec3d& position)
{
    std::unique_lock<std::shared_mutex> lock(cumulative_pos_mutex_);
    cumulative_pos_.push_back(position);
}

bool SensorCloudBuffer::takeSnapshot(CloudPtr& real_cloud, CloudPtr& virtual_cloud,
                                     eigen_utils::Vec_Vec3d& cumulative_pos)
{
    std::unique_lock<std::shared_mutex> real_cloud_lock(cumulative_real_cloud_mutex_,
                                                        std::defer_lock);
    std::unique_lock<std::shared_mutex> virtual_cloud_lock(cumulative_virtual_cloud_mutex_,
                                                           std::defer_lock);
    std::unique_lock<std::shared_mutex> pos_lock(cumulative_pos_mutex_, std::defer_lock);
    std::lock(real_cloud_lock, virtual_cloud_lock, pos_lock);

    if (cumulative_pos_.empty())
        return false;

    cumulative_real_cloud_.swap(real_cloud);
    cumulative_virtual_cloud_.swap(virtual_cloud);
    cumulative_pos_.swap(cumulative_pos);

    cumulative_real_cloud_->clear();
    cumulative_virtual_cloud_->clear();
    cumulative_pos_.clear();

    return true;
}
} // namespace map_process
