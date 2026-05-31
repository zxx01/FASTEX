/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:34
 * @LastEditTime: 2026-05-30 22:37:05
 * @Description:
 */

#ifndef _SENSOR_CLOUD_BUFFER_H_
#define _SENSOR_CLOUD_BUFFER_H_

#include <memory>
#include <shared_mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_utils/eigen_utils.h"

namespace map_process
{
class SensorCloudBuffer
{
  public:
    using CloudPtr = pcl::PointCloud<pcl::PointXYZI>::Ptr;
    using UniquePtr = std::unique_ptr<SensorCloudBuffer>;

    SensorCloudBuffer();
    ~SensorCloudBuffer() = default;

    void appendRealCloud(const CloudPtr& cloud);
    void appendVirtualCloud(const CloudPtr& cloud);
    void appendOdomPosition(const eigen_utils::Vec3d& position);
    bool takeSnapshot(CloudPtr& real_cloud, CloudPtr& virtual_cloud,
                      eigen_utils::Vec_Vec3d& cumulative_pos);

  private:
    static void appendCloud(const CloudPtr& source, const float intensity, CloudPtr& target);

    CloudPtr cumulative_real_cloud_;
    CloudPtr cumulative_virtual_cloud_;
    eigen_utils::Vec_Vec3d cumulative_pos_;

    mutable std::shared_mutex cumulative_real_cloud_mutex_;
    mutable std::shared_mutex cumulative_virtual_cloud_mutex_;
    mutable std::shared_mutex cumulative_pos_mutex_;
};
} // namespace map_process

#endif // _SENSOR_CLOUD_BUFFER_H_
