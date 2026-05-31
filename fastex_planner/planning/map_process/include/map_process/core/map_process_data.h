/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-10-15 11:18:32
 * @LastEditTime: 2025-10-28 11:18:45
 * @Description:
 */

#ifndef _MAP_PROCESS_DATA_H_
#define _MAP_PROCESS_DATA_H_

#include <unordered_map>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "process_utils/process_utils.h"

namespace map_process
{
struct MapKeyData
{
    bool is_get_first_data_;
    eigen_utils::Vec3d current_position_, initial_position_;

    // frontier data
    std::unordered_map<int, eigen_utils::Vec3d> cluster_centroids_;
    std::vector<eigen_utils::Vec_Vec3d> cluster_frontiers_;

    // view point data
    std::vector<std::pair<eigen_utils::Vec3d, double>> top_vpoints_;

    eigen_utils::Vec3d previous_perception_range_min_, previous_perception_range_max_;
};

// DynamicExpandingGrid <--> RoadMap
struct UnknownZoneData
{
    eigen_utils::Vec3d zone_centroid_;
    eigen_utils::Vec_Vec3d cluster_centroids_;
    std::unordered_set<int> voxel_addrs_;
    process_utils::CubeBox bbox_;

    UnknownZoneData() {};
    UnknownZoneData(const eigen_utils::Vec3d& zone_centroid,
                    const eigen_utils::Vec_Vec3d& cluster_centroids,
                    const std::unordered_set<int>& voxel_addrs, const process_utils::CubeBox& bbox)
        : zone_centroid_(zone_centroid), cluster_centroids_(cluster_centroids),
          voxel_addrs_(voxel_addrs), bbox_(bbox) {};
};
} // namespace map_process

#endif // !_MAP_PROCESS_DATA_H_