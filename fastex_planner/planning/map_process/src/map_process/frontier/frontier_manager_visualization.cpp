/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-04-30
 * @Description:
 */

#include "fastex_msgs/frontierStatistics.h"
#include "map_process/frontier/frontier_manager.h"
#include <vis_utils/marker_utils.h>

namespace map_process
{
void FrontierManager::pubFrontierStatistics()
{
    fastex_msgs::frontierStatistics frontier_statistics;
    frontier_statistics.resolution = sdf_map_->getResolution();
    frontier_statistics.known_cell_num = simple_known_cells_.size();

    SDFMap_frontier_statistics_pub_.publish(frontier_statistics);
}

void FrontierManager::pubSimpleGlobalFrontiersMarkers()
{
    eigen_utils::Vec_Vec3d global_frontier_cells;
    eigen_utils::Vec3iMap<eigen_utils::Vec3d> global_normals;

    std::shared_lock<std::shared_mutex> lock(global_frontier_map_mutex_);
    for (const auto& [index, frontier] : global_frontier_map_)
    {
        global_frontier_cells.push_back(frontier.pos_);
        global_normals[index] = frontier.normal_;
    }
    lock.unlock();

    visualization_msgs::Marker global_frontier_cell_markers;
    generateMarkerArray(global_frontier_cell_markers, global_frontier_cells,
                        eigen_utils::Vec4d(0.5, 0.5, 0.5, 0.5));
    global_frontiers_pub_.publish(global_frontier_cell_markers);

    visualization_msgs::Marker local_frontier_normal_markers;
    generateNormalsMarkers(local_frontier_normal_markers, global_normals,
                           eigen_utils::Vec4d(1.0, 0.0, 0.0, 0.5));
    global_frontiers_pub_.publish(local_frontier_normal_markers);
}

void FrontierManager::generateMarkerArray(visualization_msgs::Marker& frontier_cell_markers,
                                          const eigen_utils::Vec_Vec3d& frontier_cells,
                                          const eigen_utils::Vec4d& rgba) const
{
    double size = sdf_map_->getResolution();
    vis_utils::marker_utils::configureMarker(frontier_cell_markers, frontier_params_.frame_id_,
                                  "frontier_cell_markers", 0, visualization_msgs::Marker::CUBE_LIST,
                                  vis_utils::marker_utils::makeScale(size, size, size),
                                  vis_utils::marker_utils::toRosColor(rgba),
                                  frontier_cells.empty() ? visualization_msgs::Marker::DELETE
                                                         : visualization_msgs::Marker::ADD);
    vis_utils::marker_utils::appendPoints(frontier_cell_markers, frontier_cells);
}

void FrontierManager::generateNormalsMarkers(
    visualization_msgs::Marker& normal_markers,
    const eigen_utils::Vec3iMap<eigen_utils::Vec3d> frontier_normals_map,
    const eigen_utils::Vec4d& rgba) const
{
    vis_utils::marker_utils::configureMarker(
        normal_markers, frontier_params_.frame_id_, "frontier_normal_markers", 0,
        visualization_msgs::Marker::LINE_LIST, vis_utils::marker_utils::makeScale(0.05, 0.0, 0.0),
        vis_utils::marker_utils::toRosColor(rgba));

    eigen_utils::Vec3d pos;
    for (const auto& [idx, normal] : frontier_normals_map)
    {
        sdf_map_->indexToPos(idx, pos);
        vis_utils::marker_utils::appendLine(normal_markers, pos, pos + normal);
    }
}

} // namespace map_process
