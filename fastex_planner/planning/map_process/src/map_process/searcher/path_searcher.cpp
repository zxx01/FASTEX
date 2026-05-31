/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-05-09 11:24:39
 * @LastEditTime: 2026-03-03 21:47:27
 * @Description:
 */

#include "map_process/searcher/path_searcher.h"

#include "map_process/core/map_process_constants.h"
#include "process_utils/process_utils.h"

namespace map_process
{
PathSearcher::PathSearcher(double vm, double am, double yd, double ydd, double alpha_dir,
                           double beta_dir)
{
    params_ = std::make_unique<PathSearcherParams>();
    params_->vm_ = vm;
    params_->am_ = am;
    params_->yd_ = yd;
    params_->ydd_ = ydd;
    params_->alpha_dir_ = alpha_dir;
    params_->beta_dir_ = beta_dir;

    ROS_WARN(
        "Path searcher parameters: vm: %f, am: %f, yd: %f, ydd: %f, alpha_dir: %f, beta_dir: %f",
        params_->vm_, params_->am_, params_->yd_, params_->ydd_, params_->alpha_dir_,
        params_->beta_dir_);
}

PathSearcher::PathSearcher(const ros::NodeHandle& nh,
                           const std::shared_ptr<fast_planner::EDTEnvironment> edt_env,
                           const std::shared_ptr<map_process::WSRoadMap> ws_road_map)
{
    params_ = std::make_unique<PathSearcherParams>();
    loadParamsFromROS(nh);

    sdf_map_ = edt_env->sdf_map_;
    ws_road_map_ = ws_road_map;

    double resolution = sdf_map_->getResolution();
    astar_ = std::make_unique<fast_planner::Astar>();
    astar_->init(nh, edt_env);
    astar_->setResolution(resolution);

    eigen_utils::Vec3d origin, size;
    sdf_map_->getRegion(origin, size);
    caster_ = std::make_unique<RayCaster>();
    caster_->setParams(resolution, origin);
}

/**
 * @brief Load parameters from ROS
 *
 * @param nh ROS node handle
 */
void PathSearcher::loadParamsFromROS(const ros::NodeHandle& nh)
{
    bool is_param_load = true;

    is_param_load &= nh.param("exploration/vm", params_->vm_, -1.0);
    is_param_load &= nh.param("exploration/am", params_->am_, -1.0);
    is_param_load &= nh.param("exploration/yd", params_->yd_, -1.0);
    is_param_load &= nh.param("exploration/ydd", params_->ydd_, -1.0);
    is_param_load &= nh.param("exploration/alpha_dir", params_->alpha_dir_, -1.0);
    is_param_load &= nh.param("exploration/beta_dir", params_->beta_dir_, -1.0);

    if (!is_param_load)
        ROS_ERROR("Failed to load parameters for path searcher");
}

/**
 * @brief Check if a straight line between two points is collision-free.
 *
 * @param p1  Start point
 * @param p2  End point
 * @param optimistic  Whether to treat UNKNOWN cells as free
 * @return true if the straight line is collision-free
 */
bool PathSearcher::isCollisionFreeStraightLine(const eigen_utils::Vec3d& p1,
                                               const eigen_utils::Vec3d& p2,
                                               const bool optimistic) const
{
    eigen_utils::Vec3i idx;
    caster_->input(p1, p2);
    while (caster_->nextId(idx))
    {
        if (!sdf_map_->isInBox(idx) || sdf_map_->getInflateOccupancy(idx) == 1 ||
            (!optimistic && sdf_map_->getOccupancy(idx) == fast_planner::SDFMap::UNKNOWN))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Search a fine path between two points
 *
 * @param p1  Start point
 * @param p2  End point
 * @param path  Path between two points
 * @param distance  Length of the path
 * @param resolution  Resolution of the path
 * @param optimistic  whether to use Optimistic search
 * @return PATH_SEARCH_RESULT Result of the search
 */
PATH_SEARCH_RESULT PathSearcher::searchFinePath(const eigen_utils::Vec3d& p1,
                                                const eigen_utils::Vec3d& p2,
                                                eigen_utils::Vec_Vec3d& path, double& distance,
                                                const double resolution, const bool optimistic)
{
    path.clear();

    // 1. Try connect two points with straight line
    if (isCollisionFreeStraightLine(p1, p2, optimistic))
    {
        path = {p1, p2};
        distance = (p1 - p2).norm();
        return PATH_SEARCH_RESULT::SUCCESS;
    }

    // 2. Search a path using Astar
    astar_->reset();
    astar_->setResolution(resolution > 0 ? resolution : sdf_map_->getResolution());

    if (astar_->search(p1, p2, optimistic) == fast_planner::Astar::REACH_END)
    {
        path = astar_->getPath();
        distance = astar_->pathLength(path);
        return PATH_SEARCH_RESULT::SUCCESS;
    }

    // 3. Fallback: use Euclidean distance as estimate
    path = {p1, p2};
    distance = constants::kPathCostFallback;
    return PATH_SEARCH_RESULT::FAIL;
}

/**
 * @brief Search a fine path between two points with bounded region
 *
 * @param p1 Start point
 * @param p2 End point
 * @param min_bound Minimum bound of the region
 * @param max_bound Maximum bound of the region
 * @param path Path between two points
 * @param distance Length of the path
 * @param resolution Resolution of the path
 * @param optimistic Whether to use Optimistic search
 * @return PATH_SEARCH_RESULT Result of the search
 */
PATH_SEARCH_RESULT PathSearcher::searchFineBoundedPath(
    const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2, const eigen_utils::Vec3d& min_bound,
    const eigen_utils::Vec3d& max_bound, eigen_utils::Vec_Vec3d& path, double& distance,
    const double resolution, const bool optimistic)
{
    path.clear();

    // 1. Check if the two points are in the bounded region
    if (!process_utils::ProcessUtils::isInBox(p1, min_bound, max_bound) ||
        !process_utils::ProcessUtils::isInBox(p2, min_bound, max_bound))
    {
        return PATH_SEARCH_RESULT::FAIL;
    }

    // 2. Try connect two points with straight line
    if (isCollisionFreeStraightLine(p1, p2, optimistic))
    {
        path = {p1, p2};
        distance = (p1 - p2).norm();
        return PATH_SEARCH_RESULT::SUCCESS;
    }

    // 3. Search a path using Astar with bounding box constraint
    astar_->reset();
    astar_->setResolution(resolution > 0 ? resolution : sdf_map_->getResolution());
    if (astar_->boundedSearch(p1, p2, min_bound, max_bound, optimistic) ==
        fast_planner::Astar::REACH_END)
    {
        path = astar_->getPath();
        distance = astar_->pathLength(path);
        return PATH_SEARCH_RESULT::SUCCESS;
    }

    // 4. Fallback
    path = {p1, p2};
    distance = constants::kPathCostFallback;
    return PATH_SEARCH_RESULT::FAIL;
}

/**
 * @brief Searches for a coarse path from point p1 to point p2 on the WS road map.
 *
 * This function attempts to find the shortest path between two points on a WS road map.
 * If a path is found, it can optionally be optimized to a straight line if the
 * max_straight_distance parameter is greater than zero.
 *
 * @param p1 The starting point, a 3D vector of type eigen_utils::Vec3d.
 * @param p2 The ending point, a 3D vector of type eigen_utils::Vec3d.
 * @param path A vector to store the found path, of type eigen_utils::Vec_Vec3d.
 * @param distance A double to store the total distance of the path.
 * @param optimistic A boolean value to control the path optimization and search strategy.
 * @param max_straight_distance The maximum distance for straight path optimization, of type double.
 * @return PATH_SEARCH_RESULT Returns the result of the path search.
 *         Returns PATH_SEARCH_RESULT::SUCCESS if successful, otherwise returns
 * PATH_SEARCH_RESULT::FAIL.
 */
PATH_SEARCH_RESULT PathSearcher::searchCoarsePathWithWSRoadMap(
    const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2, eigen_utils::Vec_Vec3d& path,
    double& distance, const bool optimistic, const double max_straight_distance)
{
    path.clear();

    eigen_utils::Vec_Vec3f pathf;
    bool path_find = ws_road_map_->findShortestPath(p1.cast<float>(), p2.cast<float>(), optimistic,
                                                    pathf, distance);

    if (path_find)
    {
        // Optimize the path to a straight line if max_straight_distance is greater than zero
        if (max_straight_distance > 0)
        {
            optimizePathWithInterpolation(pathf, max_straight_distance, 2.0, optimistic);

            distance = computePathLength(pathf);
        }

        for (const auto& p : pathf)
            path.push_back(p.cast<double>());

        return PATH_SEARCH_RESULT::SUCCESS;
    }
    else
    {
        distance = constants::kPathCostFallback;
        return PATH_SEARCH_RESULT::FAIL;
    }
}

/**
 * @brief Searches for a coarse path from point p1 to point p2 within a specified bounding box on
 * the WS road map.
 *
 * This function attempts to find the shortest path between two points within a given bounding box
 * on a WS road map. If a path is found, it can optionally be optimized to a straight line if the
 * max_straight_distance parameter is greater than zero.
 *
 * @param p1 The starting point, a 3D vector of type eigen_utils::Vec3d.
 * @param p2 The ending point, a 3D vector of type eigen_utils::Vec3d.
 * @param path A vector to store the found path, of type eigen_utils::Vec_Vec3d.
 * @param distance A double to store the total distance of the path.
 * @param min_bound The minimum bound of the search box, a 3D vector of type eigen_utils::Vec3d.
 * @param max_bound The maximum bound of the search box, a 3D vector of type eigen_utils::Vec3d.
 * @param optimistic A boolean value to control the path optimization and search strategy.
 * @param max_straight_distance The maximum distance for straight path optimization, of type double.
 * @return PATH_SEARCH_RESULT Returns the result of the path search.
 *         Returns PATH_SEARCH_RESULT::SUCCESS if successful, otherwise returns
 * PATH_SEARCH_RESULT::FAIL.
 */
PATH_SEARCH_RESULT PathSearcher::searchCoarsePathWithWSRoadMapWithinBox(
    const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2, eigen_utils::Vec_Vec3d& path,
    double& distance, const eigen_utils::Vec3d& min_bound, const eigen_utils::Vec3d& max_bound,
    const bool optimistic, const double max_straight_distance)
{
    path.clear();

    eigen_utils::Vec_Vec3f pathf;
    process_utils::CubeBox confined_box(min_bound, max_bound);
    bool path_find = ws_road_map_->findShortestPathWithinBox(
        p1.cast<float>(), p2.cast<float>(), confined_box, optimistic, pathf, distance);

    if (path_find)
    {
        // Optimize the path to a straight line if max_straight_distance is greater than zero
        if (max_straight_distance > 0)
        {
            optimizePathWithInterpolation(pathf, max_straight_distance,
                                          constants::kPathInterpolationStep, optimistic);

            distance = computePathLength(pathf);
        }

        for (const auto& p : pathf)
            path.push_back(p.cast<double>());

        return PATH_SEARCH_RESULT::SUCCESS;
    }
    else
    {
        distance = constants::kPathCostFallback;
        return PATH_SEARCH_RESULT::FAIL;
    }
}

/**
 * @brief Compute the cost between two points
 *
 * @param p1 Start point
 * @param p2 End point
 * @param y1 Start yaw
 * @param y2 End yaw
 * @param v1 Start velocity
 * @param yd1 Start yaw rate
 * @param path Path between two points
 * @return double Cost between two points
 */
double PathSearcher::calculateMovementCost(const eigen_utils::Vec3d& p1,
                                           const eigen_utils::Vec3d& p2, const double& y1,
                                           const double& y2, const eigen_utils::Vec3d& v1,
                                           const double& yd1, eigen_utils::Vec_Vec3d& path,
                                           const PATH_SEARCH_TYPE& search_type,
                                           const bool optimistic)
{
    // 1.If the two points are the same, return 0
    if (p1.isApprox(p2, 1e-3))
    {
        path.push_back(p1);
        return 0;
    }

    // 2.Cost of position change
    double dist;
    PATH_SEARCH_RESULT result = PATH_SEARCH_RESULT::FAIL;
    if (search_type == PATH_SEARCH_TYPE::FINE)
        result = searchFinePath(p1, p2, path, dist, -1, optimistic);

    if (search_type == PATH_SEARCH_TYPE::COARSE || result == PATH_SEARCH_RESULT::FAIL)
        result = searchCoarsePathWithWSRoadMap(p1, p2, path, dist, optimistic, 5.0);

    if (result == PATH_SEARCH_RESULT::FAIL)
        return 1e3;

    double pos_cost = dist / params_->vm_;

    // Consider velocity change
    if (v1.norm() > 1e-3)
    {
        eigen_utils::Vec3d pdir = (p2 - p1).normalized();
        eigen_utils::Vec3d vdir = v1.normalized();
        double diff = std::acos(vdir.dot(pdir));
        pos_cost *= (1 + params_->alpha_dir_ * std::tanh(params_->beta_dir_ * diff));
    }

    // 3.Cost of yaw change
    double diff = std::fabs(y2 - y1);
    diff = std::min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / params_->yd_;

    return std::max(pos_cost, yaw_cost);
}

/**
 * @brief Calculate the cost of moving along a single path
 *
 * @param path Path
 * @param v1 Start velocity
 * @param y1 Start yaw
 * @param y2 End yaw
 * @return double Cost of moving along the path
 */
double PathSearcher::calculateMovementCostAlongSinglePath(
    const eigen_utils::Vec_Vec3d& path, const std::optional<eigen_utils::Vec3d>& v1,
    const std::optional<double> y1, const std::optional<double> y2)
{
    // If the path only has one point, return 0
    if (path.size() < 2)
        return 0.0;

    // Compute the time to move along the path
    eigen_utils::Vec3d dir = (path.back() - path.front()).normalized();
    eigen_utils::Vec3d cur_vel = v1.value_or(params_->vm_ * dir);

    double v1_along = cur_vel.dot(
        dir); // cur_vel along the path (positive or negative, maybe a little bigger than vm)
    double v1_perp = std::sqrt(std::max(cur_vel.squaredNorm() - v1_along * v1_along,
                                        0.0)); // cur_vel perpendicular to the path (positive)

    double dist_perp = 2 * v1_perp * v1_perp / (2 * params_->am_);
    double dist_total = astar_->pathLength(path) + dist_perp;
    double dist_acc =
        std::max((params_->vm_ * params_->vm_ - v1_along * v1_along), 0.0) / (2 * params_->am_);

    double t_pos = std::max(params_->vm_ - std::fabs(v1_along), 0.0) / params_->am_ +
                   2 * process_utils::ProcessUtils::stepFunction(-1.0 * v1_along) *
                       std::min(std::fabs(v1_along), params_->vm_) / params_->am_ +
                   process_utils::ProcessUtils::stepFunction(dist_total - dist_acc) *
                       (dist_total - dist_acc) / (params_->vm_);

    // Compute the time to rotate along the path
    double yaw_origin = y1.value_or(process_utils::ProcessUtils::calculateYaw(dir));
    double yaw_dest = y2.value_or(process_utils::ProcessUtils::calculateYaw(dir));
    double theta = std::fabs(yaw_dest - yaw_origin);
    theta = std::min(theta, 2 * M_PI - theta);
    double t_rot = theta / params_->yd_;

    return std::max(t_pos, t_rot);
}

/**
 * @brief Calculate the cost of moving along the path, which is partitioned into segments
 *
 * @param path Path
 * @param v1 Start velocity
 * @param y1 Start yaw
 * @param y2 End yaw
 * @return double Cost of moving along the path segments
 */
double PathSearcher::calculateMovementCostWithPathSegments(
    const eigen_utils::Vec_Vec3d& path, const double segment_len,
    const std::optional<eigen_utils::Vec3d>& v1, const std::optional<double> y1,
    const std::optional<double> y2)
{
    // If the path only has one point, return 0
    if (path.size() < 2)
        return 0.0;

    // Partition the path into segments with length threshold. Each segment contains at least two
    // points.
    std::vector<eigen_utils::Vec_Vec3d> path_segments;
    process_utils::ProcessUtils::partitionPath(path, segment_len, path_segments);

    // Compute the cost of moving along the path segments
    double time_cost = 0.0;

    eigen_utils::Vec3d v_origin;
    for (size_t i = 0; i < path_segments.size(); ++i)
    {
        const eigen_utils::Vec_Vec3d& cur_segment = path_segments[i];
        const eigen_utils::Vec_Vec3d& last_segment =
            (i == 0) ? path_segments[i] : path_segments[i - 1];
        const eigen_utils::Vec3d cur_dir = (cur_segment.back() - cur_segment.front()).normalized();

        // Compute the origin velocity for the first segment
        if (i == 0)
            v_origin = v1.value_or(params_->vm_ * cur_dir);

        // Compute the yaw angle of the segment
        double yaw_origin = (i == 0 && y1.has_value())
                                ? y1.value()
                                : process_utils::ProcessUtils::calculateYaw(last_segment.front(),
                                                                            last_segment.back());
        double yaw_dest = (i == path_segments.size() - 1 && y2.has_value())
                              ? y2.value()
                              : process_utils::ProcessUtils::calculateYaw(cur_segment.front(),
                                                                          cur_segment.back());

        double time =
            calculateMovementCostAlongSinglePath(cur_segment, v_origin, yaw_origin, yaw_dest);
        time_cost += time;

        // Update the origin velocity for the next segment
        if (i < path_segments.size() - 1)
        {
            double v_origin_along = v_origin.dot(cur_dir);
            double v_dest_along = std::min(v_origin_along + params_->am_ * time, params_->vm_);
            v_origin = v_dest_along * cur_dir;
        }
    }

    return time_cost;
}

/**
 * @brief Shorten the path by removing redundant waypoints.
 *
 * This method iterates through the path and attempts to skip waypoints if a direct, collision-free
 * line exists between the previous determined waypoint and the next one. Waypoints are preserved
 * if the segment length exceeds the distance threshold or if obstacles are detected.
 *
 * @param path The 3D path to be shortened (modified in place).
 * @param dist_thresh Maximum allowed distance between waypoints to maintain path density.
 */
void PathSearcher::shortenPath(eigen_utils::Vec_Vec3d& path, const double dist_thresh)
{
    if (path.empty())
    {
        ROS_WARN("Empty path to shorten");
        return;
    }

    if (path.size() <= 2)
        return;

    const double dist_thresh_sq = dist_thresh * dist_thresh;
    eigen_utils::Vec_Vec3d new_path;

    new_path.reserve(path.size());
    new_path.push_back(path.front());

    for (size_t i = 1; i < path.size() - 1; ++i)
    {
        // Add waypoints to shorten path if the distance between
        // the last added waypoint and the current waypoint is larger than the threshold
        if ((path[i] - new_path.back()).squaredNorm() > dist_thresh_sq)
        {
            new_path.push_back(path[i]);
        }
        else
        {
            Eigen::Vector3i idx;
            // Add waypoints to shorten path only to avoid collision
            caster_->input(new_path.back(), path[i + 1]);
            while (caster_->nextId(idx))
            {
                if (sdf_map_->getInflateOccupancy(idx) == 1 ||
                    sdf_map_->getOccupancy(idx) == fast_planner::SDFMap::UNKNOWN)
                {
                    new_path.push_back(path[i]);
                    break;
                }
            }
        }
    }

    if ((path.back() - new_path.back()).squaredNorm() > 1e-6)
        new_path.push_back(path.back());

    if (new_path.size() == 2)
        new_path.insert(new_path.begin() + 1, 0.5 * (new_path[0] + new_path[1]));

    path.swap(new_path);
}

} // namespace map_process
