/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-05-09 11:14:59
 * @LastEditTime: 2026-03-03 21:56:18
 * @Description:
 */

#ifndef _PATH_SEARCHER_H_
#define _PATH_SEARCHER_H_

#include <memory>
#include <optional>

#include <ros/ros.h>

#include "common_utils/eigen_utils.h"
#include "map_process/core/map_process_params.h"
#include "map_process/roadmap/whole_state_road_map.h"
#include "path_searching/astar2.h"
#include "plan_env/edt_environment.h"
#include "plan_env/raycast.h"
#include "plan_env/sdf_map.h"

namespace map_process
{
enum class PATH_SEARCH_TYPE : int8_t
{
    COARSE,
    FINE
};

enum class PATH_SEARCH_RESULT : int8_t
{
    SUCCESS,
    FAIL
};

class PathSearcher
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<PathSearcher>;
    using UniquePtr = std::unique_ptr<PathSearcher>;

    PathSearcher(/* args */) {};
    PathSearcher(double vm, double am, double yd, double ydd, double alpha_dir, double beta_dir);
    PathSearcher(const ros::NodeHandle& nh,
                 const std::shared_ptr<fast_planner::EDTEnvironment> edt_env,
                 const std::shared_ptr<map_process::WSRoadMap> ws_road_map);
    ~PathSearcher() {};

    PATH_SEARCH_RESULT searchFinePath(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2,
                                      eigen_utils::Vec_Vec3d& path, double& distance,
                                      const double resolution = -1, const bool optimistic = false);
    PATH_SEARCH_RESULT
    searchFineBoundedPath(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2,
                          const eigen_utils::Vec3d& min_bound, const eigen_utils::Vec3d& max_bound,
                          eigen_utils::Vec_Vec3d& path, double& distance,
                          const double resolution = -1, const bool optimistic = false);
    PATH_SEARCH_RESULT searchCoarsePathWithWSRoadMap(const eigen_utils::Vec3d& p1,
                                                     const eigen_utils::Vec3d& p2,
                                                     eigen_utils::Vec_Vec3d& path, double& distance,
                                                     const bool optimistic = false,
                                                     const double max_straight_distance = -1.0);
    PATH_SEARCH_RESULT searchCoarsePathWithWSRoadMapWithinBox(
        const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2, eigen_utils::Vec_Vec3d& path,
        double& distance, const eigen_utils::Vec3d& min_bound, const eigen_utils::Vec3d& max_bound,
        const bool optimistic = false, const double max_straight_distance = -1.0);

    double calculateMovementCost(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2,
                                 const double& y1, const double& y2, const eigen_utils::Vec3d& v1,
                                 const double& yd1, eigen_utils::Vec_Vec3d& path,
                                 const PATH_SEARCH_TYPE& search_type = PATH_SEARCH_TYPE::FINE,
                                 const bool optimistic = false);

    double
    calculateMovementCostAlongSinglePath(const eigen_utils::Vec_Vec3d& path,
                                         const std::optional<eigen_utils::Vec3d>& v1 = std::nullopt,
                                         const std::optional<double> y1 = std::nullopt,
                                         const std::optional<double> y2 = std::nullopt);
    double calculateMovementCostWithPathSegments(
        const eigen_utils::Vec_Vec3d& path, const double segment_len = 5.0,
        const std::optional<eigen_utils::Vec3d>& v1 = std::nullopt,
        const std::optional<double> y1 = std::nullopt,
        const std::optional<double> y2 = std::nullopt);

    template <typename EIGEN_POINT_VECTOR> double computePathLength(const EIGEN_POINT_VECTOR& path);

    void shortenPath(eigen_utils::Vec_Vec3d& path, const double dist_thresh);

    template <typename EIGEN_VECTOR_TYPE>
    void optimizePathToStraight(const std::vector<EIGEN_VECTOR_TYPE>& raw_path,
                                std::vector<EIGEN_VECTOR_TYPE>& optimized_path,
                                const double max_straight_distance, const bool optimistic = false);

    template <typename EIGEN_VECTOR_TYPE>
    static void interpolatePath(const std::vector<EIGEN_VECTOR_TYPE>& raw_path,
                                std::vector<EIGEN_VECTOR_TYPE>& interpolated_path,
                                const double step_size);

    template <typename EIGEN_VECTOR_TYPE>
    void optimizePathWithInterpolation(std::vector<EIGEN_VECTOR_TYPE>& path,
                                       const double max_straight_distance,
                                       const double interpolation_step = 2.0,
                                       const bool optimistic = false);

    const PathSearcherParams& getParams() const { return *params_; }
    const std::shared_ptr<fast_planner::SDFMap>& getSdfMap() const { return sdf_map_; }

  private:
    void loadParamsFromROS(const ros::NodeHandle& nh);
    bool isCollisionFreeStraightLine(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2,
                                     const bool optimistic) const;

    std::unique_ptr<PathSearcherParams> params_;
    map_process::WSRoadMap::SharedPtr ws_road_map_;
    std::shared_ptr<fast_planner::SDFMap> sdf_map_;

    std::unique_ptr<RayCaster> caster_;
    std::unique_ptr<fast_planner::Astar> astar_;
};
} // namespace map_process

namespace map_process
{
template <typename EIGEN_POINT_VECTOR>
inline double PathSearcher::computePathLength(const EIGEN_POINT_VECTOR& path)
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        length += (path[i] - path[i - 1]).norm();
    return length;
}

/**
 * @brief Optimizes a given path to be as straight as possible within a specified maximum straight
 * distance.
 *
 * This function takes a raw path and attempts to optimize it by making it as straight as possible,
 * while ensuring that the straight segments do not exceed the specified maximum straight distance
 * and are collision-free according to the provided SDF map.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the vector elements in the path, typically a 3D vector.
 * @param sdf_map A shared pointer to the SDF map used for collision checking.
 * @param raw_path The original path to be optimized, represented as a vector of EIGEN_VECTOR_TYPE.
 * @param optimized_path The resulting optimized path, represented as a vector of EIGEN_VECTOR_TYPE.
 * @param max_straight_distance The maximum allowable distance for straight segments in the
 * optimized path.
 * @param optimistic Whether to use optimistic collision checking.
 */
template <typename EIGEN_VECTOR_TYPE>
inline void PathSearcher::optimizePathToStraight(const std::vector<EIGEN_VECTOR_TYPE>& raw_path,
                                                 std::vector<EIGEN_VECTOR_TYPE>& optimized_path,
                                                 const double max_straight_distance,
                                                 const bool optimistic)
{
    if (raw_path.size() < 3)
    {
        optimized_path = raw_path;
        return;
    }

    if (max_straight_distance <= 0.0)
    {
        optimized_path = raw_path;
        return;
    }

    const int n = static_cast<int>(raw_path.size());
    int current_idx = 0;
    const double max_distance_squared = max_straight_distance * max_straight_distance;

    optimized_path = {raw_path[current_idx]};

    while (current_idx < n - 1)
    {
        int furthest_idx = -1;
        eigen_utils::Vec3d start(raw_path[current_idx].x(), raw_path[current_idx].y(),
                                 raw_path[current_idx].z());

        // Collision feasibility over waypoint index is not monotonic in general, so we must scan.
        for (int cand_idx = current_idx + 1; cand_idx < n; ++cand_idx)
        {
            auto diff = raw_path[cand_idx] - raw_path[current_idx];
            double distance_squared = diff.squaredNorm();

            if (distance_squared > max_distance_squared)
            {
                continue;
            }

            eigen_utils::Vec3d end(raw_path[cand_idx].x(), raw_path[cand_idx].y(),
                                   raw_path[cand_idx].z());

            fast_planner::SDFMap::CollisionCheckResult result;
            if (optimistic)
                result = sdf_map_->isOptimisticInflatedCollisionFreeStraight(start, end);
            else
                result = sdf_map_->isStrictInflatedCollisionFreeStraight(start, end);

            if (result.is_collision_free)
            {
                // Keep the furthest feasible index.
                furthest_idx = cand_idx;
            }
        }

        if (furthest_idx < 0)
        {
            // No feasible shortcut from current_idx exists; keep original progress.
            furthest_idx = current_idx + 1;
        }

        optimized_path.push_back(raw_path[furthest_idx]);
        current_idx = furthest_idx;
    }
}

/**
 * @brief Interpolates a given path by adding intermediate points between consecutive waypoints.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the vector elements in the path, typically a 3D vector.
 * @param raw_path The original path to be interpolated, represented as a vector of
 * EIGEN_VECTOR_TYPE.
 * @param interpolated_path The resulting interpolated path, represented as a vector of
 * EIGEN_VECTOR_TYPE.
 * @param step_size The distance between consecutive points in the interpolated path.
 */
template <typename EIGEN_VECTOR_TYPE>
inline void PathSearcher::interpolatePath(const std::vector<EIGEN_VECTOR_TYPE>& raw_path,
                                          std::vector<EIGEN_VECTOR_TYPE>& interpolated_path,
                                          const double step_size)
{
    interpolated_path.clear();

    if (raw_path.size() <= 1)
    {
        interpolated_path = raw_path;
        return;
    }

    if (step_size <= 0.0)
    {
        interpolated_path = raw_path;
        return;
    }

    for (size_t i = 0; i < raw_path.size() - 1; ++i)
    {
        const EIGEN_VECTOR_TYPE& start = raw_path[i];
        const EIGEN_VECTOR_TYPE& end = raw_path[i + 1];
        interpolated_path.push_back(start);

        EIGEN_VECTOR_TYPE segment = end - start;
        double distance = segment.norm();

        // Skip degenerate segments to avoid normalizing zero-length vectors.
        if (distance <= 1e-9)
            continue;

        EIGEN_VECTOR_TYPE direction = segment / distance;
        for (double traveled = step_size; traveled < distance; traveled += step_size)
            interpolated_path.push_back(start + direction * traveled);
    }

    interpolated_path.push_back(raw_path.back());
}

/**
 * @brief Smooth and simplify a path by interpolation and straight-line shortcutting.
 *
 * The processing pipeline is:
 * 1) interpolate the raw path with `interpolation_step`,
 * 2) run `optimizePathToStraight` with `max_straight_distance`,
 * 3) interpolate again with the same step.
 *
 * The path is modified in place. If `path.size() < 2`, `max_straight_distance <= 0.0`, or
 * `interpolation_step <= 0.0`, this function returns immediately and keeps `path` unchanged.
 *
 * @tparam EIGEN_VECTOR_TYPE Vector type used by the path waypoints.
 * @param path Input/output path to be processed in place.
 * @param max_straight_distance Maximum allowed shortcut distance in straight-line optimization.
 * @param interpolation_step Step size used for both interpolation passes.
 * @param optimistic Whether to use optimistic collision checking during shortcutting.
 */
template <typename EIGEN_VECTOR_TYPE>
inline void PathSearcher::optimizePathWithInterpolation(std::vector<EIGEN_VECTOR_TYPE>& path,
                                                        const double max_straight_distance,
                                                        const double interpolation_step,
                                                        const bool optimistic)
{
    if (path.size() < 2 || max_straight_distance <= 0.0 || interpolation_step <= 0.0)
        return;

    std::vector<EIGEN_VECTOR_TYPE> new_path;
    interpolatePath(path, new_path, interpolation_step);
    path.swap(new_path);

    optimizePathToStraight(path, new_path, max_straight_distance, optimistic);
    path.swap(new_path);

    interpolatePath(path, new_path, interpolation_step);
    path.swap(new_path);
}

} // namespace map_process

#endif // _PATH_SEARCHER_H_
