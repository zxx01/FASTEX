/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-26 19:55:55
 * @LastEditTime: 2026-02-06 12:39:29
 * @Description:
 */

#ifndef _PROCESS_UTILS_H_
#define _PROCESS_UTILS_H_

#include <list>
#include <memory>
#include <vector>

#include "common_utils/eigen_utils.h"

namespace process_utils
{
struct CubeBox
{
    eigen_utils::Vec3d min_;
    eigen_utils::Vec3d max_;

    CubeBox() : min_(0.0, 0.0, 0.0), max_(0.0, 0.0, 0.0) {}
    CubeBox(const eigen_utils::Vec3d& min, const eigen_utils::Vec3d& max) : min_(min), max_(max) {}
};

class ProcessUtils
{
  public:
    ProcessUtils(/* args */) {};
    ~ProcessUtils() {};

    // Step function
    static double stepFunction(const double x, const double threshold = 0.0);

    template <typename EIGEN_VECTOR_TYPE>
    static bool isInBox(const EIGEN_VECTOR_TYPE& idx, const EIGEN_VECTOR_TYPE& min,
                        const EIGEN_VECTOR_TYPE& max);
    static bool isInBox(const eigen_utils::Vec3d& pos, const CubeBox& box);

    template <typename EIGEN_VECTOR_TYPE>
    static bool isOverlapped(const EIGEN_VECTOR_TYPE& boxA_min, const EIGEN_VECTOR_TYPE& boxA_max,
                             const EIGEN_VECTOR_TYPE& boxB_min, const EIGEN_VECTOR_TYPE& boxB_max);
    static bool isOverlapped(const CubeBox& boxA, const CubeBox& boxB);
    static bool isInnerBoxWithinOuterBox(const eigen_utils::Vec3d& inner_min,
                                         const eigen_utils::Vec3d& inner_max,
                                         const eigen_utils::Vec3d& outer_min,
                                         const eigen_utils::Vec3d& outer_max);

    template <typename EIGEN_VECTOR_TYPE>
    static double minDistanceToBoundingBoxBoundary(const EIGEN_VECTOR_TYPE& point,
                                                   const EIGEN_VECTOR_TYPE& min_coords,
                                                   const EIGEN_VECTOR_TYPE& max_coords);

    template <typename T>
    static void removeElements(std::list<std::list<T>>& lists, const std::vector<int>& indices);

    // Sample points
    template <typename EIGEN_VECTOR_TYPE>
    static void samplePointsInCuboid(const EIGEN_VECTOR_TYPE& box_min,
                                     const EIGEN_VECTOR_TYPE& box_max,
                                     const EIGEN_VECTOR_TYPE& resolution,
                                     std::vector<EIGEN_VECTOR_TYPE>& points);
    static std::pair<eigen_utils::Vec3f, eigen_utils::Vec_Vec3f>
    samplePointsUniformCentered(const eigen_utils::Vec3i& sample_num,
                                const eigen_utils::Vec3f& min_bound,
                                const eigen_utils::Vec3f& max_bound);

    // Get the neighbors of the index
    static eigen_utils::Vec_Vec3i fourNeighbors(const eigen_utils::Vec3i& index);
    static eigen_utils::Vec_Vec3i sixNeighbors(const eigen_utils::Vec3i& index);
    static eigen_utils::Vec_Vec3i allNeighbors(const eigen_utils::Vec3i& index);
    static void allNeighbors(const eigen_utils::Vec3i& index, eigen_utils::Vec3iSet& neighbors);

    // Get the corners of the box
    template <typename EIGEN_VECTOR_TYPE>
    static void generateBox(const EIGEN_VECTOR_TYPE& center, const EIGEN_VECTOR_TYPE& size,
                            EIGEN_VECTOR_TYPE& min_point, EIGEN_VECTOR_TYPE& max_point);

    template <typename EIGEN_VECTOR_TYPE>
    static std::vector<EIGEN_VECTOR_TYPE> generateBoxCorners(const EIGEN_VECTOR_TYPE& min_point,
                                                             const EIGEN_VECTOR_TYPE& max_point);

    // Find the intersecting box of two boxes
    static bool
    findIntersectingBox(const eigen_utils::Vec3d& min_box1, const eigen_utils::Vec3d& max_box1,
                        const eigen_utils::Vec3d& min_box2, const eigen_utils::Vec3d& max_box2,
                        eigen_utils::Vec3d& min_intersect, eigen_utils::Vec3d& max_intersect);

    // Find the union box of two boxes
    static bool findUnionBox(const eigen_utils::Vec3d& min_box1, const eigen_utils::Vec3d& max_box1,
                             const eigen_utils::Vec3d& min_box2, const eigen_utils::Vec3d& max_box2,
                             eigen_utils::Vec3d& min_union, eigen_utils::Vec3d& max_union);

    static void partitionPath(const eigen_utils::Vec_Vec3d& path, const double length_threshold,
                              std::vector<eigen_utils::Vec_Vec3d>& path_segments);

    // Calculate the yaw angle
    static double calculateYaw(const eigen_utils::Vec3d& dir);
    static double calculateYaw(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2);

    static std::vector<std::pair<std::vector<int>, std::vector<int>>>
    groupConnectedSetsAndCombine(const std::vector<std::unordered_set<int>>& sets);

    static std::vector<Eigen::Vector3i> generate3x3x3Grid(const Eigen::Vector3i& min,
                                                          const Eigen::Vector3i& max);
};

/**
 * @brief Check if a point is within a bounding box.
 *
 * This function checks if a given point is within the specified minimum and maximum bounds.
 * It works for vectors of any dimension.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the Eigen vector.
 * @param idx The point to check.
 * @param min The minimum bounds of the box.
 * @param max The maximum bounds of the box.
 * @return true If the point is within the bounds.
 * @return false If the point is outside the bounds.
 */
template <typename EIGEN_VECTOR_TYPE>
bool ProcessUtils::isInBox(const EIGEN_VECTOR_TYPE& idx, const EIGEN_VECTOR_TYPE& min,
                           const EIGEN_VECTOR_TYPE& max)
{
    // Ensure that the dimensions of the vectors match
    assert(idx.size() == min.size() && min.size() == max.size());

    // Check if the point is within the bounds for each dimension
    for (int i = 0; i < idx.size(); ++i)
    {
        if (idx[i] < min[i] || idx[i] > max[i])
            return false;
    }

    return true;
}

/**
 * @brief Check if two bounding boxes overlap
 *
 * This function checks whether two 3D bounding boxes overlap. It takes the minimum and
 * maximum coordinates of both boxes and determines if there is any overlap along each dimension.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the Eigen vector used for the coordinates
 * @param boxA_min The minimum coordinates of the first bounding box
 * @param boxA_max The maximum coordinates of the first bounding box
 * @param boxB_min The minimum coordinates of the second bounding box
 * @param boxB_max The maximum coordinates of the second bounding box
 * @return true If the bounding boxes overlap
 * @return false If the bounding boxes do not overlap
 */
template <typename EIGEN_VECTOR_TYPE>
inline bool
ProcessUtils::isOverlapped(const EIGEN_VECTOR_TYPE& boxA_min, const EIGEN_VECTOR_TYPE& boxA_max,
                           const EIGEN_VECTOR_TYPE& boxB_min, const EIGEN_VECTOR_TYPE& boxB_max)
{
    for (int i = 0; i < 3; ++i)
    {
        if (boxA_max[i] < boxB_min[i] || boxB_max[i] < boxA_min[i])
            return false;
    }
    return true;
}

/**
 * @brief Calculate the minimum distance from a point to the boundary of an axis-aligned bounding
 * box (AABB).
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the vector, which must provide `size()` and `operator[]`
 * methods.
 * @param point The point for which the distance is to be calculated.
 * @param min_coords The minimum coordinates of the bounding box.
 * @param max_coords The maximum coordinates of the bounding box.
 * @return double The minimum distance from the point to the boundary of the bounding box.
 */
template <typename EIGEN_VECTOR_TYPE>
inline double ProcessUtils::minDistanceToBoundingBoxBoundary(const EIGEN_VECTOR_TYPE& point,
                                                             const EIGEN_VECTOR_TYPE& min_coords,
                                                             const EIGEN_VECTOR_TYPE& max_coords)
{
    // Initialize the minimum distance to a large value
    double min_distance = std::numeric_limits<double>::max();

    // Loop over each dimension
    for (int i = 0; i < point.size(); ++i)
    {
        double d = 0.0;
        if (point[i] < min_coords[i])
            d = min_coords[i] - point[i]; // Outside on the lower side
        else if (point[i] > max_coords[i])
            d = point[i] - max_coords[i]; // Outside on the upper side
        else
            d = std::min(point[i] - min_coords[i],
                         max_coords[i] - point[i]); // Inside, distance to the closest boundary

        // Update the minimum distance
        min_distance = std::min(min_distance, d);
    }

    return min_distance;
}

/**
 * @brief Sample points uniformly within a cuboid.
 *
 * This function generates a set of uniformly distributed sample points within a specified cuboid.
 * The cuboid is defined by its minimum and maximum points, and the resolution along each axis.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the Eigen vector (e.g., Eigen::Vector3f).
 * @param box_min The minimum point of the cuboid.
 * @param box_max The maximum point of the cuboid.
 * @param resolution The resolution of the sampling along each axis.
 * @param points A vector to store the generated sample points.
 */
template <typename EIGEN_VECTOR_TYPE>
inline void ProcessUtils::samplePointsInCuboid(const EIGEN_VECTOR_TYPE& box_min,
                                               const EIGEN_VECTOR_TYPE& box_max,
                                               const EIGEN_VECTOR_TYPE& resolution,
                                               std::vector<EIGEN_VECTOR_TYPE>& points)
{
    for (float x = box_min.x(); x <= box_max.x(); x += resolution.x())
        for (float y = box_min.y(); y <= box_max.y(); y += resolution.y())
            for (float z = box_min.z(); z <= box_max.z(); z += resolution.z())
                points.emplace_back(x, y, z);
}

/**
 * @brief Generate the corners of a box defined by its minimum and maximum points.
 *
 * This function generates the eight corners of a box (cuboid) defined by its minimum and maximum
 * points. The corners are returned as a vector of Eigen vectors.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the Eigen vector (e.g., Eigen::Vector3f).
 * @param min_point The minimum point of the box.
 * @param max_point The maximum point of the box.
 * @return std::vector<EIGEN_VECTOR_TYPE> A vector containing the eight corners of the box.
 */
template <typename EIGEN_VECTOR_TYPE>
std::vector<EIGEN_VECTOR_TYPE> ProcessUtils::generateBoxCorners(const EIGEN_VECTOR_TYPE& min_point,
                                                                const EIGEN_VECTOR_TYPE& max_point)
{
    std::vector<EIGEN_VECTOR_TYPE> corners(8);
    corners[0] = min_point;
    corners[1] = EIGEN_VECTOR_TYPE(min_point.x(), min_point.y(), max_point.z());
    corners[2] = EIGEN_VECTOR_TYPE(min_point.x(), max_point.y(), min_point.z());
    corners[3] = EIGEN_VECTOR_TYPE(min_point.x(), max_point.y(), max_point.z());
    corners[4] = EIGEN_VECTOR_TYPE(max_point.x(), min_point.y(), min_point.z());
    corners[5] = EIGEN_VECTOR_TYPE(max_point.x(), min_point.y(), max_point.z());
    corners[6] = EIGEN_VECTOR_TYPE(max_point.x(), max_point.y(), min_point.z());
    corners[7] = max_point;

    return corners;
}

/**
 * @brief Generate the minimum and maximum points of a box
 *
 * This function calculates the minimum and maximum points of a box given its center
 * and size. The box is assumed to be axis-aligned.
 *
 * @tparam EIGEN_VECTOR_TYPE The type of the Eigen vector (e.g., Eigen::Vector3f, Eigen::Vector3d)
 * @param center The center point of the box
 * @param size The size of the box (width, height, depth)
 * @param min_point The calculated minimum point of the box (output parameter)
 * @param max_point The calculated maximum point of the box (output parameter)
 */
template <typename EIGEN_VECTOR_TYPE>
inline void ProcessUtils::generateBox(const EIGEN_VECTOR_TYPE& center,
                                      const EIGEN_VECTOR_TYPE& size, EIGEN_VECTOR_TYPE& min_point,
                                      EIGEN_VECTOR_TYPE& max_point)
{
    min_point = center - size / 2.0;
    max_point = center + size / 2.0;
}

} // namespace process_utils

#endif // !_PROCESS_UTILS_H_