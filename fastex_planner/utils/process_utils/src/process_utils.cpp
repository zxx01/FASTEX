/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-26 20:08:19
 * @LastEditTime: 2026-02-06 12:40:05
 * @Description:
 */

#include <algorithm>
#include <stdexcept>

#include "process_utils/process_utils.h"

namespace process_utils
{
/**
 * @brief step function
 *
 * @param x the input value
 * @param threshold the threshold
 * @return double
 */
double ProcessUtils::stepFunction(const double x, const double threshold)
{
    return x >= threshold ? 1.0 : 0.0;
}

/**
 * @brief check if the cell is in the box
 *
 * @param pos the position of the cell
 * @param box the box
 * @return true
 * @return false
 */
bool ProcessUtils::isInBox(const eigen_utils::Vec3d& pos, const CubeBox& box)
{
    return isInBox(pos, box.min_, box.max_);
}

/**
 * @brief Check if two CubeBox objects overlap
 *
 * This function checks whether two CubeBox objects overlap. It iterates through each dimension (x,
 * y, z) and determines if there is any overlap between the minimum and maximum coordinates of the
 * two boxes.
 *
 * @param boxA The first CubeBox object
 * @param boxB The second CubeBox object
 * @return true If the CubeBox objects overlap
 * @return false If the CubeBox objects do not overlap
 */
bool ProcessUtils::isOverlapped(const CubeBox& boxA, const CubeBox& boxB)
{
    for (int i = 0; i < 3; ++i)
    {
        if (boxA.min_[i] > boxB.max_[i] || boxA.max_[i] < boxB.min_[i])
            return false;
    }

    return true;
}

/**
 * @brief check if the inner box is within the outer box
 *
 * @param inner_min the minimum position of the inner box
 * @param inner_max the maximum position of the inner box
 * @param outer_min the minimum position of the outer box
 * @param outer_max the maximum position of the outer box
 * @return true
 * @return false
 */
bool ProcessUtils::isInnerBoxWithinOuterBox(const eigen_utils::Vec3d& inner_min,
                                            const eigen_utils::Vec3d& inner_max,
                                            const eigen_utils::Vec3d& outer_min,
                                            const eigen_utils::Vec3d& outer_max)
{
    for (int i = 0; i < 3; ++i)
    {
        if (inner_min[i] < outer_min[i] || inner_max[i] > outer_max[i])
            return false;
    }
    return true;
}

/**
 * @brief remove elements from the list of lists
 *
 * @tparam T the type of the elements
 * @param lists the list of lists
 * @param indices the indices of the elements to be removed
 */
template <typename T>
void ProcessUtils::removeElements(std::list<std::list<T>>& lists, const std::vector<int>& indices)
{
    // First, handle the removal of inner lists at specified positions in the outer list
    auto list_it = lists.begin();
    int list_index = 0;
    int indices_index = 0;

    while (list_it != lists.end())
    {
        // If the current position is in indices, delete the entire inner list
        if (indices_index < indices.size() && list_index == indices[indices_index])
        {
            list_it = lists.erase(list_it);
            ++indices_index; // Move to the next element in indices
        }
        else
        {
            ++list_it; // Otherwise, just move the iterator
        }
        ++list_index; // Update the index of the current outer list
    }

    // Then, handle each remaining inner list, deleting elements at positions in indices
    for (auto& inner_list : lists)
    {
        auto it = inner_list.begin();
        int current_index = 0;
        indices_index = 0;

        while (it != inner_list.end())
        {
            if (indices_index < indices.size() && current_index == indices[indices_index])
            {
                it = inner_list.erase(it); // Delete and update the iterator
                ++indices_index;           // Move to the next element in indices
            }
            else
            {
                ++it; // Otherwise, just move the iterator
            }
            ++current_index; // Update the index of the current inner list
        }
    }
}

/**
 * @brief Sample points uniformly centered within a specified bounding box.
 *
 * This function generates a set of uniformly distributed sample points within a specified bounding
 * box. The points are centered within each sub-interval defined by the number of samples along each
 * axis.
 *
 * @param sample_num The number of sample points along each axis (x, y, z).
 * @param min_bound The minimum bound of the sampling region.
 * @param max_bound The maximum bound of the sampling region.
 * @return A pair containing the center of the sub-interval and the generated sample points.
 */
std::pair<eigen_utils::Vec3f, eigen_utils::Vec_Vec3f>
ProcessUtils::samplePointsUniformCentered(const eigen_utils::Vec3i& sample_num,
                                          const eigen_utils::Vec3f& min_bound,
                                          const eigen_utils::Vec3f& max_bound)
{
    if (!(sample_num.array() >= 1).all())
        throw std::invalid_argument(
            "The number of sample points must be greater than 1 along each axis.");

    // Calculate the interval between sample points along each axis
    // eigen_utils::Vec3i interval_num = sample_num;
    // eigen_utils::Vec3i interval_num = sample_num - eigen_utils::Vec3i::Ones();
    eigen_utils::Vec3i interval_num = sample_num;
    eigen_utils::Vec3f interval = (max_bound - min_bound).cwiseQuotient(interval_num.cast<float>());

    // Reserve space for the sample points
    eigen_utils::Vec_Vec3f samples;
    samples.reserve(sample_num[0] * sample_num[1] * sample_num[2]);

    // Calculate the starting point, centered within the first sub-interval
    // eigen_utils::Vec3f start_pt = min_bound + interval / 2.0f;
    eigen_utils::Vec3f start_pt = min_bound + interval / 2.0f;

    // Generate the sample points
    float x = start_pt[0];
    for (int i = 0; i < sample_num[0]; i++, x += interval[0])
    {
        float y = start_pt[1];
        for (int j = 0; j < sample_num[1]; j++, y += interval[1])
        {
            float z = start_pt[2];
            for (int k = 0; k < sample_num[2]; k++, z += interval[2])
            {
                samples.emplace_back(x, y, z);
            }
        }
    }

    return std::make_pair(interval, samples);
}

/**
 * @brief find four neighbors of the cell in sdf_map
 *
 * @param index the index of the cell
 * @return eigen_utils::Vec_Vec3i
 */
eigen_utils::Vec_Vec3i ProcessUtils::fourNeighbors(const eigen_utils::Vec3i& index)
{
    eigen_utils::Vec_Vec3i neighbors(4);
    eigen_utils::Vec3i tmp;

    tmp = index - eigen_utils::Vec3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = index + eigen_utils::Vec3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = index - eigen_utils::Vec3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = index + eigen_utils::Vec3i(0, 1, 0);
    neighbors[3] = tmp;

    return neighbors;
}

/**
 * @brief find six neighbors of the cell in sdf_map
 *
 * @param index the index of the cell
 * @return eigen_utils::Vec_Vec3i
 */
eigen_utils::Vec_Vec3i ProcessUtils::sixNeighbors(const eigen_utils::Vec3i& index)
{
    eigen_utils::Vec_Vec3i neighbors(6);
    eigen_utils::Vec3i tmp;

    tmp = index - eigen_utils::Vec3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = index + eigen_utils::Vec3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = index - eigen_utils::Vec3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = index + eigen_utils::Vec3i(0, 1, 0);
    neighbors[3] = tmp;
    tmp = index - eigen_utils::Vec3i(0, 0, 1);
    neighbors[4] = tmp;
    tmp = index + eigen_utils::Vec3i(0, 0, 1);
    neighbors[5] = tmp;

    return neighbors;
}

/**
 * @brief  find all neighbors of the cell in sdf_map
 *
 * @param index the index of the cell
 * @return eigen_utils::Vec_Vec3i
 */
eigen_utils::Vec_Vec3i ProcessUtils::allNeighbors(const eigen_utils::Vec3i& index)
{
    eigen_utils::Vec_Vec3i neighbors;
    neighbors.reserve(26);

    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
            {
                if (i == 0 && j == 0 && k == 0)
                    continue;

                neighbors.emplace_back(index + eigen_utils::Vec3i(i, j, k));
            }

    return neighbors;
}

/**
 * @brief find all neighbors of the cell in sdf_map
 *
 * @param index the index of the cell
 * @param neighbors the neighbors of the cell
 */
void ProcessUtils::allNeighbors(const eigen_utils::Vec3i& index, eigen_utils::Vec3iSet& neighbors)
{
    neighbors.clear();
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j)
            for (int k = -1; k <= 1; ++k)
            {
                if (i == 0 && j == 0 && k == 0)
                    continue;
                neighbors.insert(index + eigen_utils::Vec3i(i, j, k));
            }
}

/**
 * @brief find the intersecting box
 *
 * @param min_box1 the minimum position of the box 1
 * @param max_box1 the maximum position of the box 1
 * @param min_box2 the minimum position of the box 2
 * @param max_box2 the maximum position of the box 2
 * @param min_intersect the minimum position of the intersecting box
 * @param max_intersect the maximum position of the intersecting box
 * @return true
 * @return false
 */
bool ProcessUtils::findIntersectingBox(const eigen_utils::Vec3d& min_box1,
                                       const eigen_utils::Vec3d& max_box1,
                                       const eigen_utils::Vec3d& min_box2,
                                       const eigen_utils::Vec3d& max_box2,
                                       eigen_utils::Vec3d& min_intersect,
                                       eigen_utils::Vec3d& max_intersect)
{
    min_intersect = min_box1.cwiseMax(min_box2);
    max_intersect = max_box1.cwiseMin(max_box2);

    if ((max_intersect - min_intersect).minCoeff() >= 0)
    {
        return true;
    }
    else
    {
        min_intersect = eigen_utils::Vec3d::Zero();
        max_intersect = eigen_utils::Vec3d::Zero();
        return false;
    }
}

/**
 * @brief find the union box
 *
 * @param min_box1 the minimum position of the box 1
 * @param max_box1 the maximum position of the box 1
 * @param min_box2 the minimum position of the box 2
 * @param max_box2 the maximum position of the box 2
 * @param min_union the minimum position of the union box
 * @param max_union the maximum position of the union box
 * @return true
 * @return false
 */
bool ProcessUtils::findUnionBox(const eigen_utils::Vec3d& min_box1,
                                const eigen_utils::Vec3d& max_box1,
                                const eigen_utils::Vec3d& min_box2,
                                const eigen_utils::Vec3d& max_box2, eigen_utils::Vec3d& min_union,
                                eigen_utils::Vec3d& max_union)
{
    min_union = min_box1.cwiseMin(min_box2);
    max_union = max_box1.cwiseMax(max_box2);

    if ((max_union - min_union).minCoeff() >= 0)
        return true;
    else
        return false;
}

/**
 * @brief partition the path into segments with the length threshold, and each segment contains at
 * least 2 points.
 *
 * @param path the path to be partitioned
 * @param length_threshold the threshold of the length
 * @param path_segments the partitioned path segments
 */
void ProcessUtils::partitionPath(const eigen_utils::Vec_Vec3d& path, const double length_threshold,
                                 std::vector<eigen_utils::Vec_Vec3d>& path_segments)
{
    path_segments.clear();
    if (path.empty())
        return;

    eigen_utils::Vec_Vec3d segment;
    segment.push_back(path.front());
    double cur_length = 0.0;

    for (size_t i = 1; i < path.size(); ++i)
    {
        const auto last_point = segment.back();
        const auto cur_point = path[i];

        segment.push_back(cur_point);
        cur_length += (cur_point - last_point).norm();

        if (cur_length > length_threshold && segment.size() > 1) // At least 2 points in a segment
        {
            path_segments.push_back(std::move(segment));
            segment.clear();
            segment.push_back(cur_point);
            cur_length = 0.0;
        }
    }

    if (!segment.empty())
        path_segments.push_back(std::move(segment));
}

/**
 * @brief calculate the yaw of the direction
 *
 * @param dir the direction
 * @return double
 */
double ProcessUtils::calculateYaw(const eigen_utils::Vec3d& dir)
{
    return std::atan2(dir.y(), dir.x());
}

/**
 * @brief calculate the yaw of the direction
 *
 * @param p1 the start point
 * @param p2 the end point
 * @return double
 */
double ProcessUtils::calculateYaw(const eigen_utils::Vec3d& p1, const eigen_utils::Vec3d& p2)
{
    return std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
}

/**
 * @brief Groups connected sets and combines their elements.
 *
 * This function takes a vector of unordered sets and groups them into connected components.
 * Two sets are considered connected if they share at least one common element.
 * The function returns a vector of pairs, where each pair contains the indices of the connected
 * sets and the combined elements of these sets.
 *
 * @param sets A vector of unordered sets of integers.
 * @return std::vector<std::pair<std::vector<int>, std::vector<int>>> A vector of pairs, where each
 * pair contains:
 *         - A vector of integers representing the indices of the connected sets.
 *         - A vector of integers representing the combined elements of the connected sets.
 */
std::vector<std::pair<std::vector<int>, std::vector<int>>>
ProcessUtils::groupConnectedSetsAndCombine(const std::vector<std::unordered_set<int>>& sets)
{
    // Check if two unordered sets have common elements (internal function)
    auto hasIntersection = [](const std::unordered_set<int>& set1,
                              const std::unordered_set<int>& set2) {
        for (const auto& elem : set1)
        {
            if (set2.find(elem) != set2.end())
            {
                return true;
            }
        }
        return false;
    };

    // Use DFS to mark connected components (internal function)
    std::function<void(int, std::vector<bool>&, std::vector<int>&, std::unordered_set<int>&)> dfs;
    dfs = [&](int node, std::vector<bool>& visited, std::vector<int>& group,
              std::unordered_set<int>& combinedSet) {
        visited[node] = true;  // Mark the current node as visited
        group.push_back(node); // Add the current node index to the group
        combinedSet.insert(sets[node].begin(),
                           sets[node].end()); // Combine all elements of the current set
        for (int i = 0; i < sets.size(); ++i)
        {
            if (!visited[i] && hasIntersection(sets[node], sets[i]))
            {
                dfs(i, visited, group, combinedSet); // Recursively search connected nodes
            }
        }
    };

    std::vector<std::pair<std::vector<int>, std::vector<int>>>
        results;                                   // Store the grouping results and combined sets
    std::vector<bool> visited(sets.size(), false); // Mark if a set has been visited

    // Traverse each set to find all connected components
    for (int i = 0; i < sets.size(); ++i)
    {
        if (!visited[i])
        {
            std::vector<int> group; // Indices of the current connected component
            std::unordered_set<int>
                combinedSet; // Combined elements of the current connected component
            dfs(i, visited, group, combinedSet);

            // Convert the combined set to a vector
            std::vector<int> combinedVector(combinedSet.begin(), combinedSet.end());
            // std::sort(combinedVector.begin(), combinedVector.end()); // Sort if needed

            // Save the group and the combined set
            results.emplace_back(group, combinedVector);
        }
    }

    return results;
}

std::vector<Eigen::Vector3i> ProcessUtils::generate3x3x3Grid(const Eigen::Vector3i& min,
                                                             const Eigen::Vector3i& max)
{
    std::vector<Eigen::Vector3i> points;
    points.reserve(27); // 3x3x3 = 27 points

    // compute the middle value for each dimension
    int mid_x = (min.x() + max.x()) / 2;
    int mid_y = (min.y() + max.y()) / 2;
    int mid_z = (min.z() + max.z()) / 2;

    // generate grid points
    for (int x : {min.x(), mid_x, max.x()})
    {
        for (int y : {min.y(), mid_y, max.y()})
        {
            for (int z : {min.z(), mid_z, max.z()})
            {
                points.emplace_back(x, y, z);
            }
        }
    }

    return points;
}

} // namespace process_utils