/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-12-14 18:02:22
 * @LastEditTime: 2025-10-28 11:16:54
 * @Description:
 */

#ifndef HASH_GRID_HPP
#define HASH_GRID_HPP

#include <memory>
#include <optional>

#include "common_utils/eigen_utils.h"
#include "process_utils/process_utils.h"

namespace utils
{
template <typename T> class HashGridManager
{
  public:
    using SharedPtr = std::shared_ptr<HashGridManager>;
    using UniquePtr = std::unique_ptr<HashGridManager>;

    HashGridManager(const eigen_utils::Vec3d& origin, double grid_size)
        : origin_(origin), grid_size_(grid_size)
    {
    }
    ~HashGridManager() {}

    /**
     * @brief Clear all grid data
     *
     * This function clears all the data stored in the grid.
     */
    void clear() { grids_.clear(); }

    /**
     * @brief Reset the grid manager
     *
     * This function resets the grid manager with a new origin and grid size, and clears all
     * existing grid data.
     *
     * @param origin The new origin of the grid
     * @param grid_size The new size of each grid cell
     */
    void reset(const eigen_utils::Vec3d& origin, double grid_size)
    {
        origin_ = origin;
        grid_size_ = grid_size;
        grids_.clear();
    }

    /**
     * @brief Get the grid index for a given point
     *
     * This function calculates the grid index for a given point in space.
     *
     * @param point The point in space
     * @return eigen_utils::Vec3i The grid index corresponding to the point
     */
    eigen_utils::Vec3i getGridIndex(const eigen_utils::Vec3d& point) const
    {
        return ((point - origin_).array() / grid_size_).floor().cast<int>();
    }

    /**
     * @brief Get the center of a grid cell
     *
     * This function calculates the center point of a grid cell given its index.
     *
     * @param grid_index The index of the grid cell
     * @return eigen_utils::Vec3d The center point of the grid cell
     */
    eigen_utils::Vec3d getGridCenter(const eigen_utils::Vec3i& grid_index) const
    {
        return origin_ +
               (grid_index.cast<double>() + eigen_utils::Vec3d(0.5, 0.5, 0.5)) * grid_size_;
    }

    /**
     * @brief Get the bounds of a grid cell
     *
     * This function calculates the minimum and maximum corners of a grid cell given its index.
     *
     * @param grid_index The index of the grid cell
     * @return std::pair<eigen_utils::Vec3d, eigen_utils::Vec3d> A pair containing the minimum and
     * maximum corners of the grid cell
     */
    std::pair<eigen_utils::Vec3d, eigen_utils::Vec3d>
    getGridBounds(const eigen_utils::Vec3i& grid_index) const
    {
        eigen_utils::Vec3d min_corner = origin_ + grid_index.cast<double>() * grid_size_;
        eigen_utils::Vec3d max_corner =
            min_corner + eigen_utils::Vec3d(grid_size_, grid_size_, grid_size_);
        return {min_corner, max_corner};
    }

    /**
     * @brief Get or create grid data
     *
     * This function returns the data associated with a grid cell. If the grid cell does not exist,
     * it creates a new one with default data.
     *
     * @param grid_index The index of the grid cell
     * @return T& Reference to the data of the grid cell
     */
    T& getOrCreateGridData(const eigen_utils::Vec3i& grid_index)
    {
        if (grids_.find(grid_index) == grids_.end())
        {
            grids_[grid_index] = T(); // Default initialize grid data
        }
        return grids_[grid_index];
    }

    /**
     * @brief Get the data of a grid cell if it exists
     *
     * This function returns the data associated with a grid cell if it exists. If the grid cell
     * does not exist, it returns an empty optional.
     *
     * @param grid_index The index of the grid cell
     * @return std::optional<T> The data of the grid cell if it exists, otherwise an empty optional
     */
    std::optional<T> getGridData(const eigen_utils::Vec3i& grid_index) const
    {
        auto it = grids_.find(grid_index);
        if (it != grids_.end())
        {
            return it->second;
        }
        return std::nullopt;
    }

    /**
     * @brief Check if a grid cell exists
     *
     * This function checks whether a grid cell exists for a given index.
     *
     * @param grid_index The index of the grid cell
     * @return true If the grid cell exists
     * @return false If the grid cell does not exist
     */
    bool hasGrid(const eigen_utils::Vec3i& grid_index) const
    {
        return grids_.find(grid_index) != grids_.end();
    }

    /**
     * @brief Perform a box search within the grid
     *
     * This function performs a search within the grid for all cells that intersect with a given
     * bounding box.
     *
     * @param min_corner The minimum corner of the bounding box
     * @param max_corner The maximum corner of the bounding box
     * @return std::vector<std::pair<eigen_utils::Vec3i, T>> A vector of pairs containing the grid
     * index and the data of the intersecting cells
     */
    std::vector<std::pair<eigen_utils::Vec3i, T>>
    boxSearch(const eigen_utils::Vec3d& min_corner, const eigen_utils::Vec3d& max_corner) const
    {
        std::vector<std::pair<eigen_utils::Vec3i, T>> results;

        // Calculate the grid range of the bounding box
        eigen_utils::Vec3i min_index = getGridIndex(min_corner);
        eigen_utils::Vec3i max_index = getGridIndex(max_corner);

        // Iterate through all indices within the grid range
        for (int x = min_index.x(); x <= max_index.x(); ++x)
        {
            for (int y = min_index.y(); y <= max_index.y(); ++y)
            {
                for (int z = min_index.z(); z <= max_index.z(); ++z)
                {
                    eigen_utils::Vec3i current_index(x, y, z);

                    // Check if the grid cell exists
                    auto it = grids_.find(current_index);
                    if (it != grids_.end())
                    {
                        // Get the bounds of the grid cell
                        auto [grid_min, grid_max] = getGridBounds(current_index);

                        // Check if the grid cell intersects with the bounding box
                        if (process_utils::ProcessUtils::isOverlapped(grid_min, grid_max,
                                                                      min_corner, max_corner))
                        {
                            results.emplace_back(current_index, it->second);
                        }
                    }
                }
            }
        }

        return results;
    }

  private:
    eigen_utils::Vec3d origin_;      // The origin of the grid
    double grid_size_;               // The size of each grid cell
    eigen_utils::Vec3iMap<T> grids_; // The map of grid cells
};

} // namespace utils

#endif // !HASH_GRID_HPP
