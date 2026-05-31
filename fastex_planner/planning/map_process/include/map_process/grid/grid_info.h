/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-02 21:27:07
 * @Description:
 */

#ifndef _GRID_INFO_
#define _GRID_INFO_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "plan_env/sdf_map.h"
#include "vdb_utils/vdb_utils.h"

namespace map_process
{

// --- Lightweight Eigen <-> OpenVDB coordinate conversions ---
// Replace verbose VDBUtil::convertEigenVecToOpenVDBVec<E, V>(...) /
// VDBUtil::convertOpenVDBVecToEigenVec<E, V>(...) template calls.

inline openvdb::Vec3d toVdb(const eigen_utils::Vec3d& v)
{
    return vdb_utils::VDBUtil::convertEigenVecToOpenVDBVec<eigen_utils::Vec3d, openvdb::Vec3d>(v);
}

inline openvdb::math::Coord toVdbCoord(const eigen_utils::Vec3i& v)
{
    return vdb_utils::VDBUtil::convertEigenVecToOpenVDBVec<eigen_utils::Vec3i,
                                                           openvdb::math::Coord>(v);
}

inline eigen_utils::Vec3i fromVdbCoord(const openvdb::math::Coord& v)
{
    return vdb_utils::VDBUtil::convertOpenVDBVecToEigenVec<openvdb::math::Coord,
                                                           eigen_utils::Vec3i>(v);
}

inline eigen_utils::Vec3d fromVdb(const openvdb::Vec3d& v)
{
    return vdb_utils::VDBUtil::convertOpenVDBVecToEigenVec<openvdb::Vec3d, eigen_utils::Vec3d>(v);
}

enum class GRID_EXPLORE_STATE : uint8_t
{
    UNDEFINED,
    UNEXPLORED,
    EXPLORING,
    EXPLORED,
};

struct SubGridInfo
{
    int sub_state_ = 0;
    int voxel_num_ = 0;
    eigen_utils::Vec3i sub_index_min_ = eigen_utils::Vec3i::Zero();
    eigen_utils::Vec3i sub_index_max_ = eigen_utils::Vec3i::Zero();

    SubGridInfo() = default;
    SubGridInfo(const int state, const eigen_utils::Vec3i& index_min,
                const eigen_utils::Vec3i& index_max)
        : sub_state_(state), sub_index_min_(index_min), sub_index_max_(index_max)
    {
        voxel_num_ = (index_max.x() - index_min.x() + 1) * (index_max.y() - index_min.y() + 1) *
                     (index_max.z() - index_min.z() + 1);
    }
    ~SubGridInfo() = default;
};

class SubGridManager
{
  public:
    using UniquePtr = std::unique_ptr<SubGridManager>;
    using SharedPtr = std::shared_ptr<SubGridManager>;

    eigen_utils::Vec3d min_d_, max_d_;
    int sub_grid_num_x_, sub_grid_num_y_, sub_grid_num_z_;
    int sub_grid_num_xy_, sub_grid_num_xyz_;
    double sub_grid_size_x_, sub_grid_size_y_, sub_grid_size_z_;
    double sub_grid_size_x_inv_, sub_grid_size_y_inv_, sub_grid_size_z_inv_;
    std::vector<SubGridInfo> sub_grids_;
    fast_planner::SDFMap::Ptr sdf_;

    // Constructor and Destructor
    SubGridManager() = default;

    SubGridManager(fast_planner::SDFMap::Ptr sdf_map, const eigen_utils::Vec3d& min_d,
                   const eigen_utils::Vec3d& max_d, const int sub_grid_num_x,
                   const int sub_grid_num_y, const int sub_grid_num_z)
    {
        sdf_ = sdf_map;
        min_d_ = min_d;
        max_d_ = max_d;
        sub_grid_num_x_ = sub_grid_num_x;
        sub_grid_num_y_ = sub_grid_num_y;
        sub_grid_num_z_ = sub_grid_num_z;
        sub_grid_num_xy_ = sub_grid_num_x_ * sub_grid_num_y_;
        sub_grid_num_xyz_ = sub_grid_num_x_ * sub_grid_num_y_ * sub_grid_num_z_;

        sub_grid_size_x_ = (max_d.x() - min_d.x()) / sub_grid_num_x_;
        sub_grid_size_y_ = (max_d.y() - min_d.y()) / sub_grid_num_y_;
        sub_grid_size_z_ = (max_d.z() - min_d.z()) / sub_grid_num_z_;
        sub_grid_size_x_inv_ = 1.0 / sub_grid_size_x_;
        sub_grid_size_y_inv_ = 1.0 / sub_grid_size_y_;
        sub_grid_size_z_inv_ = 1.0 / sub_grid_size_z_;

        sub_grids_.reserve(sub_grid_num_xyz_);
        for (int z = 0; z < sub_grid_num_z_; ++z)
        {
            double z_min = min_d.z() + z * sub_grid_size_z_;
            double z_max = min_d.z() + (z + 1) * sub_grid_size_z_;

            for (int y = 0; y < sub_grid_num_y_; ++y)
            {
                double y_min = min_d.y() + y * sub_grid_size_y_;
                double y_max = min_d.y() + (y + 1) * sub_grid_size_y_;

                for (int x = 0; x < sub_grid_num_x_; ++x)
                {
                    double x_min = min_d.x() + x * sub_grid_size_x_;
                    double x_max = min_d.x() + (x + 1) * sub_grid_size_x_;

                    eigen_utils::Vec3d min_d_temp(x_min, y_min, z_min),
                        max_d_temp(x_max, y_max, z_max);
                    eigen_utils::Vec3d rounded_min_d, rounded_max_d;
                    sdf_->roundPosition(min_d_temp, rounded_min_d);
                    sdf_->roundPosition(max_d_temp, rounded_max_d);

                    eigen_utils::Vec3i index_min, index_max;
                    sdf_->posToIndex(rounded_min_d, index_min);
                    sdf_->posToIndex(rounded_max_d, index_max);

                    for (int i = 0; i < 3; ++i)
                    {
                        if (rounded_min_d[i] < min_d_temp[i])
                            index_min[i] += 1;

                        if (rounded_max_d[i] > max_d_temp[i])
                            index_max[i] -= 1;
                    }

                    sub_grids_.emplace_back(0, index_min, index_max);
                }
            }
        }
    }

    SubGridManager(const SubGridManager& other)
    {
        sdf_ = other.sdf_;
        min_d_ = other.min_d_;
        max_d_ = other.max_d_;
        sub_grid_num_x_ = other.sub_grid_num_x_;
        sub_grid_num_y_ = other.sub_grid_num_y_;
        sub_grid_num_z_ = other.sub_grid_num_z_;
        sub_grid_num_xy_ = other.sub_grid_num_xy_;
        sub_grid_num_xyz_ = other.sub_grid_num_xyz_;
        sub_grid_size_x_ = other.sub_grid_size_x_;
        sub_grid_size_y_ = other.sub_grid_size_y_;
        sub_grid_size_z_ = other.sub_grid_size_z_;
        sub_grid_size_x_inv_ = other.sub_grid_size_x_inv_;
        sub_grid_size_y_inv_ = other.sub_grid_size_y_inv_;
        sub_grid_size_z_inv_ = other.sub_grid_size_z_inv_;
        sub_grids_ = other.sub_grids_;
    }
    ~SubGridManager() = default;

    // Position, Index, Address Conversion
    int posToSubGridAddr(const eigen_utils::Vec3d& pos) const
    {
        eigen_utils::Vec3i index = posToSubGridIndex(pos);
        return indexToSubGridAddr(index);
    }

    eigen_utils::Vec3i posToSubGridIndex(const eigen_utils::Vec3d& pos) const
    {
        int x = static_cast<int>((pos.x() - min_d_.x()) * sub_grid_size_x_inv_);
        int y = static_cast<int>((pos.y() - min_d_.y()) * sub_grid_size_y_inv_);
        int z = static_cast<int>((pos.z() - min_d_.z()) * sub_grid_size_z_inv_);

        x = std::max(0, std::min(x, sub_grid_num_x_ - 1));
        y = std::max(0, std::min(y, sub_grid_num_y_ - 1));
        z = std::max(0, std::min(z, sub_grid_num_z_ - 1));

        return eigen_utils::Vec3i(x, y, z);
    }

    int indexToSubGridAddr(const eigen_utils::Vec3i& index) const
    {
        return index.x() + index.y() * sub_grid_num_x_ +
               index.z() * sub_grid_num_x_ * sub_grid_num_y_;
    }

    eigen_utils::Vec3i subGridAddrToIndex(const int addr) const
    {
        int z = addr / sub_grid_num_xy_;
        int rem = addr % sub_grid_num_xy_;
        int y = rem / sub_grid_num_x_;
        int x = rem % sub_grid_num_x_;

        return eigen_utils::Vec3i(x, y, z);
    }

    // Getter and Setter
    void setSubGridState(const int addr, const int state) { sub_grids_[addr].sub_state_ = state; }
    int getSubGridState(const int addr) const { return sub_grids_[addr].sub_state_; }

    bool isValidAddress(const int addr) const { return addr >= 0 && addr < sub_grid_num_xyz_; }
    bool isValidIndex(const eigen_utils::Vec3i& index) const
    {
        return index.x() >= 0 && index.x() < sub_grid_num_x_ && index.y() >= 0 &&
               index.y() < sub_grid_num_y_ && index.z() >= 0 && index.z() < sub_grid_num_z_;
    }

    /**
     * @brief Finds the boundary voxels between two neighboring grids.
     *
     * This function checks if two grids are adjacent in a specified direction and, if so,
     * returns the boundary voxels between them. The direction is specified as a 3D vector
     * where each component can be -1, 0, or 1, indicating the relative position of the
     * neighboring grid along each axis.
     *
     * @param currentMin The minimum coordinates of the current grid.
     * @param currentMax The maximum coordinates of the current grid.
     * @param neighborMin The minimum coordinates of the neighboring grid.
     * @param neighborMax The maximum coordinates of the neighboring grid.
     * @param direction The direction vector indicating the relative position of the neighboring
     * grid.
     * @return std::optional<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3i>> The boundary voxels
     * between the grids, or std::nullopt if the grids are not adjacent in the specified direction.
     */
    std::optional<std::pair<eigen_utils::Vec3i, eigen_utils::Vec3i>>
    FindBoundaryVoxels(const eigen_utils::Vec3i& currentMin, const eigen_utils::Vec3i& currentMax,
                       const eigen_utils::Vec3i& neighborMin, const eigen_utils::Vec3i& neighborMax,
                       const eigen_utils::Vec3i& direction) const
    {
        // Check if direction is valid
        if (direction == eigen_utils::Vec3i(0, 0, 0))
        {
            return std::nullopt;
        }

        // Determine adjacency conditions for each axis
        // For direction[i]:
        //   1: neighborMin[i] = currentMax[i] + 1
        //  -1: neighborMax[i] = currentMin[i] - 1
        //   0: The grids must overlap on this axis [min_cur..max_cur] and [min_neigh..max_neigh]
        //   must intersect

        // Check if adjacency conditions are met for each axis
        for (int i = 0; i < 3; ++i)
        {
            int d = direction[i];
            if (d == 1)
            {
                // Neighbor is in the positive direction
                if (neighborMin[i] != currentMax[i] + 1)
                {
                    return std::nullopt;
                }
            }
            else if (d == -1)
            {
                // Neighbor is in the negative direction
                if (neighborMax[i] != currentMin[i] - 1)
                {
                    return std::nullopt;
                }
            }
            else
            {
                // d == 0: The grids must overlap on this axis
                int intersectMin = std::max(currentMin[i], neighborMin[i]);
                int intersectMax = std::min(currentMax[i], neighborMax[i]);
                if (intersectMin > intersectMax)
                {
                    // No overlap
                    return std::nullopt;
                }
            }
        }

        // If we reach here, adjacency conditions are met
        // Determine the shape of the boundary range based on the number of non-zero axes in
        // direction
        int nonzeroCount = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (direction[i] != 0)
                nonzeroCount++;
        }

        // Initialize the boundary min and max indices
        eigen_utils::Vec3i bMin, bMax;

        // For each axis, determine the boundary layer based on direction:
        // If direction[i] != 0, it's a tight layer:
        //   direction[i] = 1: The boundary layer on this axis is currentMax[i]
        //   direction[i] = -1: The boundary layer on this axis is currentMin[i]
        // If direction[i] == 0, use the overlapping range on this axis
        for (int i = 0; i < 3; ++i)
        {
            int d = direction[i];
            if (d == 1)
            {
                // Neighbor is in the positive direction, boundary on currentMax[i]
                bMin[i] = currentMax[i];
                bMax[i] = currentMax[i];
            }
            else if (d == -1)
            {
                // Neighbor is in the negative direction, boundary on currentMin[i]
                bMin[i] = currentMin[i];
                bMax[i] = currentMin[i];
            }
            else
            {
                // d == 0, find the overlapping range
                int intersectMin = std::max(currentMin[i], neighborMin[i]);
                int intersectMax = std::min(currentMax[i], neighborMax[i]);
                bMin[i] = intersectMin;
                bMax[i] = intersectMax;
            }
        }

        // For multi-axis adjacency:
        // nonzeroCount indicates:
        // 1. If nonzeroCount = 1, it's face adjacency. The axis is fixed to a single layer, the
        // other two axes provide a 1D or 2D range.
        // 2. If nonzeroCount = 2, it's edge adjacency. Two axes are fixed to single layers, the
        // remaining axis forms a line (or a point if the range is a single value).
        // 3. If nonzeroCount = 3, it's point adjacency. All three axes are fixed to single layers,
        // forming a single voxel point.

        // The logic has already initialized bMin and bMax, no further special handling is needed
        // because:
        // - For non-zero direction axes, they are fixed to single layers (bMin[i] = bMax[i])
        // - For zero direction axes, the overlapping range is used

        // Final check: if nonzeroCount = 3, bMin == bMax indicates a single point (no issue here)
        // If nonzeroCount = 2, two axes are single layers, one axis has a range -> a line segment
        // or a point If nonzeroCount = 1, one axis is a single layer, two axes have ranges -> a
        // face region If nonzeroCount = 1 or 2 with no overlap, it has already returned nullopt
        // above.

        return std::make_pair(bMin, bMax);
    }
};

class GridInfo
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GridInfo() {};
    GridInfo(const eigen_utils::Vec3d& center, const double cell_resolution,
             const openvdb::Vec3d& cell_offset);
    GridInfo(const GridInfo& other);
    GridInfo(GridInfo&& other) noexcept;
    ~GridInfo() {};

    GridInfo& operator=(const GridInfo& other);
    GridInfo& operator=(GridInfo&& other) noexcept;

    std::string getClassInfo() const;

    // --- Group 1: Identity & lifecycle ---
    eigen_utils::Vec3d center_;       // center coordinate of this grid (T)
    eigen_utils::Vec3d valid_center_; // valid center coordinate of this grid (T)
    bool is_updated_;                 // whether this grid has been updated (T)
    bool is_active_;                  // whether this grid is active (T)

    // --- Group  2: Exploration state ---
    GRID_EXPLORE_STATE cur_exp_state_;  // current exploration state of this grid (T)
    GRID_EXPLORE_STATE last_exp_state_; // previous exploration state of this grid (T)

    // --- Group 3: Unknown-zone data ---
    SubGridManager::SharedPtr sub_grid_manager_;    // manager of this grid's sub-grids
    eigen_utils::Vec_Vec3d unknown_zone_centroids_; // centroids of unknown zones in this grid
    std::vector<std::unordered_set<int>>
        unknown_zone_cluster_ids_; // IDs of adjacent clusters for unknown zones in this grid
    std::vector<eigen_utils::Vec_Vec3d>
        unknown_zone_cluster_centroids_; // centroids of adjacent clusters for unknown zones in this
                                         // grid

    // --- Group 4: Voxel occupancy ---
    int all_num_;                           // total number of voxels in this grid (T)
    int unknown_num_;                       // number of unknown voxels in this grid (T)
    int known_num_;                         // number of known voxels in this grid (T)
    openvdb::BoolGrid::Ptr occupancy_grid_; // voxel grid for counting voxels in this grid (T)

    // --- Group 5: Frontier-cluster data ---
    std::unordered_map<int, eigen_utils::Vec3d>
        contained_clusters_ids_; // records which frontiers are contained in this grid (by cluster
                                 // ID) clusters (T)

    // --- Group 6: 3D geometry (vertices & bbox) ---
    bool is_vertices_updated_;       // whether this grid's vertices have been updated (T)
    eigen_utils::Vec3d vmin_, vmax_; // min and max coordinates of this grid (T)
    eigen_utils::Vec3d valid_vmin_, valid_vmax_; // valid min and max coordinates of this grid (T)
    eigen_utils::Vec_Vec3d vertices_;            // the eight vertices of this grid (T)
};

inline GridInfo::GridInfo(const eigen_utils::Vec3d& center, const double cell_resolution,
                          const openvdb::Vec3d& cell_offset)
{
    center_ = center;
    valid_center_ = center;
    unknown_zone_centroids_ = {center_};
    last_exp_state_ = cur_exp_state_ = GRID_EXPLORE_STATE::UNDEFINED;
    all_num_ = unknown_num_ = known_num_ = 0;
    occupancy_grid_ = openvdb::BoolGrid::create(false);
    vdb_utils::VDBUtil::setVoxelSize(*occupancy_grid_, cell_resolution, cell_offset);
    is_updated_ = is_active_ = is_vertices_updated_ = false;
}

inline GridInfo::GridInfo(const GridInfo& other)
    : center_(other.center_), valid_center_(other.valid_center_), is_updated_(other.is_updated_),
      is_active_(other.is_active_), cur_exp_state_(other.cur_exp_state_),
      last_exp_state_(other.last_exp_state_),
      sub_grid_manager_(other.sub_grid_manager_
                            ? std::make_shared<SubGridManager>(*other.sub_grid_manager_)
                            : nullptr),
      unknown_zone_centroids_(other.unknown_zone_centroids_),
      unknown_zone_cluster_ids_(other.unknown_zone_cluster_ids_),
      unknown_zone_cluster_centroids_(other.unknown_zone_cluster_centroids_),
      all_num_(other.all_num_), unknown_num_(other.unknown_num_), known_num_(other.known_num_),
      occupancy_grid_(other.occupancy_grid_ ? other.occupancy_grid_->deepCopy() : nullptr),
      contained_clusters_ids_(other.contained_clusters_ids_),
      is_vertices_updated_(other.is_vertices_updated_), vmin_(other.vmin_), vmax_(other.vmax_),
      valid_vmin_(other.valid_vmin_), valid_vmax_(other.valid_vmax_), vertices_(other.vertices_)
{
}

inline GridInfo::GridInfo(GridInfo&& other) noexcept
    : center_(std::move(other.center_)), valid_center_(std::move(other.valid_center_)),
      is_updated_(other.is_updated_), is_active_(other.is_active_),
      cur_exp_state_(other.cur_exp_state_), last_exp_state_(other.last_exp_state_),
      sub_grid_manager_(std::move(other.sub_grid_manager_)),
      unknown_zone_centroids_(std::move(other.unknown_zone_centroids_)),
      unknown_zone_cluster_ids_(std::move(other.unknown_zone_cluster_ids_)),
      unknown_zone_cluster_centroids_(std::move(other.unknown_zone_cluster_centroids_)),
      all_num_(other.all_num_), unknown_num_(other.unknown_num_), known_num_(other.known_num_),
      occupancy_grid_(std::move(other.occupancy_grid_)),
      contained_clusters_ids_(std::move(other.contained_clusters_ids_)),
      is_vertices_updated_(other.is_vertices_updated_), vmin_(std::move(other.vmin_)),
      vmax_(std::move(other.vmax_)), valid_vmin_(std::move(other.valid_vmin_)),
      valid_vmax_(std::move(other.valid_vmax_)), vertices_(std::move(other.vertices_))
{
    other.occupancy_grid_ = nullptr; // Ensure the moved-from object is in a safe state.
}

inline GridInfo& GridInfo::operator=(const GridInfo& other)
{
    if (this != &other)
    {
        center_ = other.center_;
        valid_center_ = other.valid_center_;
        is_updated_ = other.is_updated_;
        is_active_ = other.is_active_;
        cur_exp_state_ = other.cur_exp_state_;
        last_exp_state_ = other.last_exp_state_;
        sub_grid_manager_ = other.sub_grid_manager_
                                ? std::make_shared<SubGridManager>(*other.sub_grid_manager_)
                                : nullptr;
        unknown_zone_centroids_ = other.unknown_zone_centroids_;
        unknown_zone_cluster_ids_ = other.unknown_zone_cluster_ids_;
        unknown_zone_cluster_centroids_ = other.unknown_zone_cluster_centroids_;
        all_num_ = other.all_num_;
        unknown_num_ = other.unknown_num_;
        known_num_ = other.known_num_;
        occupancy_grid_ = other.occupancy_grid_ ? other.occupancy_grid_->deepCopy() : nullptr;
        contained_clusters_ids_ = other.contained_clusters_ids_;
        is_vertices_updated_ = other.is_vertices_updated_;
        vmin_ = other.vmin_;
        vmax_ = other.vmax_;
        valid_vmin_ = other.valid_vmin_;
        valid_vmax_ = other.valid_vmax_;
        vertices_ = other.vertices_;
    }
    return *this;
}

inline GridInfo& GridInfo::operator=(GridInfo&& other) noexcept
{
    if (this != &other)
    {
        center_ = std::move(other.center_);
        valid_center_ = std::move(other.valid_center_);
        is_updated_ = other.is_updated_;
        is_active_ = other.is_active_;
        cur_exp_state_ = other.cur_exp_state_;
        last_exp_state_ = other.last_exp_state_;
        sub_grid_manager_ = std::move(other.sub_grid_manager_);
        unknown_zone_centroids_ = std::move(other.unknown_zone_centroids_);
        unknown_zone_cluster_ids_ = std::move(other.unknown_zone_cluster_ids_);
        unknown_zone_cluster_centroids_ = std::move(other.unknown_zone_cluster_centroids_);
        all_num_ = other.all_num_;
        unknown_num_ = other.unknown_num_;
        known_num_ = other.known_num_;
        occupancy_grid_ = std::move(other.occupancy_grid_);
        contained_clusters_ids_ = std::move(other.contained_clusters_ids_);
        is_vertices_updated_ = other.is_vertices_updated_;
        vmin_ = std::move(other.vmin_);
        vmax_ = std::move(other.vmax_);
        valid_vmin_ = std::move(other.valid_vmin_);
        valid_vmax_ = std::move(other.valid_vmax_);
        vertices_ = std::move(other.vertices_);
        other.occupancy_grid_ = nullptr; // Ensure the moved-from object is in a safe state.
    }
    return *this;
}

inline std::string GridInfo::getClassInfo() const
{
    return "center: " + eigen_utils::vec_to_string(center_) +
           ", all_num: " + std::to_string(all_num_) +
           ", unknown_num: " + std::to_string(unknown_num_) +
           ", known_num: " + std::to_string(known_num_) +
           ", is_updated: " + std::to_string(is_updated_) +
           ", is_active: " + std::to_string(is_active_) +
           ", cur_exp_state: " + std::to_string(static_cast<int>(cur_exp_state_)) +
           ", last_exp_state: " + std::to_string(static_cast<int>(last_exp_state_)) +
           //  ", is_vertices_updated: " + std::to_string(is_vertices_updated_) +
           //  ", is_markers_saved: " + std::to_string(is_markers_saved_) +
           //  ", vertices: " + eigen_utils::vec_to_string(vertices_) +
           //  ", markers_vector: " + std::to_string(markers_vector_.size()) +
           //  ", occupancy_grid: " + vdb_utils::VDBUtil::getGridInfo(*occupancy_grid_) +
           ", contained_frontier_ids: " + std::to_string(contained_clusters_ids_.size()) +
           ", vmin: " + eigen_utils::vec_to_string(vmin_) +
           ", vmax: " + eigen_utils::vec_to_string(vmax_);
}

} // namespace map_process

#endif // _GRID_INFO_
