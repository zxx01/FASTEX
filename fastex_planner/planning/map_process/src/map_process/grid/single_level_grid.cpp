/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-02 14:44:55
 * @Description:
 */

#include <stdexcept>
#include <string>

#include "map_process/frontier/frontier_manager.h"
#include "map_process/grid/single_level_grid.h"
#include "plan_env/sdf_map.h"
#include "process_utils/process_utils.h"

namespace map_process
{
namespace
{
std::runtime_error makeGridIndexError(const char* func_name, const eigen_utils::Vec3i& index)
{
    return std::runtime_error(std::string(func_name) + ": The grid index " +
                              eigen_utils::vec_to_string(index) + " does not exist!");
}

/**
 * @brief Pure state machine: current exploration state + observations → next state.
 *
 * Transition table (rules evaluated top-to-bottom, first match wins):
 *
 *   From        | Frontier present? | Label-cloud coverage?          | → To
 *   ------------|-------------------|--------------------------------|-----------
 *   UNDEFINED   | yes               | —                              | EXPLORING
 *   UNDEFINED   | no                | no data                        | known%>20% ? EXPLORED : UNEXPLORED
 *   UNDEFINED   | no                | all covered                    | EXPLORED
 *   UNDEFINED   | no                | not all covered                | UNEXPLORED
 *   UNEXPLORED  | yes               | —                              | EXPLORING
 *   UNEXPLORED  | no                | no data                        | known%>20% ? EXPLORED : UNEXPLORED
 *   UNEXPLORED  | no                | all covered                    | EXPLORED
 *   UNEXPLORED  | no                | not all covered                | UNEXPLORED
 *   EXPLORING   | yes               | —                              | EXPLORING
 *   EXPLORING   | no                | no data                        | EXPLORED
 *   EXPLORING   | no                | all covered                    | EXPLORED
 *   EXPLORING   | no                | not all covered                | UNEXPLORED
 *   EXPLORED    | yes               | —                              | EXPLORING
 *   EXPLORED    | no                | not all covered (has label)    | UNEXPLORED
 *   EXPLORED    | *                 | *                              | EXPLORED
 *
 * Key insight: frontier clusters dominate — their presence forces EXPLORING
 * regardless of all other signals.
 */
GRID_EXPLORE_STATE decideGridExploreState(const GridInfo& grid, const bool has_label_cloud,
                                          const bool all_covered)
{
    const bool has_clusters = !grid.contained_clusters_ids_.empty();
    const float known_ratio =
        grid.all_num_ > 0 ? static_cast<float>(grid.known_num_) / static_cast<float>(grid.all_num_)
                          : 0.0f;

    switch (grid.cur_exp_state_)
    {
    case GRID_EXPLORE_STATE::UNDEFINED:
    case GRID_EXPLORE_STATE::UNEXPLORED:
        if (has_clusters)
            return GRID_EXPLORE_STATE::EXPLORING;

        if (!has_label_cloud)
            return known_ratio > 0.2f ? GRID_EXPLORE_STATE::EXPLORED
                                      : GRID_EXPLORE_STATE::UNEXPLORED;

        return all_covered ? GRID_EXPLORE_STATE::EXPLORED : GRID_EXPLORE_STATE::UNEXPLORED;

    case GRID_EXPLORE_STATE::EXPLORING:
        if (has_clusters)
            return GRID_EXPLORE_STATE::EXPLORING;

        if (!has_label_cloud)
            return GRID_EXPLORE_STATE::EXPLORED;

        return all_covered ? GRID_EXPLORE_STATE::EXPLORED : GRID_EXPLORE_STATE::UNEXPLORED;

    case GRID_EXPLORE_STATE::EXPLORED:
        if (has_clusters)
            return GRID_EXPLORE_STATE::EXPLORING;

        if (has_label_cloud && !all_covered)
            return GRID_EXPLORE_STATE::UNEXPLORED;

        return GRID_EXPLORE_STATE::EXPLORED;
    }

    throw std::runtime_error("Grid explore state error!");
}

} // namespace

// ===================== GridGeometry =====================

SingleLevelGrid::GridGeometry::GridGeometry()
    : cell_resolution_(0.0), cell_offset_(0.0, 0.0, 0.0), grid_resolution_(0.0, 0.0, 0.0),
      min_now_(0.0, 0.0, 0.0), max_now_(0.0, 0.0, 0.0), min_set_(0.0, 0.0, 0.0),
      max_set_(0.0, 0.0, 0.0), is_min_max_set_(false)
{
}

SingleLevelGrid::GridGeometry::GridGeometry(const eigen_utils::Vec3d& grid_resolution,
                                            const double cell_resolution,
                                            const openvdb::Vec3d& cell_offset)
    : cell_resolution_(cell_resolution), cell_offset_(cell_offset),
      grid_resolution_(grid_resolution), min_now_(0.0, 0.0, 0.0), max_now_(0.0, 0.0, 0.0),
      min_set_(0.0, 0.0, 0.0), max_set_(0.0, 0.0, 0.0), is_min_max_set_(false)
{
}

openvdb::BoolGrid::Ptr SingleLevelGrid::GridGeometry::createSymbolGrid() const
{
    openvdb::BoolGrid::Ptr symbol_grid = openvdb::BoolGrid::create(false);

    if (grid_resolution_.x() > 0.0 && grid_resolution_.y() > 0.0 && grid_resolution_.z() > 0.0)
    {
        vdb_utils::VDBUtil::setVoxelSize(*symbol_grid, toVdb(grid_resolution_));
    }

    return symbol_grid;
}

void SingleLevelGrid::GridGeometry::setRangeConstraint(const eigen_utils::Vec3d& min_set,
                                                       const eigen_utils::Vec3d& max_set)
{
    min_set_ = min_set;
    max_set_ = max_set;
    is_min_max_set_ = true;
}

bool SingleLevelGrid::GridGeometry::shouldCreateGrid(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                     const openvdb::math::Coord& grid_ijk) const
{
    if (!is_min_max_set_)
        return true;

    openvdb::BBoxd voxel_bboxd;
    vdb_utils::VDBUtil::getVoxelBBoxd(symbol_grid, grid_ijk, voxel_bboxd);
    const eigen_utils::Vec3d box_min = fromVdb(voxel_bboxd.min());
    const eigen_utils::Vec3d box_max = fromVdb(voxel_bboxd.max());
    return process_utils::ProcessUtils::isOverlapped(min_set_, max_set_, box_min, box_max);
}

GridInfo SingleLevelGrid::GridGeometry::createGridInfo(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                       const openvdb::math::Coord& grid_ijk,
                                                       const bool is_active) const
{
    openvdb::Vec3d center_openvdb;
    vdb_utils::VDBUtil::getVoxelCenter(symbol_grid, grid_ijk, center_openvdb);
    const eigen_utils::Vec3d center_eigen(center_openvdb.x(), center_openvdb.y(),
                                          center_openvdb.z());

    GridInfo grid(center_eigen, cell_resolution_, cell_offset_);
    grid.is_active_ = is_active;
    return grid;
}

void SingleLevelGrid::GridGeometry::initializeGridGeometry(
    GridInfo& grid, const eigen_utils::Vec3i& grid_index, const openvdb::BoolGrid::Ptr& symbol_grid,
    const std::shared_ptr<fast_planner::SDFMap>& sdf_map)
{
    const openvdb::math::Transform& grid_tf(symbol_grid->transform());
    const openvdb::math::Coord grid_ijk = toVdbCoord(grid_index);

    const eigen_utils::Vec3d left_bottom_lower = fromVdb(grid_tf.indexToWorld(grid_ijk));
    const eigen_utils::Vec3d right_top_upper =
        fromVdb(grid_tf.indexToWorld(grid_ijk.offsetBy(1, 1, 1)));

    const eigen_utils::Vec3d left_top_lower(left_bottom_lower.x(), right_top_upper.y(),
                                            left_bottom_lower.z());
    const eigen_utils::Vec3d right_top_lower(right_top_upper.x(), right_top_upper.y(),
                                             left_bottom_lower.z());
    const eigen_utils::Vec3d right_bottom_lower(right_top_upper.x(), left_bottom_lower.y(),
                                                left_bottom_lower.z());
    const eigen_utils::Vec3d right_bottom_upper(right_top_upper.x(), left_bottom_lower.y(),
                                                right_top_upper.z());
    const eigen_utils::Vec3d left_bottom_upper(left_bottom_lower.x(), left_bottom_lower.y(),
                                               right_top_upper.z());
    const eigen_utils::Vec3d left_top_upper(left_bottom_lower.x(), right_top_upper.y(),
                                            right_top_upper.z());

    grid.vertices_ = {left_bottom_lower, right_bottom_lower, right_top_lower, left_top_lower,
                      left_bottom_upper, right_bottom_upper, right_top_upper, left_top_upper};
    grid.vmin_ = left_bottom_lower;
    grid.vmax_ = right_top_upper;
    grid.valid_vmin_ = is_min_max_set_ ? grid.vmin_.cwiseMax(min_set_) : grid.vmin_;
    grid.valid_vmax_ = is_min_max_set_ ? grid.vmax_.cwiseMin(max_set_) : grid.vmax_;
    grid.valid_center_ = (grid.valid_vmin_ + grid.valid_vmax_) / 2.0;
    grid.unknown_zone_centroids_ = {grid.valid_center_};
    grid.is_vertices_updated_ = true;

    updateCurrentRange(grid.valid_vmin_, grid.valid_vmax_);

    openvdb::BBoxd bbox(toVdb(grid.valid_vmin_), toVdb(grid.valid_vmax_));
    openvdb::math::CoordBBox cbbox;
    vdb_utils::VDBUtil::convertBBoxdToCoordBox(grid.occupancy_grid_, bbox, false, cbbox);
    grid.all_num_ = cbbox.dim().x() * cbbox.dim().y() * cbbox.dim().z();
    grid.unknown_num_ = grid.all_num_;

    grid.sub_grid_manager_ =
        std::make_shared<SubGridManager>(sdf_map, grid.valid_vmin_, grid.valid_vmax_, 8, 8, 4);
}

void SingleLevelGrid::GridGeometry::getCurrentRange(eigen_utils::Vec3d& min,
                                                    eigen_utils::Vec3d& max) const
{
    min = min_now_;
    max = max_now_;
}

void SingleLevelGrid::GridGeometry::locatePointGrid(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                    const eigen_utils::Vec3d& point,
                                                    eigen_utils::Vec3i& index) const
{
    const openvdb::math::Transform& grid_tf(symbol_grid->transform());
    const openvdb::math::Coord grid_ijk = grid_tf.worldToIndexNodeCentered(toVdb(point));
    index = fromVdbCoord(grid_ijk);
}

eigen_utils::Vec_Vec3i
SingleLevelGrid::GridGeometry::getGridIndicesInRange(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                     const eigen_utils::Vec3d& min,
                                                     const eigen_utils::Vec3d& max) const
{
    openvdb::math::CoordBBox cbbox;
    vdb_utils::VDBUtil::convertBBoxdToCoordBox(symbol_grid, openvdb::BBoxd(toVdb(min), toVdb(max)),
                                               false, cbbox);

    eigen_utils::Vec_Vec3i grid_indices;
    for (auto iter = cbbox.beginXYZ(); iter; ++iter)
    {
        grid_indices.push_back(fromVdbCoord(*iter));
    }
    return grid_indices;
}

void SingleLevelGrid::GridGeometry::getGridBBoxd(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                 const openvdb::math::Coord& grid_ijk,
                                                 openvdb::BBoxd& bbox) const
{
    vdb_utils::VDBUtil::getVoxelBBoxd(symbol_grid, grid_ijk, bbox);
}

void SingleLevelGrid::GridGeometry::updateCurrentRange(const eigen_utils::Vec3d& valid_min,
                                                       const eigen_utils::Vec3d& valid_max)
{
    min_now_ = min_now_.cwiseMin(valid_min);
    max_now_ = max_now_.cwiseMax(valid_max);
}

// ===================== GridStorage =====================

SingleLevelGrid::GridStorage::GridStorage() : symbol_grid_(openvdb::BoolGrid::create(false)) {}

SingleLevelGrid::GridStorage::GridStorage(const openvdb::BoolGrid::Ptr& symbol_grid)
    : symbol_grid_(symbol_grid)
{
}

void SingleLevelGrid::GridStorage::resetSymbolGrid(const openvdb::BoolGrid::Ptr& symbol_grid)
{
    symbol_grid_ = symbol_grid;
}

openvdb::BoolGrid::Ptr& SingleLevelGrid::GridStorage::symbolGrid() { return symbol_grid_; }

const openvdb::BoolGrid::Ptr& SingleLevelGrid::GridStorage::symbolGrid() const
{
    return symbol_grid_;
}

eigen_utils::Vec3iMap<GridInfo>& SingleLevelGrid::GridStorage::gridData() { return grid_data_; }

const eigen_utils::Vec3iMap<GridInfo>& SingleLevelGrid::GridStorage::gridData() const
{
    return grid_data_;
}

void SingleLevelGrid::GridStorage::resetGridUpdateFlags()
{
    for (auto& pair : grid_data_)
        pair.second.is_updated_ = false;
}

void SingleLevelGrid::GridStorage::clearUpdateIds() { update_ids_.clear(); }

void SingleLevelGrid::GridStorage::recordUpdatedGrid(const eigen_utils::Vec3i& grid_index)
{
    update_ids_.push_back(grid_index);
}

eigen_utils::Vec_Vec3i SingleLevelGrid::GridStorage::getUpdateIds() const { return update_ids_; }

bool SingleLevelGrid::GridStorage::checkGridExisted(const eigen_utils::Vec3i& index) const
{
    return grid_data_.find(index) != grid_data_.end();
}

GridInfo* SingleLevelGrid::GridStorage::findGrid(const eigen_utils::Vec3i& index)
{
    auto it = grid_data_.find(index);
    return it == grid_data_.end() ? nullptr : &it->second;
}

const GridInfo* SingleLevelGrid::GridStorage::findGrid(const eigen_utils::Vec3i& index) const
{
    auto it = grid_data_.find(index);
    return it == grid_data_.end() ? nullptr : &it->second;
}

GridInfo& SingleLevelGrid::GridStorage::getGridInfo(const eigen_utils::Vec3i& index)
{
    return grid_data_.at(index);
}

const GridInfo& SingleLevelGrid::GridStorage::getGridInfo(const eigen_utils::Vec3i& index) const
{
    return grid_data_.at(index);
}

bool SingleLevelGrid::GridStorage::hasVoxel(const openvdb::math::Coord& grid_ijk) const
{
    openvdb::BoolGrid::ConstAccessor grid_acc = symbol_grid_->getConstAccessor();
    return grid_acc.isValueOn(grid_ijk);
}

void SingleLevelGrid::GridStorage::activateVoxel(const openvdb::math::Coord& grid_ijk)
{
    openvdb::BoolGrid::Accessor grid_acc = symbol_grid_->getAccessor();
    grid_acc.setValue(grid_ijk, true);
}

void SingleLevelGrid::GridStorage::insertGrid(const eigen_utils::Vec3i& grid_index, GridInfo&& grid)
{
    grid_data_[grid_index] = std::move(grid);
}

// ===================== GridExplorationUpdater =====================

SingleLevelGrid::GridExplorationUpdater::GridExplorationUpdater() : storage_(nullptr) {}

SingleLevelGrid::GridExplorationUpdater::GridExplorationUpdater(GridStorage* storage)
    : storage_(storage)
{
}

void SingleLevelGrid::GridExplorationUpdater::setStorage(GridStorage* storage)
{
    storage_ = storage;
}

void SingleLevelGrid::GridExplorationUpdater::setSDFMap(
    const std::shared_ptr<fast_planner::SDFMap>& sdf_map)
{
    sdf_map_ = sdf_map;
}

void SingleLevelGrid::GridExplorationUpdater::setFrontierManager(
    const std::shared_ptr<FrontierManager>& frontier_manager)
{
    frontier_manager_ = frontier_manager;
}

// ===================== GridMarkerBuilder =====================

SingleLevelGrid::GridMarkerBuilder::GridMarkerBuilder(GridStorage* storage) : storage_(storage) {}

void SingleLevelGrid::GridMarkerBuilder::setStorage(GridStorage* storage) { storage_ = storage; }

void SingleLevelGrid::GridMarkerBuilder::buildMarkerSegments(const eigen_utils::Vec_Vec3d& vertices,
                                                             eigen_utils::Vec_Vec3d& pts1,
                                                             eigen_utils::Vec_Vec3d& pts2)
{
    pts1.clear();
    pts2.clear();
    pts1.reserve(12);
    pts2.reserve(12);

    for (int i = 0; i < 4; ++i)
    {
        pts1.push_back(vertices[i]);
        pts2.push_back(vertices[(i + 1) % 4]);
    }
    for (int i = 0; i < 4; ++i)
    {
        pts1.push_back(vertices[i]);
        pts2.push_back(vertices[i + 4]);
    }
    for (int i = 4; i < 8; ++i)
    {
        pts1.push_back(vertices[i]);
        pts2.push_back(vertices[(i + 1) % 4 + 4]);
    }
}

void SingleLevelGrid::GridMarkerBuilder::getGridMarkerWithGridIndex(
    const eigen_utils::Vec3i& index, eigen_utils::Vec_Vec3d& pts1,
    eigen_utils::Vec_Vec3d& pts2) const
{
    pts1.clear();
    pts2.clear();
    const GridInfo* grid = storage_->findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    if (!grid->is_vertices_updated_)
        throw std::runtime_error(std::string(__func__) + ": The vertices of grid " +
                                 eigen_utils::vec_to_string(index) + " are not updated!");

    buildMarkerSegments(grid->vertices_, pts1, pts2);
}

void SingleLevelGrid::GridMarkerBuilder::getAllGridMarkers(const bool ignore_inactive_grid,
                                                           eigen_utils::Vec_Vec3d& pts1,
                                                           eigen_utils::Vec_Vec3d& pts2) const
{
    for (const auto& pair : storage_->gridData())
    {
        const auto& grid = pair.second;
        if (ignore_inactive_grid && !grid.is_active_)
            continue;

        if (!grid.is_vertices_updated_)
            continue;

        eigen_utils::Vec_Vec3d start_point_vector, end_point_vector;
        buildMarkerSegments(grid.vertices_, start_point_vector, end_point_vector);
        pts1.insert(pts1.end(), start_point_vector.begin(), start_point_vector.end());
        pts2.insert(pts2.end(), end_point_vector.begin(), end_point_vector.end());
    }
}

// ===================== SingleLevelGrid =====================

SingleLevelGrid::SingleLevelGrid()
    : geometry_(), storage_(geometry_.createSymbolGrid()), exploration_updater_(&storage_),
      marker_builder_(&storage_)
{
}

SingleLevelGrid::SingleLevelGrid(const eigen_utils::Vec3d& grid_resolution,
                                 const double cell_resolution, const openvdb::Vec3d& cell_offset)
    : geometry_(grid_resolution, cell_resolution, cell_offset),
      storage_(geometry_.createSymbolGrid()), exploration_updater_(&storage_),
      marker_builder_(&storage_)
{
}

SingleLevelGrid::SingleLevelGrid(const eigen_utils::Vec3d& grid_resolution,
                                 const double cell_resolution, const openvdb::Vec3d& cell_offset,
                                 const eigen_utils::Vec3d& min_set,
                                 const eigen_utils::Vec3d& max_set)
    : SingleLevelGrid(grid_resolution, cell_resolution, cell_offset)
{
    geometry_.setRangeConstraint(min_set, max_set);
}

/**
 * @brief Set the SDF map object
 *
 * @param sdf_map the SDF map object
 */
void SingleLevelGrid::setSDFMap(const std::shared_ptr<fast_planner::SDFMap>& sdf_map)
{
    sdf_map_ = sdf_map;
    exploration_updater_.setSDFMap(sdf_map);
}

void SingleLevelGrid::setFrontierManager(const std::shared_ptr<FrontierManager>& frontier_manager)
{
    exploration_updater_.setFrontierManager(frontier_manager);
}

/**
 * @brief extend the grid data by the grid indices and set the active state
 * @param extend_grid_indices the indices of the grids that need to be extended
 * @param is_active the active state
 *
 */
void SingleLevelGrid::extendGridDataByGridIndices(
    const std::vector<openvdb::math::Coord>& extend_grid_indices, const bool is_active)
{
    for (const auto& grid_ijk : extend_grid_indices)
        createAVoxelAndGridData(grid_ijk, is_active);

    initializeGridGeometryCache();
}

/**
 * @brief Update the grid data by the changed cells and the cluster centroids,
 *        and refresh the active and relevant grid state.
 *
 * @param changed_cells the changed cells that need to be inserted into the grid
 * @param cluster_centroids the centroids of the frontier clusters that need to be inserted into the
 * grid
 * @param label_clouds_covered the label clouds covered flag
 * @param extra_update_ids the ids of the newly extended grids that need to be updated
 * @param update_min  the min point of the update box
 * @param update_max  the max point of the update box
 */
void SingleLevelGrid::updateGridData(std::vector<openvdb::Vec3d>& changed_cells,
                                     std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
                                     const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio,
                                     const eigen_utils::Vec_Vec3i& extra_update_ids,
                                     const eigen_utils::Vec3d& update_min,
                                     const eigen_utils::Vec3d& update_max)
{
    exploration_updater_.updateGridData(changed_cells, cluster_centroids,
                                        label_clouds_covered_ratio, extra_update_ids,
                                        storage_.symbolGrid(), update_min, update_max);
}

void SingleLevelGrid::GridExplorationUpdater::updateGridData(
    std::vector<openvdb::Vec3d>& changed_cells,
    std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
    const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio,
    const eigen_utils::Vec_Vec3i& extra_update_ids, const openvdb::BoolGrid::Ptr& symbol_grid,
    const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max)
{
    // 1.Insert all the changed cells into the grid_data
    std::vector<openvdb::Vec3d> reserved_cells;
    inputChangedCells(changed_cells, symbol_grid, reserved_cells);

    // 1.1 Keep cells that are not contained in the active grids for the next update.
    changed_cells.swap(reserved_cells);

    // 2. Input frontier clusters centroids to the grid
    std::unordered_map<int, eigen_utils::Vec3d> reserved_centroids;
    inputFrontierClusters(cluster_centroids, symbol_grid, reserved_centroids);

    // 2.1 Keep centroids that are not contained in the active grids for the next update.
    cluster_centroids.swap(reserved_centroids);

    // 3. Set all grids as not updated
    storage_->resetGridUpdateFlags();
    storage_->clearUpdateIds();

    // 4. Update the active grid located in the updated box
    openvdb::BBoxd bbox(toVdb(update_min), toVdb(update_max));
    openvdb::math::CoordBBox cbbox;
    vdb_utils::VDBUtil::convertBBoxdToCoordBox(symbol_grid, bbox, false, cbbox);

    for (auto cbbox_iter = cbbox.beginXYZ(); cbbox_iter; ++cbbox_iter)
    {
        eigen_utils::Vec3i temp_ijk_eigen = fromVdbCoord(*cbbox_iter);

        // 4.1 If the grid is not existed or not active, skip it
        GridInfo* grid = storage_->findGrid(temp_ijk_eigen);
        if (grid == nullptr)
            continue;
        if (!grid->is_active_)
            continue;

        // 4.2 Update the grid info
        updateGridInfo(*grid, temp_ijk_eigen, label_clouds_covered_ratio);
    }

    // 5. Update the active grid located in the extra_update_ids,
    //    which are not necessarily contained in the updated box.
    for (const auto& id : extra_update_ids)
    {
        auto& grid = storage_->getGridInfo(id);
        updateGridInfo(grid, id, label_clouds_covered_ratio);
    }
}

/**
 * @brief Update the grid information
 *
 * @param grid the grid info
 */
void SingleLevelGrid::updateGridExplorationInfo(const eigen_utils::Vec3i& grid_index)
{
    exploration_updater_.updateGridExplorationInfo(grid_index);
}

void SingleLevelGrid::GridExplorationUpdater::updateGridExplorationInfo(
    const eigen_utils::Vec3i& grid_index)
{
    GridInfo* grid = storage_->findGrid(grid_index);
    if (grid == nullptr)
    {
        ROS_ERROR_STREAM(__func__ << ": The grid index is not existed in the grid data.");
        return;
    }

    if (!grid->is_active_)
        return;

    computeGridUnknownZones(*grid);
    refineGridCentroid(*grid);
}

void SingleLevelGrid::GridExplorationUpdater::extractRelevantGridIds(
    eigen_utils::Vec3iSet& grid_ids) const
{
    for (const auto& pair : storage_->gridData())
    {
        const auto& grid = pair.second;
        if (!grid.is_active_ || grid.cur_exp_state_ == GRID_EXPLORE_STATE::EXPLORED)
            continue;

        grid_ids.insert(pair.first);
    }
}

void SingleLevelGrid::GridExplorationUpdater::getGridCentroids(
    const eigen_utils::Vec3i& index, eigen_utils::Vec_Vec3d& centroids) const
{
    const GridInfo* grid = storage_->findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    centroids = grid->unknown_zone_centroids_;
}

void SingleLevelGrid::GridExplorationUpdater::getGridExploreState(const eigen_utils::Vec3i& index,
                                                                  GRID_EXPLORE_STATE& state) const
{
    const GridInfo* grid = storage_->findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    state = grid->cur_exp_state_;
}

void SingleLevelGrid::GridExplorationUpdater::getGridFrontierClusters(
    const eigen_utils::Vec3i& index,
    std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const
{
    const GridInfo* grid = storage_->findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    cluster_centroids = grid->contained_clusters_ids_;
}

void SingleLevelGrid::GridExplorationUpdater::getGridUnknownZoneClusterIds(
    const eigen_utils::Vec3i& index, std::vector<std::unordered_set<int>>& cluster_ids) const
{
    const GridInfo* grid = storage_->findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    cluster_ids = grid->unknown_zone_cluster_ids_;
}

/**
 * @brief Update the grid explore state
 *
 * @param grid the grid info
 * @param grid_index the grid index
 * @param label_clouds_covered_ratio the label clouds covered ratio
 */
void SingleLevelGrid::GridExplorationUpdater::updateGridInfo(
    GridInfo& grid, const eigen_utils::Vec3i& grid_index,
    const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio)
{
    // Ensure only one update each time to avoid repeated computation
    if (grid.is_updated_)
        return;

    storage_->recordUpdatedGrid(grid_index);

    grid.is_updated_ = true;
    grid.known_num_ = grid.occupancy_grid_->activeVoxelCount();
    grid.unknown_num_ = std::max(grid.all_num_ - grid.known_num_, 0);
    updateGridExploreState(grid, grid_index, label_clouds_covered_ratio);
}

void SingleLevelGrid::GridExplorationUpdater::updateGridExploreState(
    GridInfo& grid, const eigen_utils::Vec3i& grid_index,
    const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio)
{
    auto label_it = label_clouds_covered_ratio.find(grid_index);
    const bool has_label_cloud = label_it != label_clouds_covered_ratio.end();
    const bool all_covered = has_label_cloud && label_it->second <= 0.1;
    const GRID_EXPLORE_STATE previous_state = grid.cur_exp_state_;
    grid.cur_exp_state_ = decideGridExploreState(grid, has_label_cloud, all_covered);
    grid.last_exp_state_ = previous_state;
}

// ===================== Unknown-zone expansion helpers =====================

void SingleLevelGrid::GridExplorationUpdater::collectFrontierSeeds(
    const GridInfo& grid, eigen_utils::Vec_Vec3d& frontiers_in_grid,
    eigen_utils::Vec3iMap<int>& voxel_to_cluster_id) const
{
    frontiers_in_grid.clear();
    voxel_to_cluster_id.clear();

    eigen_utils::Vec3i local_ft_idx;
    for (const auto& pair : grid.contained_clusters_ids_)
    {
        eigen_utils::Vec_Vec3d local_fts;
        frontier_manager_->getClusterFrontiers(pair.first, local_fts);
        frontiers_in_grid.insert(frontiers_in_grid.end(), local_fts.begin(), local_fts.end());

        for (const auto& ft : local_fts)
        {
            sdf_map_->posToIndex(ft, local_ft_idx);
            voxel_to_cluster_id[local_ft_idx] = pair.first;
        }
    }
}

bool SingleLevelGrid::GridExplorationUpdater::expandUnknownZoneFromSeed(
    const eigen_utils::Vec3i& ft_idx, const eigen_utils::Vec3d& grid_min,
    const eigen_utils::Vec3d& grid_max, const SubGridManager::SharedPtr& subgrid_manager,
    const eigen_utils::Vec3iMap<int>& voxel_to_cluster_id, eigen_utils::Vec3iSet& voxel_visited,
    std::unordered_set<int>& subgrid_visited, int& out_voxel_num,
    eigen_utils::Vec3d& out_zone_center_sum, eigen_utils::Vec3dSet<3>& out_unknown_zone,
    std::unordered_set<int>& out_unknown_cluster_ids,
    std::unordered_set<int>& out_unknown_subgrid_ids) const
{
    std::queue<eigen_utils::Vec3i> voxel_q;
    std::queue<int> subgrid_q;

    voxel_q.push(ft_idx);
    voxel_visited.insert(ft_idx);
    out_unknown_cluster_ids.insert(voxel_to_cluster_id.at(ft_idx));

    eigen_utils::Vec3i cur_idx;
    eigen_utils::Vec3d cur_pos, nbr_pos;
    eigen_utils::Vec_Vec3i nbr_indices;

    auto isFrontier = [&voxel_to_cluster_id](const eigen_utils::Vec3i& index, int& id) -> bool {
        auto it = voxel_to_cluster_id.find(index);
        if (it != voxel_to_cluster_id.end())
        {
            id = it->second;
            return true;
        }
        id = -1;
        return false;
    };

    while (!voxel_q.empty() || !subgrid_q.empty())
    {
        // --- Voxel-level BFS ---
        while (!voxel_q.empty())
        {
            cur_idx = voxel_q.front();
            voxel_q.pop();

            int cluster_id = -1;
            if (sdf_map_->isInflatedUnknown(cur_idx))
            {
                sdf_map_->indexToPos(cur_idx, cur_pos);

                if (out_unknown_zone.find(cur_pos) != out_unknown_zone.end())
                    continue;

                out_unknown_zone.insert(cur_pos);
                out_zone_center_sum += cur_pos;
                ++out_voxel_num;
            }
            else if (!isFrontier(cur_idx, cluster_id))
            {
                continue;
            }

            nbr_indices = process_utils::ProcessUtils::allNeighbors(cur_idx);
            for (const eigen_utils::Vec3i& nbr_idx : nbr_indices)
            {
                if (voxel_visited.find(nbr_idx) != voxel_visited.end() ||
                    !sdf_map_->isInBox(nbr_idx))
                    continue;

                voxel_visited.insert(nbr_idx);
                sdf_map_->indexToPos(nbr_idx, nbr_pos);
                if (!process_utils::ProcessUtils::isInBox(nbr_pos, grid_min, grid_max))
                    continue;

                int subgrid_addr = subgrid_manager->posToSubGridAddr(nbr_pos);
                if (subgrid_manager->getSubGridState(subgrid_addr) == 0)
                {
                    if (subgrid_visited.find(subgrid_addr) != subgrid_visited.end())
                        continue;

                    subgrid_q.push(subgrid_addr);
                    subgrid_visited.insert(subgrid_addr);
                    out_unknown_subgrid_ids.insert(subgrid_addr);
                }
                else
                {
                    cluster_id = -1;
                    if (sdf_map_->isInflatedUnknown(nbr_idx) || isFrontier(nbr_idx, cluster_id))
                    {
                        voxel_q.push(nbr_idx);
                        if (cluster_id >= 0)
                            out_unknown_cluster_ids.insert(cluster_id);
                    }
                }
            }
        }

        // --- Subgrid-level BFS ---
        while (!subgrid_q.empty())
        {
            int cur_subgrid_addr = subgrid_q.front();
            subgrid_q.pop();

            // Batch-add all voxels in this completely-unknown subgrid
            eigen_utils::Vec3d mind, maxd;
            sdf_map_->indexToPos(subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_min_,
                                 mind);
            sdf_map_->indexToPos(subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_max_,
                                 maxd);

            // Insert key corner positions so the centroid clamping has
            // sufficient coverage over this subgrid.
            {
                eigen_utils::Vec_Vec3i key_pis = process_utils::ProcessUtils::generate3x3x3Grid(
                    subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_min_,
                    subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_max_);
                eigen_utils::Vec3d pos;
                for (const auto& key_pi : key_pis)
                {
                    if (!sdf_map_->isInBox(key_pi))
                        continue;
                    sdf_map_->indexToPos(key_pi, pos);
                    out_unknown_zone.insert(pos);
                }
            }

            const int subgrid_voxel_count =
                subgrid_manager->sub_grids_[cur_subgrid_addr].voxel_num_;
            out_zone_center_sum += 0.5 * subgrid_voxel_count * (mind + maxd);
            out_voxel_num += subgrid_voxel_count;

            // Expand to neighboring subgrids
            eigen_utils::Vec3i cur_subgrid_idx =
                subgrid_manager->subGridAddrToIndex(cur_subgrid_addr);
            nbr_indices = process_utils::ProcessUtils::allNeighbors(cur_subgrid_idx);

            for (const eigen_utils::Vec3i& nbr_subgrid_idx : nbr_indices)
            {
                if (!subgrid_manager->isValidIndex(nbr_subgrid_idx))
                    continue;

                int nbr_addr = subgrid_manager->indexToSubGridAddr(nbr_subgrid_idx);

                if (subgrid_manager->getSubGridState(nbr_addr) == 0)
                {
                    if (subgrid_visited.find(nbr_addr) != subgrid_visited.end())
                        continue;

                    subgrid_q.push(nbr_addr);
                    subgrid_visited.insert(nbr_addr);
                    out_unknown_subgrid_ids.insert(nbr_addr);
                }
                else
                {
                    eigen_utils::Vec3i nbr_dire = cur_subgrid_idx - nbr_subgrid_idx;
                    auto boundary = subgrid_manager->FindBoundaryVoxels(
                        subgrid_manager->sub_grids_[nbr_addr].sub_index_min_,
                        subgrid_manager->sub_grids_[nbr_addr].sub_index_max_,
                        subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_min_,
                        subgrid_manager->sub_grids_[cur_subgrid_addr].sub_index_max_, nbr_dire);

                    if (boundary)
                    {
                        auto [b_mini, b_maxi] = *boundary;
                        eigen_utils::Vec3i nbr_idx;
                        int cluster_id = -1;

                        for (int x = b_mini[0]; x <= b_maxi[0]; ++x)
                            for (int y = b_mini[1]; y <= b_maxi[1]; ++y)
                                for (int z = b_mini[2]; z <= b_maxi[2]; ++z)
                                {
                                    nbr_idx << x, y, z;
                                    if (voxel_visited.find(nbr_idx) != voxel_visited.end() ||
                                        !sdf_map_->isInBox(nbr_idx))
                                        continue;

                                    voxel_visited.insert(nbr_idx);

                                    cluster_id = -1;
                                    if (sdf_map_->isInflatedUnknown(nbr_idx) ||
                                        isFrontier(nbr_idx, cluster_id))
                                    {
                                        voxel_q.push(nbr_idx);
                                        if (cluster_id >= 0)
                                            out_unknown_cluster_ids.insert(cluster_id);
                                    }
                                }
                    }
                }
            }
        }
    }

    return out_voxel_num > 0;
}

void SingleLevelGrid::GridExplorationUpdater::finalizeUnknownZoneCentroid(
    GridInfo& grid, const eigen_utils::Vec3d& zone_center_sum, int voxel_num,
    const eigen_utils::Vec3dSet<3>& unknown_zone,
    const std::unordered_set<int>& unknown_subgrid_ids,
    const SubGridManager::SharedPtr& subgrid_manager, std::unordered_set<int>& unknown_cluster_ids,
    const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
    std::vector<int>& cluster_voxel_num)
{
    if (unknown_cluster_ids.empty())
        std::cerr << "unknown_cluster_ids is empty." << std::endl;

    eigen_utils::Vec3d zone_center = zone_center_sum / voxel_num;

    // Clamp the centroid to an actual unknown voxel if the weighted center
    // falls outside the completely-unknown subgrid region.
    if (unknown_subgrid_ids.find(subgrid_manager->posToSubGridAddr(zone_center)) ==
        unknown_subgrid_ids.end())
    {
        zone_center = *(std::min_element(
            unknown_zone.begin(), unknown_zone.end(),
            [&zone_center](const eigen_utils::Vec3d& a, const eigen_utils::Vec3d& b) {
                return (a - zone_center).squaredNorm() < (b - zone_center).squaredNorm();
            }));
    }

    eigen_utils::Vec3d rounded_center;
    sdf_map_->roundPosition(zone_center, rounded_center);
    grid.unknown_zone_centroids_.emplace_back(rounded_center);

    eigen_utils::Vec_Vec3d nbr_cluster_centroids;
    nbr_cluster_centroids.reserve(unknown_cluster_ids.size());
    for (const auto& id : unknown_cluster_ids)
        nbr_cluster_centroids.push_back(cluster_centroids.at(id));

    grid.unknown_zone_cluster_ids_.push_back(std::move(unknown_cluster_ids));
    grid.unknown_zone_cluster_centroids_.push_back(std::move(nbr_cluster_centroids));

    cluster_voxel_num.emplace_back(voxel_num);
}

void SingleLevelGrid::GridExplorationUpdater::buildFallbackCentroidsFromActiveClusters(
    GridInfo& grid, const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids)
{
    std::cerr << "No unknown zone is found, and set the centroids of the active clusters "
                 "in each group as the unknown zone centroids."
              << std::endl;

    std::unordered_set<int> fc_ids;
    for (const auto& [fc_id, fc_centroid] : cluster_centroids)
        fc_ids.insert(fc_id);

    std::vector<process_utils::CubeBox> ac_bboxes;
    eigen_utils::Vec_Vec3d ac_centroids;
    std::vector<int> ac_ids;

    {
        std::vector<map_process::FrontierClusterSummary> ac_summaries;
        frontier_manager_->getActiveFrontierClusterSummaries(ac_summaries);

        ac_bboxes.reserve(ac_summaries.size());
        ac_centroids.reserve(ac_summaries.size());
        ac_ids.reserve(ac_summaries.size());

        for (const auto& fc_info : ac_summaries)
        {
            if (fc_ids.find(fc_info.id_) != fc_ids.end())
            {
                ac_bboxes.emplace_back(fc_info.box_min_, fc_info.box_max_);
                ac_centroids.push_back(fc_info.centroid_);
                ac_ids.push_back(fc_info.id_);
            }
        }
    }

    // Group active clusters by bounding-box overlap (DFS)
    std::vector<std::vector<size_t>> ac_cluster_groups;
    std::vector<bool> visited(ac_bboxes.size(), false);

    for (size_t i = 0; i < ac_bboxes.size(); ++i)
    {
        if (visited[i])
            continue;

        std::vector<size_t> group;
        std::function<void(size_t)> dfs = [&](size_t idx) {
            visited[idx] = true;
            group.push_back(idx);
            for (size_t j = 0; j < ac_bboxes.size(); ++j)
            {
                if (!visited[j] &&
                    process_utils::ProcessUtils::isOverlapped(ac_bboxes[idx], ac_bboxes[j]))
                    dfs(j);
            }
        };
        dfs(i);
        ac_cluster_groups.push_back(std::move(group));
    }

    // Sort groups by size descending
    std::sort(ac_cluster_groups.begin(), ac_cluster_groups.end(),
              [](const std::vector<size_t>& a, const std::vector<size_t>& b) {
                  return a.size() > b.size();
              });

    for (const auto& group : ac_cluster_groups)
    {
        eigen_utils::Vec3d centroid(0.0, 0.0, 0.0);
        std::unordered_set<int> cluster_ids;
        eigen_utils::Vec_Vec3d nbr_cluster_centroids;

        for (const auto& idx : group)
        {
            centroid += ac_centroids[idx];
            cluster_ids.insert(ac_ids[idx]);
            nbr_cluster_centroids.push_back(ac_centroids[idx]);
        }
        centroid /= group.size();

        if (sdf_map_->isInflatedOccupied(centroid))
        {
            eigen_utils::Vec3d valid_centroid;
            if (sdf_map_->bfsNearestFree(eigen_utils::Vec3d(8.0, 8.0, 8.0), centroid,
                                         valid_centroid))
            {
                centroid = valid_centroid;
            }
            else
            {
                ROS_WARN("the nearest free point is not found, set the centroid as the "
                         "closest point to the calculated center!");
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& idx : group)
                {
                    double dist = (ac_centroids[idx] - centroid).squaredNorm();
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        centroid = ac_centroids[idx] + eigen_utils::Vec3d(0.1, 0.1, 0.1);
                    }
                }
            }
        }

        grid.unknown_zone_centroids_.push_back(centroid);
        grid.unknown_zone_cluster_ids_.push_back(cluster_ids);
        grid.unknown_zone_cluster_centroids_.push_back(nbr_cluster_centroids);
    }
}

void SingleLevelGrid::GridExplorationUpdater::computeGridUnknownZones(GridInfo& grid)
{
    if (!grid.is_active_ || (grid.cur_exp_state_ != GRID_EXPLORE_STATE::EXPLORING &&
                             grid.last_exp_state_ != GRID_EXPLORE_STATE::EXPLORING))
        return;

    // Save previous state for fallback
    eigen_utils::Vec_Vec3d last_centroids;
    std::vector<std::unordered_set<int>> last_cluster_ids;
    std::vector<eigen_utils::Vec_Vec3d> last_cluster_centroids;
    grid.unknown_zone_centroids_.swap(last_centroids);
    grid.unknown_zone_cluster_ids_.swap(last_cluster_ids);
    grid.unknown_zone_cluster_centroids_.swap(last_cluster_centroids);

    const eigen_utils::Vec3d grid_min = grid.valid_vmin_;
    const eigen_utils::Vec3d grid_max = grid.valid_vmax_;
    const auto& cluster_centroids = grid.contained_clusters_ids_;
    const auto& subgrid_manager = grid.sub_grid_manager_;

    // 1. Collect frontier seeds
    eigen_utils::Vec_Vec3d frontiers_in_grid;
    eigen_utils::Vec3iMap<int> voxel_to_cluster_id;
    collectFrontierSeeds(grid, frontiers_in_grid, voxel_to_cluster_id);

    // 2. Expand unknown zones from each frontier seed
    eigen_utils::Vec3iSet voxel_visited;
    std::unordered_set<int> subgrid_visited;
    std::vector<int> cluster_voxel_num;

    eigen_utils::Vec3i ft_idx;
    for (const eigen_utils::Vec3d& ft_pos : frontiers_in_grid)
    {
        if (!process_utils::ProcessUtils::isInBox(ft_pos, grid_min, grid_max))
            continue;

        sdf_map_->posToIndex(ft_pos, ft_idx);
        if (voxel_visited.find(ft_idx) != voxel_visited.end())
            continue;

        int voxel_num = 0;
        eigen_utils::Vec3d zone_center_sum(0.0, 0.0, 0.0);
        eigen_utils::Vec3dSet<3> unknown_zone;
        std::unordered_set<int> unknown_cluster_ids;
        std::unordered_set<int> unknown_subgrid_ids;

        if (expandUnknownZoneFromSeed(ft_idx, grid_min, grid_max, subgrid_manager,
                                      voxel_to_cluster_id, voxel_visited, subgrid_visited,
                                      voxel_num, zone_center_sum, unknown_zone, unknown_cluster_ids,
                                      unknown_subgrid_ids))
        {
            finalizeUnknownZoneCentroid(grid, zone_center_sum, voxel_num, unknown_zone,
                                        unknown_subgrid_ids, subgrid_manager, unknown_cluster_ids,
                                        cluster_centroids, cluster_voxel_num);
        }
    }

    // 3. Fallback or sort
    if (grid.unknown_zone_centroids_.empty())
    {
        if (grid.cur_exp_state_ == GRID_EXPLORE_STATE::EXPLORING)
            buildFallbackCentroidsFromActiveClusters(grid, cluster_centroids);
        else
            grid.unknown_zone_centroids_.swap(last_centroids);
    }
    else
    {
        const auto max_iter = std::max_element(cluster_voxel_num.begin(), cluster_voxel_num.end());
        const int max_index = std::distance(cluster_voxel_num.begin(), max_iter);

        std::swap(grid.unknown_zone_centroids_[0], grid.unknown_zone_centroids_[max_index]);
        grid.unknown_zone_cluster_ids_[0].swap(grid.unknown_zone_cluster_ids_[max_index]);
        grid.unknown_zone_cluster_centroids_[0].swap(
            grid.unknown_zone_cluster_centroids_[max_index]);
    }
}

/**
 * @brief Refine the grid centroid
 *
 * This function refines the centroid of the grid by ensuring that it is within the map boundaries
 * and not in an inflated occupancy area. If the centroid is out of the map boundaries, it is moved
 * to the boundary of the map. If the centroid is in an inflated occupancy area, it is moved to the
 * nearest free point.
 *
 * @param grid The grid info
 * @return true if the grid centroid is modified
 * @return false if the grid centroid is not modified
 */
bool SingleLevelGrid::GridExplorationUpdater::refineGridCentroid(GridInfo& grid)
{
    bool is_modified = false;
    for (auto& centroid : grid.unknown_zone_centroids_)
    {
        // 1. If the grid centroid is out of the box, modify it to the boundary of the box
        if (!sdf_map_->isInBox(centroid))
        {
            is_modified = true;
            eigen_utils::Vec3d map_box_min, map_box_max;
            sdf_map_->getBox(map_box_min, map_box_max);

            eigen_utils::Vec3d intersect_min, intersect_max;
            if (process_utils::ProcessUtils::findIntersectingBox(map_box_min, map_box_max,
                                                                 grid.valid_vmin_, grid.valid_vmax_,
                                                                 intersect_min, intersect_max))
                centroid = (intersect_min + intersect_max) / 2.0;
            else
                ROS_WARN("Failed to find the intersecting box!");
        }

        // 2. If the grid centroid is in the inflated occupancy, modify it to the nearest free point
        // in the occupancy map
        if (sdf_map_->getInflateOccupancy(centroid) == 1)
        {
            eigen_utils::Vec3d search_box_size(8.0, 8.0, 8.0);
            is_modified = sdf_map_->bfsNearestFree(search_box_size, centroid, centroid);
            if (!is_modified)
                ROS_WARN("Failed to find the nearest free point for the grid centroid!");
        }

        // 3. Keep the centroid inside its grid box to avoid drifting to a non-existing neighbor
        // grid due to rounding/search post-processing.
        const double eps = 1e-3;
        eigen_utils::Vec3d lower = grid.valid_vmin_ + eigen_utils::Vec3d::Constant(eps);
        eigen_utils::Vec3d upper = grid.valid_vmax_ - eigen_utils::Vec3d::Constant(eps);
        for (int i = 0; i < 3; ++i)
        {
            if (lower[i] > upper[i])
            {
                const double mid = 0.5 * (grid.valid_vmin_[i] + grid.valid_vmax_[i]);
                lower[i] = mid;
                upper[i] = mid;
            }
        }
        const eigen_utils::Vec3d clamped_centroid = centroid.cwiseMax(lower).cwiseMin(upper);
        if (!clamped_centroid.isApprox(centroid))
        {
            centroid = clamped_centroid;
            is_modified = true;
        }
    }

    return is_modified;
}

/**
 * @brief create a voxel and initialize the grid data
 * @param grid_ijk the index of the grid
 * @param is_active the active state
 */
bool SingleLevelGrid::createAVoxelAndGridData(const openvdb::math::Coord& grid_ijk,
                                              const bool is_active)
{
    // If the grid is already created, return
    if (storage_.hasVoxel(grid_ijk))
        return false;

    if (!geometry_.shouldCreateGrid(storage_.symbolGrid(), grid_ijk))
        return false;

    storage_.activateVoxel(grid_ijk);
    storage_.insertGrid(fromVdbCoord(grid_ijk),
                        geometry_.createGridInfo(storage_.symbolGrid(), grid_ijk, is_active));

    return true;
}

/**
 * @brief Inputs the changed cells and updates the grid data.
 *
 * @param changed_cells The changed cells that need to be inserted into the grid.
 * @param reserved_cells  The reserved cells that are not contained in the active grids.
 */
void SingleLevelGrid::GridExplorationUpdater::inputChangedCells(
    const std::vector<openvdb::Vec3d>& changed_cells, const openvdb::BoolGrid::Ptr& symbol_grid,
    std::vector<openvdb::Vec3d>& reserved_cells)
{
    const openvdb::math::Transform& grid_tf(symbol_grid->transform());

    // 1.1 Classify the changed cells by the grid index
    eigen_utils::Vec3iMap<std::vector<openvdb::Vec3d>> cell_pos_map;
    for (const auto& cell : changed_cells)
    {
        eigen_utils::Vec3i temp_ijk_eigen = fromVdbCoord(grid_tf.worldToIndexNodeCentered(cell));
        cell_pos_map[temp_ijk_eigen].push_back(cell);
    }

    // 1.2 Update the occupancy grid by the changed cells and get the reserved cells
    for (const auto& [grid_index, cell_pos_vec] : cell_pos_map)
    {
        GridInfo* grid = storage_->findGrid(grid_index);
        if (grid == nullptr)
        {
            reserved_cells.insert(reserved_cells.end(), cell_pos_vec.begin(), cell_pos_vec.end());
            continue;
        }

        // 1.3 If the grid is not active, insert the cells into the reserved_cells
        if (!grid->is_active_)
        {
            reserved_cells.insert(reserved_cells.end(), cell_pos_vec.begin(), cell_pos_vec.end());
            continue;
        }

        // 1.4 Insert the cells into the active grid
        openvdb::BoolGrid::Accessor cell_grid_acc = grid->occupancy_grid_->getAccessor();
        openvdb::math::Transform& cell_grid_tf(grid->occupancy_grid_->transform());
        for (const auto& cell_pos : cell_pos_vec)
        {
            openvdb::math::Coord cell_ijk = cell_grid_tf.worldToIndexNodeCentered(cell_pos);
            cell_grid_acc.setValue(cell_ijk, true);

            // Set the state of the subgrids
            int subgrid_addr = grid->sub_grid_manager_->posToSubGridAddr(fromVdb(cell_pos));
            grid->sub_grid_manager_->setSubGridState(subgrid_addr, 1);
        }
    }
}

/**
 * @brief Inputs the frontier clusters and updates the grid data.
 *
 * @param cluster_centroids The centroids of the frontier clusters.
 * @param reserved_centroids The centroids of the frontier clusters that are not contained in the
 * active grids.
 */
void SingleLevelGrid::GridExplorationUpdater::inputFrontierClusters(
    const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
    const openvdb::BoolGrid::Ptr& symbol_grid,
    std::unordered_map<int, eigen_utils::Vec3d>& reserved_centroids)
{
    // Clear the contained_clusters_ids_ of each grid
    for (auto& pair : storage_->gridData())
    {
        auto& grid = pair.second;
        grid.contained_clusters_ids_.clear();
    }

    // Input the frontier clusters centroids
    openvdb::math::Transform& grid_tf(symbol_grid->transform());
    for (const auto& pair : cluster_centroids)
    {
        openvdb::math::Coord cluster_centroid_ijk =
            grid_tf.worldToIndexNodeCentered(toVdb(pair.second));
        eigen_utils::Vec3i cluster_centroid_ijk_eigen = fromVdbCoord(cluster_centroid_ijk);
        GridInfo* grid = storage_->findGrid(cluster_centroid_ijk_eigen);
        if (grid != nullptr)
        {
            if (grid->is_active_)
                grid->contained_clusters_ids_.insert(pair);
            else
                reserved_centroids.insert(pair);
        }
        else
        {
            reserved_centroids.insert(pair);
            std::cerr << __func__ + std::string(": The grid does not exist!") << std::endl;
        }
    }
}

/**
 * @brief update the base coordinate of the grid
 *
 */
/**
 * @brief Iterate ALL grids and lazily initialize geometry for newly created ones.
 *
 * Called after extendGridDataByGridIndices which creates GridInfo entries
 * without geometry.  The `is_vertices_updated_` guard ensures we only pay
 * the cost for grids that haven't been initialized yet.
 */
void SingleLevelGrid::initializeGridGeometryCache()
{
    for (auto& pair : storage_.gridData())
    {
        GridInfo& grid = pair.second;

        if (!grid.is_vertices_updated_)
        {
            geometry_.initializeGridGeometry(grid, pair.first, storage_.symbolGrid(), sdf_map_);
        }
    }
}

/**
 * @brief Check if the grid existed
 *
 * @param index the index of the grid
 * @return true if the grid existed
 * @return false  if the grid not existed
 */
bool SingleLevelGrid::checkGridExisted(const eigen_utils::Vec3i& index) const
{
    return storage_.checkGridExisted(index);
}

/**
 * @brief Check if the grid is active
 *
 * @param index the index of the grid
 * @return true if the grid is active
 * @return false if the grid is not active
 */
bool SingleLevelGrid::checkGridActive(const eigen_utils::Vec3i& index) const
{
    const GridInfo* grid = storage_.findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    return grid->is_active_;
}

/**
 * @brief Collect grid indices whose exploration state transitioned in or out of EXPLORED.
 * @param[out] to_explored   Grid indices that changed TO EXPLORED this frame.
 * @param[out] from_explored Grid indices that changed FROM EXPLORED this frame.
 */
void SingleLevelGrid::collectStateTransitionGrids(eigen_utils::Vec3iSet& to_explored,
                                                  eigen_utils::Vec3iSet& from_explored) const
{
    to_explored.clear();
    from_explored.clear();

    for (const auto& [grid_index, grid_info] : storage_.gridData())
    {
        if (!grid_info.is_active_)
            continue;

        if (grid_info.last_exp_state_ != GRID_EXPLORE_STATE::EXPLORED &&
            grid_info.cur_exp_state_ == GRID_EXPLORE_STATE::EXPLORED)
            to_explored.insert(grid_index);
        else if (grid_info.last_exp_state_ == GRID_EXPLORE_STATE::EXPLORED &&
                 grid_info.cur_exp_state_ != GRID_EXPLORE_STATE::EXPLORED)
            from_explored.insert(grid_index);
    }
}

void SingleLevelGrid::getGridCentroids(const eigen_utils::Vec3i& index,
                                       eigen_utils::Vec_Vec3d& centroids) const
{
    exploration_updater_.getGridCentroids(index, centroids);
}

/**
 * @brief get the state of the grid by the index
 *
 * @param index the index of the grid
 * @param state the state of the grid
 */
void SingleLevelGrid::getGridExploreState(const eigen_utils::Vec3i& index,
                                          GRID_EXPLORE_STATE& state) const
{
    exploration_updater_.getGridExploreState(index, state);
}

/**
 * @brief get the min and max of the grid by the index
 *
 * @param index the index of the grid
 * @param min the min point of the grid
 * @param max the max point of the grid
 */
void SingleLevelGrid::getGridMinMaxBox(const eigen_utils::Vec3i& index, eigen_utils::Vec3d& min,
                                       eigen_utils::Vec3d& max) const
{
    const GridInfo* grid = storage_.findGrid(index);
    if (grid == nullptr)
        throw makeGridIndexError(__func__, index);

    min = grid->vmin_;
    max = grid->vmax_;
}

/**
 * @brief get the min and max of the grid
 *
 * @param min the min point of the grid
 * @param max the max point of the grid
 */
void SingleLevelGrid::getSingleLevelGridRange(eigen_utils::Vec3d& min,
                                              eigen_utils::Vec3d& max) const
{
    geometry_.getCurrentRange(min, max);
}

/**
 * @brief get the point located grid by the point
 *
 * @param point the point
 * @param index the index of the grid
 */
void SingleLevelGrid::getPointLocatedGrid(const eigen_utils::Vec3d& point,
                                          eigen_utils::Vec3i& index) const
{
    geometry_.locatePointGrid(storage_.symbolGrid(), point, index);
}

/**
 * @brief Get the IDs of the grids that have been updated.
 *
 * This function returns a vector containing the IDs of the grids that have been updated.
 * The IDs are stored in the `update_ids_` member variable.
 *
 * @return eigen_utils::Vec_Vec3i A vector containing the IDs of the updated grids.
 */
eigen_utils::Vec_Vec3i SingleLevelGrid::getGridUpdateIds() const { return storage_.getUpdateIds(); }

/**
 * @brief Gets the grid indices within a specified range.
 *
 * This function calculates and returns the grid indices that fall within the specified minimum and
 * maximum bounds. It uses the VDB utility to convert the bounding box to coordinate box and
 * iterates through the coordinates to collect the grid indices.
 *
 * @param min The minimum bound of the range.
 * @param max The maximum bound of the range.
 * @return eigen_utils::Vec_Vec3i A vector of grid indices within the specified range.
 */
eigen_utils::Vec_Vec3i SingleLevelGrid::getGridIndicesInRange(const eigen_utils::Vec3d& min,
                                                              const eigen_utils::Vec3d& max) const
{
    return geometry_.getGridIndicesInRange(storage_.symbolGrid(), min, max);
}

/**
 * @brief get the frontier clusters by the index of the grid
 *
 * @param index the index of the grid
 * @param cluster_centroids the centroids of frontier clusters in the grid
 */
void SingleLevelGrid::getGridFrontierClusters(
    const eigen_utils::Vec3i& index,
    std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const
{
    exploration_updater_.getGridFrontierClusters(index, cluster_centroids);
}

void SingleLevelGrid::getGridUnknownZoneClusterIds(
    const eigen_utils::Vec3i& index, std::vector<std::unordered_set<int>>& cluster_ids) const
{
    exploration_updater_.getGridUnknownZoneClusterIds(index, cluster_ids);
}

/**
 * @brief get the bbox of the grid by the index
 *
 * @param grid_ijk the index of the grid
 * @param bbox the bbox of the grid
 */
void SingleLevelGrid::getGridBBoxd(const openvdb::math::Coord& grid_ijk, openvdb::BBoxd& bbox) const
{
    geometry_.getGridBBoxd(storage_.symbolGrid(), grid_ijk, bbox);
}

/**
 * @brief get all bboxes of the grids
 *
 * @param ignore_inactive_grid whether to ignore the inactive grid
 * @param bbox_map the map of the bbox
 */
void SingleLevelGrid::getGridAllBBoxd(const bool ignore_inactive_grid,
                                      eigen_utils::Vec3iMap<openvdb::BBoxd>& bbox_map) const
{
    for (const auto& pair : storage_.gridData())
    {
        const auto grid = pair.second;
        if (ignore_inactive_grid && !grid.is_active_)
            continue;
        const openvdb::v7_0::math::Coord grid_ijk = toVdbCoord(pair.first);
        openvdb::BBoxd bbox;
        geometry_.getGridBBoxd(storage_.symbolGrid(), grid_ijk, bbox);
        bbox_map.insert(std::make_pair(pair.first, bbox));
    }
}

/**
 * @brief get the relevant grid ids (active and not explored)
 *
 * @param grid_ids the ids of the relevant grids
 */
void SingleLevelGrid::extractRelevantGridIds(eigen_utils::Vec3iSet& grid_ids) const
{
    exploration_updater_.extractRelevantGridIds(grid_ids);
}

/**
 * @brief get the vertices of the grid by the index
 *
 * @param index the index of the grid
 * @param vertices the vertices of the grid
 */
void SingleLevelGrid::getGridMarkerWithGridIndex(const eigen_utils::Vec3i& index,
                                                 eigen_utils::Vec_Vec3d& pts1,
                                                 eigen_utils::Vec_Vec3d& pts2) const
{
    marker_builder_.getGridMarkerWithGridIndex(index, pts1, pts2);
}

/**
 * @brief get the markers of the grids
 *
 * @param ignore_inactive_grid whether to ignore the inactive grid
 * @param pts1 the start points of the markers
 * @param pts2 the end points of the markers
 */
void SingleLevelGrid::getAllGridMarkers(const bool ignore_inactive_grid,
                                        eigen_utils::Vec_Vec3d& pts1,
                                        eigen_utils::Vec_Vec3d& pts2) const
{
    marker_builder_.getAllGridMarkers(ignore_inactive_grid, pts1, pts2);
}

} // namespace map_process
