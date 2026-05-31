/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-03 20:17:24
 * @LastEditTime: 2026-05-02 09:48:12
 * @Description:
 */

#ifndef _SINGLE_LEVEL_GRID_
#define _SINGLE_LEVEL_GRID_

#include <memory>
#include <unordered_map>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "grid_info.h"
#include "vdb_utils/vdb_utils.h"

namespace fast_planner
{
class SDFMap;
} // namespace fast_planner

namespace map_process
{
class FrontierManager;
class SingleLevelGrid
{
  private:
    class GridGeometry
    {
      public:
        // --- Factory & lifecycle ---
        GridGeometry();
        GridGeometry(const eigen_utils::Vec3d& grid_resolution, double cell_resolution,
                     const openvdb::Vec3d& cell_offset);

        openvdb::BoolGrid::Ptr createSymbolGrid() const;
        void setRangeConstraint(const eigen_utils::Vec3d& min_set,
                                const eigen_utils::Vec3d& max_set);
        bool shouldCreateGrid(const openvdb::BoolGrid::Ptr& symbol_grid,
                              const openvdb::math::Coord& grid_ijk) const;
        GridInfo createGridInfo(const openvdb::BoolGrid::Ptr& symbol_grid,
                                const openvdb::math::Coord& grid_ijk, bool is_active) const;

        // --- Geometry initialization (one-shot, per grid) ---
        /**
         * @brief Compute the 8 corner vertices, bbox, and sub-grid structure for
         *        a newly created grid.  Sets `is_vertices_updated_ = true` —
         *        this is a one-shot initialization, never refreshed after creation.
         */
        void initializeGridGeometry(GridInfo& grid, const eigen_utils::Vec3i& grid_index,
                                    const openvdb::BoolGrid::Ptr& symbol_grid,
                                    const std::shared_ptr<fast_planner::SDFMap>& sdf_map);

        // --- Queries ---
        void getCurrentRange(eigen_utils::Vec3d& min, eigen_utils::Vec3d& max) const;
        void locatePointGrid(const openvdb::BoolGrid::Ptr& symbol_grid,
                             const eigen_utils::Vec3d& point, eigen_utils::Vec3i& index) const;
        eigen_utils::Vec_Vec3i getGridIndicesInRange(const openvdb::BoolGrid::Ptr& symbol_grid,
                                                     const eigen_utils::Vec3d& min,
                                                     const eigen_utils::Vec3d& max) const;
        void getGridBBoxd(const openvdb::BoolGrid::Ptr& symbol_grid,
                          const openvdb::math::Coord& grid_ijk, openvdb::BBoxd& bbox) const;

      private:
        void updateCurrentRange(const eigen_utils::Vec3d& valid_min,
                                const eigen_utils::Vec3d& valid_max);

        // --- Resolution & offset ---
        double cell_resolution_;
        openvdb::Vec3d cell_offset_;
        eigen_utils::Vec3d grid_resolution_;

        // --- Range tracking ---
        eigen_utils::Vec3d min_now_;
        eigen_utils::Vec3d max_now_;
        eigen_utils::Vec3d min_set_;
        eigen_utils::Vec3d max_set_;
        bool is_min_max_set_;
    };

    class GridStorage
    {
      public:
        GridStorage();
        explicit GridStorage(const openvdb::BoolGrid::Ptr& symbol_grid);

        void resetSymbolGrid(const openvdb::BoolGrid::Ptr& symbol_grid);
        openvdb::BoolGrid::Ptr& symbolGrid();
        const openvdb::BoolGrid::Ptr& symbolGrid() const;
        eigen_utils::Vec3iMap<GridInfo>& gridData();
        const eigen_utils::Vec3iMap<GridInfo>& gridData() const;
        /**
         * @brief Reset the per-frame `is_updated_` flag on every grid.
         *
         * Called at the beginning of each grid-data update cycle (Phase 3 of
         * updateGridData).  Paired with updateGridInfo(), which sets the flag
         * back to true once a grid has been processed.
         */
        void resetGridUpdateFlags();
        void clearUpdateIds();
        void recordUpdatedGrid(const eigen_utils::Vec3i& grid_index);
        eigen_utils::Vec_Vec3i getUpdateIds() const;
        bool checkGridExisted(const eigen_utils::Vec3i& index) const;
        GridInfo* findGrid(const eigen_utils::Vec3i& index);
        const GridInfo* findGrid(const eigen_utils::Vec3i& index) const;
        GridInfo& getGridInfo(const eigen_utils::Vec3i& index);
        const GridInfo& getGridInfo(const eigen_utils::Vec3i& index) const;
        bool hasVoxel(const openvdb::math::Coord& grid_ijk) const;
        void activateVoxel(const openvdb::math::Coord& grid_ijk);
        void insertGrid(const eigen_utils::Vec3i& grid_index, GridInfo&& grid);

      private:
        openvdb::BoolGrid::Ptr symbol_grid_;
        eigen_utils::Vec3iMap<GridInfo> grid_data_;
        eigen_utils::Vec_Vec3i update_ids_;
    };

    class GridExplorationUpdater
    {
      public:
        GridExplorationUpdater();
        explicit GridExplorationUpdater(GridStorage* storage);

        void setStorage(GridStorage* storage);
        void setSDFMap(const std::shared_ptr<fast_planner::SDFMap>& sdf_map);
        void setFrontierManager(const std::shared_ptr<FrontierManager>& frontier_manager);

        void updateGridData(std::vector<openvdb::Vec3d>& changed_cells,
                            std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
                            const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio,
                            const eigen_utils::Vec_Vec3i& extra_update_ids,
                            const openvdb::BoolGrid::Ptr& symbol_grid,
                            const eigen_utils::Vec3d& update_min,
                            const eigen_utils::Vec3d& update_max);

        void updateGridExplorationInfo(const eigen_utils::Vec3i& grid_index);
        void extractRelevantGridIds(eigen_utils::Vec3iSet& grid_ids) const;
        void getGridCentroids(const eigen_utils::Vec3i& index,
                              eigen_utils::Vec_Vec3d& centroids) const;
        void getGridExploreState(const eigen_utils::Vec3i& index, GRID_EXPLORE_STATE& state) const;
        void getGridFrontierClusters(
            const eigen_utils::Vec3i& index,
            std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const;
        void getGridUnknownZoneClusterIds(const eigen_utils::Vec3i& index,
                                          std::vector<std::unordered_set<int>>& cluster_ids) const;

      private:
        /**
         * @brief Per-grid per-frame update: refresh known/unknown voxel counts
         *        and re-evaluate the exploration state.
         *
         * Sets `is_updated_ = true` as a guard to prevent double-processing
         * within a single frame.  The flag is reset at the start of every
         * update cycle by GridStorage::resetGridUpdateFlags().
         */
        void updateGridInfo(GridInfo& grid, const eigen_utils::Vec3i& grid_index,
                            const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio);
        void updateGridExploreState(GridInfo& grid, const eigen_utils::Vec3i& grid_index,
                                    const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio);
        /**
         * @brief Compute or update the unknown-zone centroids, cluster assignments,
         *        and sorting for a single grid.
         *
         * Orchestrates a three-stage pipeline: (1) collect frontier seeds from
         * contained clusters, (2) expand each seed through voxel- and subgrid-level
         * BFS, (3) fall back to active-cluster grouping or sort results by size.
         */
        void computeGridUnknownZones(GridInfo& grid);
        bool refineGridCentroid(GridInfo& grid);

        // --- Unknown-zone expansion helpers ---

        /**
         * @brief Collect frontier seeds inside the grid and build a voxel-to-cluster
         *        lookup map.
         * @param grid          The grid whose contained clusters provide the seeds.
         * @param[out] frontiers_in_grid  World positions of frontier voxels in this grid.
         * @param[out] voxel_to_cluster_id  Map from voxel index to frontier-cluster id.
         */
        void collectFrontierSeeds(const GridInfo& grid, eigen_utils::Vec_Vec3d& frontiers_in_grid,
                                  eigen_utils::Vec3iMap<int>& voxel_to_cluster_id) const;

        /**
         * @brief Expand a single frontier seed through connected unknown space using
         *        a combined voxel- and subgrid-level BFS.
         * @param ft_idx                  Index of the frontier seed voxel.
         * @param grid_min, grid_max      World-space bounds of the parent grid.
         * @param subgrid_manager         Sub-grid structure of the parent grid.
         * @param voxel_to_cluster_id     Frontier voxel → cluster id lookup.
         * @param[in,out] voxel_visited   Globally visited voxel set (shared across seeds).
         * @param[in,out] subgrid_visited Globally visited subgrid set (shared across seeds).
         * @param[out] out_voxel_num      Total number of unknown voxels in this zone.
         * @param[out] out_zone_center_sum Running sum of world positions (divide by voxel_num
         *                                 for the centroid).
         * @param[out] out_unknown_zone   Set of world positions of discovered unknown voxels
         *                                 (used for centroid clamping).
         * @param[out] out_unknown_cluster_ids  Frontier-cluster ids connected to this zone.
         * @param[out] out_unknown_subgrid_ids  Completely-unknown subgrid addresses in this zone.
         * @return true if any unknown voxels were discovered, false otherwise.
         */
        bool expandUnknownZoneFromSeed(const eigen_utils::Vec3i& ft_idx,
                                       const eigen_utils::Vec3d& grid_min,
                                       const eigen_utils::Vec3d& grid_max,
                                       const SubGridManager::SharedPtr& subgrid_manager,
                                       const eigen_utils::Vec3iMap<int>& voxel_to_cluster_id,
                                       eigen_utils::Vec3iSet& voxel_visited,
                                       std::unordered_set<int>& subgrid_visited, int& out_voxel_num,
                                       eigen_utils::Vec3d& out_zone_center_sum,
                                       eigen_utils::Vec3dSet<3>& out_unknown_zone,
                                       std::unordered_set<int>& out_unknown_cluster_ids,
                                       std::unordered_set<int>& out_unknown_subgrid_ids) const;

        /**
         * @brief Compute the final centroid of an expanded unknown-zone group and
         *        append it to the grid's unknown-zone storage.
         * @param grid              The parent grid (results are written to its
         *                          unknown_zone_* members).
         * @param zone_center_sum   Running sum of world positions (from BFS).
         * @param voxel_num         Total unknown voxel count in this zone.
         * @param unknown_zone      World positions of discovered unknown voxels
         *                          (for centroid clamping).
         * @param unknown_subgrid_ids Completely-unknown subgrid addresses in this zone.
         * @param subgrid_manager   Sub-grid structure for address queries.
         * @param[in,out] unknown_cluster_ids  Frontier-cluster ids (moved into grid).
         * @param cluster_centroids Map from cluster id to centroid (for building neighbor list).
         * @param[out] cluster_voxel_num  Accumulator of per-zone voxel counts (for sorting).
         */
        void finalizeUnknownZoneCentroid(
            GridInfo& grid, const eigen_utils::Vec3d& zone_center_sum, int voxel_num,
            const eigen_utils::Vec3dSet<3>& unknown_zone,
            const std::unordered_set<int>& unknown_subgrid_ids,
            const SubGridManager::SharedPtr& subgrid_manager,
            std::unordered_set<int>& unknown_cluster_ids,
            const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
            std::vector<int>& cluster_voxel_num);

        /**
         * @brief Fallback path: when no unknown zone is found via BFS, group the
         *        grid's active frontier clusters by bounding-box overlap and use
         *        each group's centroid as a synthetic unknown-zone center.
         * @param grid              The parent grid (results are written to its
         *                          unknown_zone_* members).
         * @param cluster_centroids Frontier clusters contained in this grid.
         */
        void buildFallbackCentroidsFromActiveClusters(
            GridInfo& grid, const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids);

        void inputChangedCells(const std::vector<openvdb::Vec3d>& changed_cells,
                               const openvdb::BoolGrid::Ptr& symbol_grid,
                               std::vector<openvdb::Vec3d>& reserved_cells);
        void
        inputFrontierClusters(const std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids,
                              const openvdb::BoolGrid::Ptr& symbol_grid,
                              std::unordered_map<int, eigen_utils::Vec3d>& reserved_centroids);

        GridStorage* storage_;
        std::shared_ptr<fast_planner::SDFMap> sdf_map_;
        std::shared_ptr<FrontierManager> frontier_manager_;
    };

    class GridMarkerBuilder
    {
      public:
        GridMarkerBuilder() = default;
        explicit GridMarkerBuilder(GridStorage* storage);

        void setStorage(GridStorage* storage);

        void getGridMarkerWithGridIndex(const eigen_utils::Vec3i& index,
                                        eigen_utils::Vec_Vec3d& pts1,
                                        eigen_utils::Vec_Vec3d& pts2) const;
        void getAllGridMarkers(const bool ignore_inactive_grid, eigen_utils::Vec_Vec3d& pts1,
                               eigen_utils::Vec_Vec3d& pts2) const;

      private:
        static void buildMarkerSegments(const eigen_utils::Vec_Vec3d& vertices,
                                        eigen_utils::Vec_Vec3d& pts1, eigen_utils::Vec_Vec3d& pts2);

        GridStorage* storage_;
    };

    GridGeometry geometry_;
    GridStorage storage_;
    GridExplorationUpdater exploration_updater_;
    GridMarkerBuilder marker_builder_;

    std::shared_ptr<fast_planner::SDFMap> sdf_map_;

    bool createAVoxelAndGridData(const openvdb::math::Coord& grid_ijk, const bool is_active);
    void initializeGridGeometryCache();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SharedPtr = std::shared_ptr<SingleLevelGrid>;
    using ConstSharedPtr = std::shared_ptr<const SingleLevelGrid>;
    using UniquePtr = std::unique_ptr<SingleLevelGrid>;
    using ConstUniquePtr = std::unique_ptr<const SingleLevelGrid>;

    SingleLevelGrid();
    SingleLevelGrid(const eigen_utils::Vec3d& grid_resolution, const double cell_resolution,
                    const openvdb::Vec3d& cell_offset);
    SingleLevelGrid(const eigen_utils::Vec3d& grid_resolution, const double cell_resolution,
                    const openvdb::Vec3d& cell_offset, const eigen_utils::Vec3d& min_set,
                    const eigen_utils::Vec3d& max_set);
    SingleLevelGrid(const SingleLevelGrid& other) = delete;
    SingleLevelGrid(SingleLevelGrid&& other) noexcept = default;
    SingleLevelGrid& operator=(const SingleLevelGrid& other) = delete;
    SingleLevelGrid& operator=(SingleLevelGrid&& other) noexcept = default;
    ~SingleLevelGrid() = default;

    void setSDFMap(const std::shared_ptr<fast_planner::SDFMap>& sdf_map);
    void setFrontierManager(const std::shared_ptr<FrontierManager>& frontier_manager);

    void extendGridDataByGridIndices(const std::vector<openvdb::math::Coord>& extend_grid_indices,
                                     const bool is_active);

    void updateGridData(std::vector<openvdb::Vec3d>& changed_cells,
                        std::unordered_map<int, eigen_utils::Vec3d>& reserved_centroids,
                        const eigen_utils::Vec3iMap<float>& label_clouds_covered_ratio,
                        const eigen_utils::Vec_Vec3i& extra_update_ids,
                        const eigen_utils::Vec3d& update_min, const eigen_utils::Vec3d& update_max);

    void updateGridExplorationInfo(const eigen_utils::Vec3i& grid_index);

    void extractRelevantGridIds(eigen_utils::Vec3iSet& grid_ids) const;

    bool checkGridExisted(const eigen_utils::Vec3i& index) const;
    bool checkGridActive(const eigen_utils::Vec3i& index) const;
    void collectStateTransitionGrids(eigen_utils::Vec3iSet& to_explored,
                                     eigen_utils::Vec3iSet& from_explored) const;
    void getGridCentroids(const eigen_utils::Vec3i& index, eigen_utils::Vec_Vec3d& centroids) const;
    void getGridExploreState(const eigen_utils::Vec3i& index, GRID_EXPLORE_STATE& state) const;
    void getGridMinMaxBox(const eigen_utils::Vec3i& index, eigen_utils::Vec3d& min,
                          eigen_utils::Vec3d& max) const;
    void
    getGridFrontierClusters(const eigen_utils::Vec3i& index,
                            std::unordered_map<int, eigen_utils::Vec3d>& cluster_centroids) const;
    void getGridUnknownZoneClusterIds(const eigen_utils::Vec3i& index,
                                      std::vector<std::unordered_set<int>>& cluster_ids) const;
    void getGridBBoxd(const openvdb::math::Coord& grid_ijk, openvdb::BBoxd& bbox) const;
    void getGridAllBBoxd(const bool ignore_inactive_grid,
                         eigen_utils::Vec3iMap<openvdb::BBoxd>& bbox_map) const;
    void getSingleLevelGridRange(eigen_utils::Vec3d& min, eigen_utils::Vec3d& max) const;
    void getPointLocatedGrid(const eigen_utils::Vec3d& point, eigen_utils::Vec3i& index) const;
    eigen_utils::Vec_Vec3i getGridUpdateIds() const;
    eigen_utils::Vec_Vec3i getGridIndicesInRange(const eigen_utils::Vec3d& min,
                                                 const eigen_utils::Vec3d& max) const;

    void getGridMarkerWithGridIndex(const eigen_utils::Vec3i& index, eigen_utils::Vec_Vec3d& pts1,
                                    eigen_utils::Vec_Vec3d& pts2) const;
    void getAllGridMarkers(const bool ignore_inactive_grid, eigen_utils::Vec_Vec3d& pts1,
                           eigen_utils::Vec_Vec3d& pts2) const;
};

} // namespace map_process

#endif // _SINGLE_LEVEL_GRID_
