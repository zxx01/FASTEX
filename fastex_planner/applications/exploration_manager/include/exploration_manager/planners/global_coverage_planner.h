/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:33
 * @LastEditTime: 2026-05-31 17:20:51
 * @Description:
 */

#ifndef _GLOBAL_COVERAGE_PLANNER_H_
#define _GLOBAL_COVERAGE_PLANNER_H_

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "common_utils/eigen_utils.h"
#include "exploration_manager/planners/lkh_interface.h"
#include "exploration_manager/planning_types.h"
#include "map_process/grid/dynamic_expanding_grid.h"

namespace fastex_explorer
{
/**
 * @brief Planner responsible for global DynamicExpandingGrid tour generation.
 *
 * This component owns both the full global planning path and the incremental
 * replanning logic that reuses the previous relevant-grid tour.
 */
class GlobalCoveragePlanner
{
  public:
    using UniquePtr = std::unique_ptr<GlobalCoveragePlanner>;

    /**
     * @brief Lightweight visualization payload for incremental global planning.
     *
     * The manager uses this to publish partitioned nodes and segment edges
     * without exposing the planner's internal bookkeeping structures.
     */
    struct IncrementalVisualizationData
    {
        /// Nodes that remain in the local replanning window as standalone vertices.
        eigen_utils::Vec_Vec3d independent_pts;
        /// Nodes that belong to preserved out-of-range segments.
        eigen_utils::Vec_Vec3d segment_pts;
        /// Polyline representation of preserved out-of-range segments.
        std::vector<eigen_utils::Vec_Vec3d> segment_edges;

        /**
         * @brief Reset all cached visualization primitives.
         */
        void clear()
        {
            independent_pts.clear();
            segment_pts.clear();
            segment_edges.clear();
        }
    };

    /**
     * @brief Construct the global coverage planner.
     *
     * @param dynamic_expanding_grid Shared grid model used to build cost matrices and tours.
     * @param lkh_interface Shared LKH wrapper used to solve ATSP instances.
     * @param tsp_dir Directory where temporary LKH problem files are stored.
     * @param drone_id Active robot id used to derive unique LKH file names.
     */
    GlobalCoveragePlanner(
        const map_process::DynamicExpandingGrid::SharedPtr& dynamic_expanding_grid,
        LkhInterface& lkh_interface, const std::string& tsp_dir, const int drone_id);
    ~GlobalCoveragePlanner() = default;

    /**
     * @brief Plan a global grid coverage tour.
     *
     * The planner performs either a full ATSP solve or an incremental update
     * depending on @p refresh_global, and updates the returned grid tour and
     * optimal grid index order accordingly.
     *
     * @param cur_pos Current vehicle position.
     * @param cur_vel Current vehicle velocity.
     * @param cur_yaw Current vehicle yaw state.
     * @param refresh_global Full replan flag, updated when incremental fallback is needed.
     * @param global_grid_tour Output polyline tour through global relevant grids.
     * @param optimal_indices Output order of relevant-grid vertices.
     * @param vis_data Optional output used to visualize incremental replanning partitions.
     * @return PLAN_RESULT Planning outcome.
     */
    PLAN_RESULT planCoverageTour(const eigen_utils::Vec3d& cur_pos,
                                 const eigen_utils::Vec3d& cur_vel,
                                 const eigen_utils::Vec3d& cur_yaw, bool& refresh_global,
                                 eigen_utils::Vec_Vec3d& global_grid_tour,
                                 std::vector<int>& optimal_indices,
                                 IncrementalVisualizationData* vis_data = nullptr);
    /**
     * @brief Remove virtual depot and vehicle offsets from raw LKH output.
     *
     * @param raw_optimal_indices Raw tour indices returned by LKH.
     * @param valid_start_idx First valid application-level node index.
     * @param filtered_indices Output indices mapped back to planner-local ids.
     */
    static void extractTourIndices(const std::vector<int>& raw_optimal_indices,
                                   const int valid_start_idx, std::vector<int>& filtered_indices);

  private:
    /**
     * @brief Solve a full global ATSP using all current relevant grids.
     */
    PLAN_RESULT planFullCoverageTour(const eigen_utils::Vec3d& cur_pos,
                                     const eigen_utils::Vec3d& cur_vel,
                                     const eigen_utils::Vec3d& cur_yaw,
                                     eigen_utils::Vec_Vec3d& global_grid_tour,
                                     std::vector<int>& optimal_indices) const;
    /**
     * @brief Build the ATSP cost matrix for full global planning.
     */
    void buildFullCoverageCostMatrix(const eigen_utils::Vec3d& cur_pos,
                                     const eigen_utils::Vec3d& cur_vel,
                                     const eigen_utils::Vec3d& cur_yaw,
                                     Eigen::MatrixXd& global_matrix) const;
    /**
     * @brief Build the reduced ATSP cost matrix for incremental replanning.
     */
    void buildIncrementalCoverageCostMatrix(
        const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
        const eigen_utils::Vec3d& cur_yaw,
        const std::vector<std::vector<int>>& planning_considered_indices,
        std::vector<int>& considered_vertices_indices, Eigen::MatrixXd& global_matrix) const;
    /**
     * @brief Partition the previous global path into local standalone nodes and reserved segments.
     */
    void defineLocalRangeAndPartitionNodes(const eigen_utils::Vec3d& cur_pos,
                                           const double divide_range, const int segment_pt_num,
                                           std::vector<int>& independent_pt_ids,
                                           std::vector<std::vector<int>>& segment_pt_ids,
                                           eigen_utils::Vec3iSet& local_grid_indices_set,
                                           IncrementalVisualizationData* vis_data) const;
    /**
     * @brief Compare the previous global path with the latest relevant-grid snapshot.
     *
     * This step identifies reserved nodes, deleted nodes, and newly added nodes
     * for the incremental replanning pipeline.
     */
    void processAddedAndDeletedNodes(
        const std::vector<int>& independent_pt_ids,
        const std::vector<std::vector<int>>& segment_pt_ids,
        std::vector<int>& independent_reserved_indices,
        std::vector<std::vector<int>>& segment_reserved_indices,
        std::vector<int>& independent_added_indices, std::vector<int>& segment_added_indices,
        const eigen_utils::Vec3iSet& local_grid_indices_set,
        std::vector<map_process::RelevantGridAttributes>& cur_relevant_graph_vertices_data) const;
    /**
     * @brief Flatten reserved and newly added nodes into ATSP planning units.
     */
    static void
    extractNodesForPlanning(const std::vector<int>& independent_reserved_indices,
                            const std::vector<int>& independent_added_indices,
                            const std::vector<std::vector<int>>& segment_reserved_indices,
                            std::vector<std::vector<int>>& planning_considered_indices);
    /**
     * @brief Reconstruct a grid-index path from reduced LKH ordering and preserved segments.
     */
    static void reconstructGlobalPath(const std::vector<int>& lkh_result,
                                      const std::vector<int>& considered_vertices_indices,
                                      const std::vector<std::vector<int>>& segment_reserved_indices,
                                      std::vector<int>& final_indices, bool& precise_global);
    /**
     * @brief Insert newly discovered out-of-range nodes back into the reconstructed global path.
     */
    void insertNewNodes(std::vector<int>& final_indices,
                        const std::vector<int>& segment_added_indices,
                        const std::vector<map_process::RelevantGridAttributes>&
                            cur_relevant_graph_vertices_data) const;

    /// Shared relevant-grid model used for all global coverage reasoning.
    map_process::DynamicExpandingGrid::SharedPtr dynamic_expanding_grid_;
    /// Shared LKH wrapper used to solve ATSP instances.
    LkhInterface& lkh_interface_;
    /// Temporary LKH workspace.
    std::string tsp_dir_;
    /// Active robot id for namespacing LKH problems.
    int drone_id_;
    /// Previous optimal global relevant-grid order used by incremental replanning.
    std::vector<int> last_optimal_grid_indices_;
    /// Snapshot of relevant-grid attributes aligned with @ref last_optimal_grid_indices_.
    std::vector<map_process::RelevantGridAttributes> last_relevant_graph_vertices_data_;
};
} // namespace fastex_explorer

#endif // _GLOBAL_COVERAGE_PLANNER_H_
