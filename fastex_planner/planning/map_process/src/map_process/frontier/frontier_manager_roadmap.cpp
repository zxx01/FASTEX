/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-04-30
 * @LastEditTime: 2026-05-30 20:56:46
 * @Description:
 */

#include "map_process/frontier/frontier_manager.h"

#include "process_utils/process_utils.h"

namespace map_process
{
namespace
{
/**
 * @brief Snapshot of a newly generated frontier-cluster top viewpoint.
 *
 * This struct decouples roadmap insertion from the active cluster container by
 * storing only the cluster id, viewpoint position and the inserted roadmap id.
 */
struct PendingTopViewpoint
{
    int cluster_id = -1;
    eigen_utils::Vec3d pos;
    int roadmap_id = -1;
};

/**
 * @brief Insert a roadmap vertex and connect it to nearby valid roadmap vertices.
 *
 * @param ws_road_map Target whole-state roadmap.
 * @param pos Position of the vertex to insert.
 * @param vertex_type Semantic type of the inserted roadmap vertex.
 * @param failure_context Warning text emitted when adding an edge fails.
 * @param inserted_vertex_id Output roadmap vertex id.
 * @return true if a valid insertion anchor is found and the vertex is inserted.
 * @return false if no nearby valid roadmap connection can be found.
 */
bool insertRoadMapVertexWithNearestEdges(
    const WSRoadMap::SharedPtr& ws_road_map, const eigen_utils::Vec3f& pos,
    const WSRoadMap::VertexExtraDataType::VertexType vertex_type, const char* failure_context,
    int& inserted_vertex_id)
{
    eigen_utils::Vec_Vec3f near_points;
    std::vector<int> near_ids;
    std::vector<eigen_utils::Vec_Vec3f> paths;
    std::vector<double> dists;

    const bool success = ws_road_map->findNearestValidPointsInGraph(pos, false, near_points,
                                                                    near_ids, paths, dists, 8.0);
    if (!success)
        return false;

    WSRoadMap::VertexExtraDataType vertex_extra_data(
        WSRoadMap::VertexExtraDataType::VertexSource::SAMPLE,
        WSRoadMap::VertexExtraDataType::VertexState::FREE, vertex_type);
    WSRoadMap::VertexType vertex(pos, true, vertex_extra_data);
    std::tie(inserted_vertex_id, std::ignore) = ws_road_map->insertVertex(vertex);

    WSRoadMap::EdgeExtraDataType edge_extra_data(process_utils::CubeBox(), false, false);
    for (size_t j = 0; j < near_ids.size(); ++j)
    {
        WSRoadMap::EdgeType edge(dists[j], true, edge_extra_data);
        const bool edge_added = ws_road_map->addTwoWayEdge(inserted_vertex_id, near_ids[j], edge);
        if (!edge_added)
            ROS_WARN("%s", failure_context);
    }

    return true;
}

/**
 * @brief Remove obsolete top viewpoints from both roadmap and history graph.
 *
 * @param whole_state_road_map Whole-state roadmap used by the exploration stack.
 * @param history_pos_graph History-position graph used for topological traces.
 * @param removed_cluster_topvp_rm_ids Cached removed cluster roadmap ids and anchor positions.
 */
void removeObsoleteTopViewpoints(
    const WSRoadMap::SharedPtr& whole_state_road_map,
    const HistoryPosGraph::SharedPtr& history_pos_graph,
    const std::vector<std::pair<int, eigen_utils::Vec3d>>& removed_cluster_topvp_rm_ids)
{
    for (const auto& [rmv_id, pos_d] : removed_cluster_topvp_rm_ids)
    {
        if (rmv_id != -1)
            whole_state_road_map->deleteVertex(rmv_id);

        history_pos_graph->deleteVertex(history_pos_graph->getVertexId(pos_d.cast<float>()));
    }
}

/**
 * @brief Snapshot the top viewpoints of newly appended active clusters.
 *
 * The function reads active clusters under a shared lock and stores only the
 * data needed by the roadmap update path.
 *
 * @param active_clusters Active frontier-cluster list.
 * @param active_clusters_mutex Mutex guarding the active cluster list.
 * @param new_cluster_start_index First cluster index considered newly added.
 * @param pending_top_viewpoints Output snapshot vector.
 */
void collectPendingTopViewpoints(
    const std::list<std::shared_ptr<FrontierClusterInfo>>& active_clusters,
    std::shared_mutex& active_clusters_mutex, const int new_cluster_start_index,
    std::vector<PendingTopViewpoint>& pending_top_viewpoints)
{
    std::shared_lock<std::shared_mutex> active_lock(active_clusters_mutex);
    if (static_cast<int>(active_clusters.size()) <= new_cluster_start_index)
        return;

    auto it = active_clusters.begin();
    std::advance(it, new_cluster_start_index);

    pending_top_viewpoints.reserve(active_clusters.size() - new_cluster_start_index);
    for (; it != active_clusters.end(); ++it)
    {
        if ((*it)->viewpoints_.empty())
        {
            ROS_WARN("Skip updating top viewpoint in WS roadmap because cluster %d has no "
                     "viewpoint.",
                     (*it)->id_);
            continue;
        }

        pending_top_viewpoints.push_back({(*it)->id_, (*it)->viewpoints_.front().pos_, -1});
    }
}

/**
 * @brief Insert pending cluster top viewpoints into the whole-state roadmap.
 *
 * @param whole_state_road_map Whole-state roadmap used by the exploration stack.
 * @param pending_top_viewpoints Snapshot viewpoints to be inserted. Successful
 * insertions write back their roadmap ids into this vector.
 */
void insertPendingTopViewpointsIntoRoadMap(const WSRoadMap::SharedPtr& whole_state_road_map,
                                           std::vector<PendingTopViewpoint>& pending_top_viewpoints)
{
    for (auto& pending_top_viewpoint : pending_top_viewpoints)
    {
        const eigen_utils::Vec3f vp_f = pending_top_viewpoint.pos.cast<float>();
        int vp_id = -1;
        const bool inserted = insertRoadMapVertexWithNearestEdges(
            whole_state_road_map, vp_f, WSRoadMap::VertexExtraDataType::VertexType::VIEWPOINT,
            "Failed to add edge between the top viewpoint and the nearest vertex on the roadmap.",
            vp_id);

        if (!inserted)
        {
            ROS_WARN("Failed to insert the top viewpoint into the WS roadmap.");
            continue;
        }

        pending_top_viewpoint.roadmap_id = vp_id;
    }
}

/**
 * @brief Write inserted roadmap ids back to the owning active clusters.
 *
 * @param active_clusters Active frontier-cluster list.
 * @param active_clusters_mutex Mutex guarding the active cluster list.
 * @param pending_top_viewpoints Snapshot viewpoints carrying inserted roadmap ids.
 */
void writeBackTopViewpointRoadMapIds(
    std::list<std::shared_ptr<FrontierClusterInfo>>& active_clusters,
    std::shared_mutex& active_clusters_mutex,
    const std::vector<PendingTopViewpoint>& pending_top_viewpoints)
{
    std::unique_lock<std::shared_mutex> active_lock(active_clusters_mutex);
    for (const auto& pending_top_viewpoint : pending_top_viewpoints)
    {
        if (pending_top_viewpoint.roadmap_id == -1)
            continue;

        for (const auto& cluster : active_clusters)
        {
            if (cluster->id_ == pending_top_viewpoint.cluster_id)
            {
                cluster->top_vp_roadmap_id_ = pending_top_viewpoint.roadmap_id;
                break;
            }
        }
    }
}

/**
 * @brief Ensure the current plan position exists as a roadmap vertex.
 *
 * @param whole_state_road_map Whole-state roadmap used by the exploration stack.
 * @param plan_position_f Current plan position in float coordinates.
 * @return Roadmap vertex id of the current plan position, or `-1` on failure.
 */
int ensurePlanPositionVertex(const WSRoadMap::SharedPtr& whole_state_road_map,
                             const eigen_utils::Vec3f& plan_position_f)
{
    int plan_pos_id = whole_state_road_map->getVertexId(plan_position_f);
    if (plan_pos_id != -1)
        return plan_pos_id;

    const bool inserted = insertRoadMapVertexWithNearestEdges(
        whole_state_road_map, plan_position_f,
        WSRoadMap::VertexExtraDataType::VertexType::CURRENT_POSITION,
        "Failed to add edge between the plan position and the nearest vertex on the roadmap.",
        plan_pos_id);
    if (!inserted)
    {
        ROS_ERROR("Failed to insert the plan position into the WS roadmap.");
        return -1;
    }

    return plan_pos_id;
}

/**
 * @brief Connect top viewpoints to the current plan position when roadmap paths are missing.
 *
 * @param path_searcher Path-search module used for bounded path search.
 * @param whole_state_road_map Whole-state roadmap used for roadmap queries.
 * @param plan_position Current plan position in double coordinates.
 * @param plan_position_f Current plan position in float coordinates.
 * @param plan_pos_id Roadmap vertex id of the current plan position.
 * @param pending_top_viewpoints Snapshot viewpoints to be connected.
 */
void connectTopViewpointsToPlanPosition(
    const PathSearcher::SharedPtr& path_searcher, const WSRoadMap::SharedPtr& whole_state_road_map,
    const eigen_utils::Vec3d& plan_position, const eigen_utils::Vec3f& plan_position_f,
    const int plan_pos_id, const std::vector<PendingTopViewpoint>& pending_top_viewpoints)
{
    if (plan_pos_id == -1)
        return;

    const eigen_utils::Vec3d margin(8.0, 8.0, 8.0);
    process_utils::CubeBox confined_box;
    eigen_utils::Vec_Vec3f path_f;
    eigen_utils::Vec_Vec3d path_d;
    double dist = 0.0;

    for (const auto& pending_top_viewpoint : pending_top_viewpoints)
    {
        const eigen_utils::Vec3f vp_f = pending_top_viewpoint.pos.cast<float>();
        confined_box.min_ = vp_f.cwiseMin(plan_position_f).cast<double>() - margin;
        confined_box.max_ = vp_f.cwiseMax(plan_position_f).cast<double>() + margin;

        const bool path_exists = whole_state_road_map->findShortestPathWithinBox(
            vp_f, plan_position_f, confined_box, false, path_f, dist);

        if (path_exists)
            continue;

        const PATH_SEARCH_RESULT result = path_searcher->searchFineBoundedPath(
            pending_top_viewpoint.pos, plan_position, confined_box.min_ + margin / 2,
            confined_box.max_ - margin / 2, path_d, dist, -1.0, false);

        if (result == PATH_SEARCH_RESULT::FAIL)
            dist = (plan_position - pending_top_viewpoint.pos).norm() * 2;

        WSRoadMap::EdgeExtraDataType edge_extra_data(process_utils::CubeBox(), false, false);
        WSRoadMap::EdgeType edge(dist, true, edge_extra_data);

        const int vp_id = whole_state_road_map->getVertexId(vp_f);
        const bool edge_added = whole_state_road_map->addTwoWayEdge(vp_id, plan_pos_id, edge);
        if (!edge_added)
            ROS_WARN("Failed to add edge between the top viewpoint and the plan position on the "
                     "roadmap.");
    }
}

/**
 * @brief Mirror inserted top viewpoints into the history position graph.
 *
 * @param history_pos_graph History-position graph used by the exploration stack.
 * @param pending_top_viewpoints Snapshot viewpoints to be inserted.
 */
void insertTopViewpointsIntoHistoryGraph(
    const HistoryPosGraph::SharedPtr& history_pos_graph,
    const std::vector<PendingTopViewpoint>& pending_top_viewpoints)
{
    const eigen_utils::Vec3f last_history_pos_f = history_pos_graph->getLastKeyPos();
    const int last_history_pos_id = history_pos_graph->getVertexId(last_history_pos_f);
    if (last_history_pos_id < 0)
        return;

    for (const auto& pending_top_viewpoint : pending_top_viewpoints)
    {
        const eigen_utils::Vec3f vp_f = pending_top_viewpoint.pos.cast<float>();
        auto [v_id, inserted] =
            history_pos_graph->insertVertex(HistoryPosGraph::GraphVertexType(vp_f, true));
        const bool edge_added = history_pos_graph->addTwoWayEdge(
            v_id, last_history_pos_id,
            HistoryPosGraph::GraphEdgeType((vp_f - last_history_pos_f).norm(), true));

        if (!edge_added)
            ROS_WARN("Failed to add edge between the top viewpoint and the last history "
                     "position on the roadmap.");
    }
}
} // namespace

/**
 * @brief Synchronize frontier-cluster top viewpoints with roadmap and history graph.
 *
 * The update performs four steps:
 * 1. Remove obsolete top viewpoints belonging to deleted clusters.
 * 2. Snapshot top viewpoints from newly added active clusters.
 * 3. Insert valid viewpoints into the whole-state roadmap and write back roadmap ids.
 * 4. Connect viewpoints to the current plan position and mirror them into the history graph.
 *
 * @param plan_position Current planning position used as the connectivity anchor.
 */
void FrontierManager::updateTopViewpointsInWSRoadMap(const eigen_utils::Vec3d& plan_position)
{
    // Step 1: Remove obsolete top viewpoints from both roadmap and history graph, and clear the
    // cache.
    removeObsoleteTopViewpoints(whole_state_road_map_, history_pos_graph_,
                                removed_cluster_topvp_rm_ids_);
    removed_cluster_topvp_rm_ids_.clear();

    std::vector<PendingTopViewpoint> pending_top_viewpoints;
    collectPendingTopViewpoints(active_frontier_clusters_info_ptr_,
                                active_frontier_clusters_info_mutex_, new_cluster_start_index_,
                                pending_top_viewpoints);
    if (pending_top_viewpoints.empty())
        return;

    insertPendingTopViewpointsIntoRoadMap(whole_state_road_map_, pending_top_viewpoints);
    writeBackTopViewpointRoadMapIds(active_frontier_clusters_info_ptr_,
                                    active_frontier_clusters_info_mutex_, pending_top_viewpoints);

    const eigen_utils::Vec3f plan_position_f = plan_position.cast<float>();
    const int plan_pos_id = ensurePlanPositionVertex(whole_state_road_map_, plan_position_f);
    connectTopViewpointsToPlanPosition(path_searcher_, whole_state_road_map_, plan_position,
                                       plan_position_f, plan_pos_id, pending_top_viewpoints);
    insertTopViewpointsIntoHistoryGraph(history_pos_graph_, pending_top_viewpoints);
}

} // namespace map_process
