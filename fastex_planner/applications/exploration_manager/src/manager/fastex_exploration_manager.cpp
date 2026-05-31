/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-03-29 10:35:40
 * @LastEditTime: 2026-05-31 17:28:04
 * @Description:
 */

#include <functional>
#include <stdexcept>

#include <vis_utils/marker_utils.h>

#include "exploration_manager/fastex_exploration_manager.h"
#include "fastex_msgs/DataLog.h"
#include "file_utils/file_rw.h"
#include "map_process/core/map_process_constants.h"
#include "process_utils/process_utils.h"
#include "time_utils/time_utils.h"

namespace fastex_explorer
{
using fast_planner::Astar;

namespace
{
const struct FastexExplorationTimerLogFlags
{
    bool plan_explore_motion{true};
    bool update_exploration_preprocess{true};

    bool plan_global_coverage_path{false};
    bool search_frontier_clusters_in_grid0{false};
    bool plan_local_coverage_path{false};
    bool refine_local_tour{false};
    bool plan_explore_tour{false};

    bool compute_grid_cost_matrix{false};
    bool solve_grid_tsplkh{false};

    bool compute_local_cluster_cost_matrix{false};
    bool solve_local_cluster_tsplkh{false};

    bool compute_local_sop_cost_matrix{false};
} kTimerLogFlags;
} // namespace

/**
 * @brief Load the exploration parameters from the ROS parameter server
 *
 * @param nh The ROS node handle
 */
void FastexExplorationManager::loadParamsFromROS(ros::NodeHandle& nh)
{
    bool is_params_load = true;

    is_params_load &= nh.param("exploration/init_x", ep_->init_x_, 0.0);
    is_params_load &= nh.param("exploration/init_y", ep_->init_y_, 0.0);
    is_params_load &= nh.param("exploration/init_z", ep_->init_z_, 2.0);
    is_params_load &= nh.param("exploration/drone_id", ep_->drone_id_, 0);
    is_params_load &= nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    is_params_load &= nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    is_params_load &= nh.param("exploration/plan_dist", ep_->plan_dist_, 5.0);
    is_params_load &= nh.param("exploration/straight_max_dist", ep_->straight_max_dist_, 12.0);

    if (!is_params_load)
        ROS_ERROR("Failed to load exploration parameters");
}

/**
 * @brief Set the current position of the robot
 *
 * @param pos The current position of the robot
 */
void FastexExplorationManager::setCurrentPosition(const eigen_utils::Vec3d& pos)
{
    cur_pos_ = pos;
    map_process_->setCurrentPosition(pos);

    ed_->traveled_dist_ +=
        (ed_->traveled_trajectory_.empty() ? 0.0 : (pos - ed_->traveled_trajectory_.back()).norm());

    if (ed_->traveled_trajectory_.empty() || (pos - ed_->traveled_trajectory_.back()).norm() > 1e-2)
    {
        ed_->traveled_trajectory_.push_back(pos);
    }
}

/**
 * @brief Transite the exploration phase
 *
 * @param phase The exploration phase
 */
void FastexExplorationManager::transiteExplorationPhase(const EXPL_PHASE& phase)
{
    expl_phase_ = phase;
}

/**
 * @brief Get the exploration phase
 *
 * @return EXPL_PHASE The exploration phase
 */
EXPL_PHASE FastexExplorationManager::getExplorationPhase() { return expl_phase_; }

/**
 * @brief Initialize the exploration manager
 *
 * @param nh The ROS node handle
 */
void FastexExplorationManager::initialize(ros::NodeHandle& nh)
{
    // init planner manager and edt environment
    planner_manager_ = std::make_shared<FastPlannerManager>();
    planner_manager_->initPlanModules(nh);
    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_ = edt_environment_->sdf_map_;

    // init exploration params and data
    last_tour_id_ = 0;
    expl_phase_ = EXPL_PHASE::EXPL;
    home_path_search_type_ = map_process::PATH_SEARCH_TYPE::COARSE;

    ep_ = std::make_shared<ExplorationParams>();
    ed_ = std::make_shared<ExplorationData>();
    loadParamsFromROS(nh);

    // init preprocess ele ments
    map_process_ = std::make_unique<map_process::MapProcess>(nh, edt_environment_);
    map_process_->setInitialPosition(eigen_utils::Vec3d(ep_->init_x_, ep_->init_y_, ep_->init_z_));

    const auto& whole_state_road_map = map_process_->getWholeStateRoadMap();
    const auto& frontier_manager = map_process_->getFrontierManager();
    const auto& dynamic_grid = map_process_->getDynamicExpandingGrid();
    const auto& history_pos_graph = map_process_->getHistoryPosGraph();

    path_searcher_ =
        std::make_shared<map_process::PathSearcher>(nh, edt_environment_, whole_state_road_map);

    frontier_manager->setPathSearcher(path_searcher_);
    dynamic_grid->setPathSearcher(path_searcher_);
    history_pos_graph->setPathSearcher(path_searcher_);

    // Set planner Astar params
    const double resolution_ = sdf_map_->getResolution();
    planner_manager_->path_finder_->lambda_heu_ = 1.0;
    planner_manager_->path_finder_->max_search_time_ = 0.5;
    planner_manager_->path_finder_->setResolution(resolution_);

    // Setup runtime data publishers
    ed_->planning_iter_num_ = 0;
    expl_preprocess_time_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/expl_preprocess_time", 10);
    expl_motion_log_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/expl_motion_log", 10);
    lkh_interface_ = std::make_unique<LkhInterface>();
    global_coverage_planner_ = std::make_unique<GlobalCoveragePlanner>(
        dynamic_grid, *lkh_interface_, ep_->tsp_dir_, ep_->drone_id_);
    local_frontier_planner_ = std::make_unique<LocalFrontierPlanner>(
        frontier_manager, dynamic_grid, path_searcher_, *lkh_interface_, ep_->straight_max_dist_,
        ep_->tsp_dir_, ep_->drone_id_);

    refresh_global_ = true;

    visualization_ = std::make_shared<fast_planner::PlanningVisualization>(nh);

    incremental_global_planning_pub_ =
        nh.advertise<visualization_msgs::Marker>("/planning_vis/incremental_global_planning", 10);
}

/**
 * @brief Plan the exploration motion
 *
 * @param pos The current position of the robot
 * @param vel The current velocity of the robot
 * @param acc The current acceleration of the robot
 * @param yaw The current yaw of the robot
 * @return PLAN_RESULT The result of the planning
 */
PLAN_RESULT FastexExplorationManager::planExploreMotion(const eigen_utils::Vec3d& pos,
                                                        const eigen_utils::Vec3d& vel,
                                                        const eigen_utils::Vec3d& acc,
                                                        const eigen_utils::Vec3d& yaw,
                                                        const PLAN_STATE& plan_state)
{
    const eigen_utils::Vec3d cur_pos = pos;
    const eigen_utils::Vec3d cur_vel = vel;
    const eigen_utils::Vec3d cur_acc = acc;
    const eigen_utils::Vec3d cur_yaw = yaw;

    // Init the exploration data
    eigen_utils::Vec3d next_pos;
    double next_yaw;
    PLAN_RESULT res;

    // 1.Plan the target point to explore
    time_utils::Timer::Ptr timer_motion =
        std::make_shared<time_utils::Timer>("planExploreTargetPoint");
    timer_motion->start();

    res = planExploreTargetPoint(cur_pos, cur_vel, cur_yaw, next_pos, next_yaw, plan_state);

    if (res == PLAN_RESULT::SUCCEED)
    {
        // 2.Plan trajectory (position and yaw) to the target point
        ros::Time start_time = ros::Time::now();

        ed_->path_next_goal_pos_ = next_pos;
        ed_->path_next_goal_yaw_ = next_yaw;
        res = planExploreTrajectory(cur_pos, cur_vel, cur_acc, cur_yaw, next_pos, next_yaw);

        double total_time = (ros::Time::now() - start_time).toSec();
        ROS_ERROR_COND(total_time > 0.1, "trajectory planning time too long!!!");
    }
    else
    {
        ROS_ERROR("Failed to plan the target point to explore.");
    }

    timer_motion->stop(kTimerLogFlags.plan_explore_motion, "ms");

    fastex_msgs::DataLog data_log_msg;
    data_log_msg.iteration_num = ed_->planning_iter_num_;
    data_log_msg.start_time =
        file_utils::formatDouble(timer_motion->getStartTime("us") / 1000000.0, 6);
    data_log_msg.end_time =
        file_utils::formatDouble(timer_motion->getStopTime("us") / 1000000.0, 6);

    expl_motion_log_pub_.publish(data_log_msg);

    return res;
}

void FastexExplorationManager::visualizePlanningData() { map_process_->visualizeData(); }

void FastexExplorationManager::addCurrentPositionToRoadMap(const eigen_utils::Vec3d& pos)
{
    // If the the current position( plan_pos ) is not in the road map, add the current position(
    // plan_pos )
    eigen_utils::Vec3f pos_f = pos.cast<float>();
    int pos_vertex_id;
    const auto& whole_state_road_map = map_process_->getWholeStateRoadMap();

    if (whole_state_road_map->getVertexId(pos_f) >= 0)
    {
        pos_vertex_id = whole_state_road_map->getVertexId(pos_f);
    }
    else
    {
        bool success;
        eigen_utils::Vec_Vec3f near_points;
        std::vector<int> near_ids;
        std::vector<eigen_utils::Vec_Vec3f> paths;
        std::vector<double> dists;

        success = whole_state_road_map->findNearestValidPointsInGraph(
            pos_f, false, near_points, near_ids, paths, dists,
            map_process::constants::kNeighborSearchRange);

        if (success)
        {
            map_process::WSRoadMap::VertexExtraDataType vertex_extra_data(
                map_process::WSRoadMap::VertexExtraDataType::VertexSource::SAMPLE,
                map_process::WSRoadMap::VertexExtraDataType::VertexState::FREE,
                map_process::WSRoadMap::VertexExtraDataType::VertexType::CURRENT_POSITION);
            map_process::WSRoadMap::VertexType vertex(pos_f, true, vertex_extra_data);
            std::tie(pos_vertex_id, std::ignore) = whole_state_road_map->insertVertex(vertex);

            map_process::WSRoadMap::EdgeExtraDataType edge_extra_data(process_utils::CubeBox(),
                                                                      false, false);

            for (size_t i = 0; i < near_points.size(); ++i)
            {
                map_process::WSRoadMap::EdgeType edge(dists[i], true, edge_extra_data);
                whole_state_road_map->addTwoWayEdge(pos_vertex_id, near_ids[i], edge);
            }
        }
        else
        {
            ROS_ERROR("Failed to find nearest valid points in the graph.");
        }
    }
}

/**
 * @brief Update the exploration preprocess data
 *
 * @param pos
 */
void FastexExplorationManager::updateExplorationPreprocessData(const eigen_utils::Vec3d& pos)
{
    time_utils::Timer::Ptr timer_preprocess =
        std::make_shared<time_utils::Timer>("update Exploration Preprocess Data");

    addCurrentPositionToRoadMap(pos);

    // 1.Update Map Process Elements
    timer_preprocess->start();

    map_process_->updateElements(pos, ed_->planning_iter_num_);

    timer_preprocess->stop(kTimerLogFlags.update_exploration_preprocess, "ms");

    fastex_msgs::DataLog data_log_msg;
    data_log_msg.iteration_num = ed_->planning_iter_num_;
    data_log_msg.start_time =
        file_utils::formatDouble(timer_preprocess->getStartTime("us") / 1000000.0, 6);
    data_log_msg.end_time =
        file_utils::formatDouble(timer_preprocess->getStopTime("us") / 1000000.0, 6);

    expl_preprocess_time_pub_.publish(data_log_msg);
}

/**
 * @brief Plan the exploration to the target point
 *
 * @param pos The current position of the robot
 * @param vel The current velocity of the robot
 * @param yaw The current yaw of the robot
 * @param target_pos The target position
 * @param target_yaw The target yaw
 * @return PLAN_RESULT The result of the planning
 */
PLAN_RESULT FastexExplorationManager::planExploreTargetPoint(
    const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& vel, const eigen_utils::Vec3d& yaw,
    eigen_utils::Vec3d& target_pos, double& target_yaw, const PLAN_STATE& plan_state)
{
    eigen_utils::Vec3d default_target_pos;

    // 2.Plan the global tour
    time_utils::Timer::Ptr timer_target = std::make_shared<time_utils::Timer>("planExploreTour");
    timer_target->start();

    if (plan_state == PLAN_STATE::GLOBAL_DECISION)
    {
        if (expl_phase_ == EXPL_PHASE::EXPL) // Exploration phase
        {
            const auto& dynamic_grid = map_process_->getDynamicExpandingGrid();

            updateGlobalTopViewpoints();

            // Plan the global tour
            // 1. Plan the global tour for the DynamicExpandingGrid
            std::vector<int> grid_indices;
            GlobalCoveragePlanner::IncrementalVisualizationData incremental_vis_data;

            time_utils::Timer::Ptr timer_global =
                std::make_shared<time_utils::Timer>("plan Global Coverage Path.");
            timer_global->start();

            PLAN_RESULT res = global_coverage_planner_->planCoverageTour(
                pos, vel, yaw, refresh_global_, ed_->global_grid_tour_, grid_indices,
                &incremental_vis_data);

            timer_global->stop(kTimerLogFlags.plan_global_coverage_path, "ms");

            if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
                return res;

            visualizeIncrementalGlobalPlanning(incremental_vis_data);

            // Search the frontier clusters in the first grid.
            time_utils::Timer::Ptr timer_frontier =
                std::make_shared<time_utils::Timer>("search Frontier Clusters In Grid0");
            timer_frontier->start();

            dynamic_grid->getRelevantFrontierClusters({grid_indices[0]}, ed_->grid_clusters_ids_);

            timer_frontier->stop(kTimerLogFlags.search_frontier_clusters_in_grid0, "ms");

            time_utils::Timer::Ptr timer_local =
                std::make_shared<time_utils::Timer>("plan Local Coverage Path.");
            timer_local->start();

            // 2. Plan the local tour for the frontier clusters
            LocalFrontierPlan local_plan;
            res = local_frontier_planner_->planFrontierTour(pos, vel, yaw, ed_->grid_clusters_ids_,
                                                            grid_indices, ed_->top_vpoints_,
                                                            local_plan);
            if (res == PLAN_RESULT::FAIL || res == PLAN_RESULT::NO_FRONTIER)
                return res;

            ed_->local_cluster_tour_ = local_plan.cluster_tour;
            ed_->local_refined_tour_ = local_plan.refined_tour;
            ed_->chosen_top_vpoint_ = ed_->top_vpoints_[local_plan.refine_cluster_indices[0]].first;
            default_target_pos = local_plan.default_target_pos;
            ed_->global_tour_ =
                local_plan.used_refined_tour ? local_plan.refined_tour : local_plan.cluster_tour;

            timer_local->stop(kTimerLogFlags.plan_local_coverage_path, "ms");
        }
        else if (expl_phase_ == EXPL_PHASE::HOME) // Home phase
        {
            ed_->global_grid_tour_.clear();
            ed_->local_cluster_tour_.clear();
            ed_->grid_clusters_ids_.clear();

            PLAN_RESULT res = planHomePath(pos, default_target_pos, ed_->global_tour_);

            if (res == PLAN_RESULT::FAIL)
                return res;
        }
        else
        {
            throw std::invalid_argument("Invalid exploration phase");
        }
    }
    else if (plan_state == PLAN_STATE::LOCAL_DECISION ||
             plan_state == PLAN_STATE::TRAJECTORY_PLANNING)
    {
        // Do Nothing
    }
    else
    {
        throw std::invalid_argument("Invalid plan state");
    }

    // 4.Get the target position and yaw (truncated from the global tour)
    bool reset_start_idx = (plan_state == PLAN_STATE::GLOBAL_DECISION);

    extractIntemediateTarget(ed_->global_tour_, default_target_pos, ep_->plan_dist_,
                             reset_start_idx, target_pos, target_yaw);

    if (std::fabs(target_yaw - yaw[0]) < 0.1)
        target_yaw += 0.1;

    timer_target->stop(kTimerLogFlags.plan_explore_tour, "ms");

    return PLAN_RESULT::SUCCEED;
}

/**
 * @brief Plan the exploration trajectory to the target position
 *
 * @param pos The current position of the robot
 * @param vel The current velocity of the robot
 * @param acc The current acceleration of the robot
 * @param yaw The current yaw of the robot
 * @param target_pos The target position
 * @param target_yaw The target yaw
 * @return PLAN_RESULT The result of the planning
 */
PLAN_RESULT FastexExplorationManager::planExploreTrajectory(
    const eigen_utils::Vec3d& pos, const eigen_utils::Vec3d& vel, const eigen_utils::Vec3d& acc,
    const eigen_utils::Vec3d& yaw, const eigen_utils::Vec3d& target_pos, const double& target_yaw)
{
    // 1.Compute time lower bound of yaw and use in trajectory generation
    double yaw_diff = std::abs(target_yaw - yaw[0]);
    const auto& searcher_params = path_searcher_->getParams();
    double time_lb = std::min(yaw_diff, 2 * M_PI - yaw_diff) / searcher_params.yd_;

    // 2.Generate trajectory of x,y,z
    planner_manager_->path_finder_->reset();
    if (planner_manager_->path_finder_->search(pos, target_pos, false) == Astar::REACH_END)
    {
        ed_->next_goal_path_ = planner_manager_->path_finder_->getPath();
    }
    else
    {
        double path_len{0.0};
        map_process::PATH_SEARCH_RESULT path_searcher_res =
            path_searcher_->searchCoarsePathWithWSRoadMap(pos, target_pos, ed_->next_goal_path_,
                                                          path_len, false, ep_->straight_max_dist_);

        if (path_searcher_res == map_process::PATH_SEARCH_RESULT::FAIL)
        {
            ROS_ERROR("No path to next viewpoint");
            return PLAN_RESULT::FAIL;
        }
    }
    path_searcher_->shortenPath(ed_->next_goal_path_, 3.0);

    const double radius_far = 7.0;
    const double radius_close = 1.5;
    const double len = Astar::pathLength(ed_->next_goal_path_);
    if (len < radius_close) // too short.
    {
        ROS_INFO("too short.");
        // Next viewpoint is very close, no need to search kinodynamic path,
        // just use waypoints-based optimization
        planner_manager_->planExploreTraj(ed_->next_goal_path_, vel, acc, time_lb);
        ed_->traj_next_goal_ = target_pos;
    }
    else if (len > radius_far) // too long.
    {
        ROS_INFO("too long.");
        // Next viewpoint is far away, select intermediate goal on geometric path (this also deal
        // with dead end)
        double len2 = 0;
        eigen_utils::Vec_Vec3d truncated_path = {ed_->next_goal_path_.front()};
        for (size_t i = 1; i < ed_->next_goal_path_.size() && len2 < radius_far; ++i)
        {
            auto cur_pt = ed_->next_goal_path_[i];
            len2 += (cur_pt - truncated_path.back()).norm();
            truncated_path.push_back(cur_pt);
        }
        ed_->traj_next_goal_ = truncated_path.back();
        planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
    }
    else // Mid goal
    {
        ROS_INFO("Mid goal");
        // Search kino path to exactly next viewpoint and optimize
        ed_->traj_next_goal_ = target_pos;
        if (!planner_manager_->kinodynamicReplan(pos, vel, acc, ed_->traj_next_goal_,
                                                 eigen_utils::Vec3d(0, 0, 0), time_lb))
        {
            return PLAN_RESULT::FAIL;
        }
    }

    if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        ROS_ERROR("Lower bound not satified!");

    // 3.Generate trajectory of yaw
    planner_manager_->planYawExplore(yaw, target_yaw, true, ep_->relax_time_);

    return PLAN_RESULT::SUCCEED;
}

/**
 * @brief Plan the path to the home position
 *
 * @param pos The current position of the robot
 * @param home_pos  The home position
 * @param path  The path to the home position
 * @return PLAN_RESULT  The result of the planning
 */
PLAN_RESULT FastexExplorationManager::planHomePath(const eigen_utils::Vec3d& pos,
                                                   eigen_utils::Vec3d& home_pos,
                                                   eigen_utils::Vec_Vec3d& path)
{
    path.clear();
    home_pos = eigen_utils::Vec3d(ep_->init_x_, ep_->init_y_, ep_->init_z_);
    const auto& whole_state_road_map = map_process_->getWholeStateRoadMap();

    if (sdf_map_->isInflatedOccupied(home_pos))
    {
        std::vector<int> candidate_indices;
        eigen_utils::Vec_Vec3f candidate_positions;
        eigen_utils::Vec3d new_home_position;

        bool success = whole_state_road_map->nearestSearch(
            home_pos.cast<float>(), 100, candidate_positions, candidate_indices, 8.0);

        if (success)
        {
            success = false;

            for (const eigen_utils::Vec3f& candidate_position : candidate_positions)
            {
                new_home_position = candidate_position.cast<double>();
                if (sdf_map_->isInflatedFree(new_home_position))
                {
                    home_pos = new_home_position;
                    success = true;
                    break;
                }
            }
        }

        if (!success)
        {
            success = sdf_map_->bfsNearestFree(eigen_utils::Vec3d(8.0, 8.0, 8.0), home_pos,
                                               new_home_position);
            if (success)
                home_pos = new_home_position;
        }

        if (!success)
        {
            ROS_ERROR("Failed to find a free home position.");
            return PLAN_RESULT::FAIL;
        }

        ep_->init_x_ = home_pos[0];
        ep_->init_y_ = home_pos[1];
        ep_->init_z_ = home_pos[2];
    }

    bool optimistic = false;
    map_process::PATH_SEARCH_RESULT result;
    if (home_path_search_type_ == map_process::PATH_SEARCH_TYPE::FINE)
    {
        double path_len;
        result = path_searcher_->searchFinePath(pos, home_pos, path, path_len);

        path_searcher_->optimizePathWithInterpolation(path, ep_->straight_max_dist_, 2.0,
                                                      optimistic);

        if (path_len > 15.0 || result == map_process::PATH_SEARCH_RESULT::FAIL)
        {
            home_path_search_type_ = map_process::PATH_SEARCH_TYPE::COARSE;
            ROS_WARN("Switch from fine to coarse search.");
        }
    }
    else
    {
        double path_len;
        result = path_searcher_->searchCoarsePathWithWSRoadMap(pos, home_pos, path, path_len,
                                                               optimistic, ep_->straight_max_dist_);
        if (path_len <= 15.0)
        {
            home_path_search_type_ = map_process::PATH_SEARCH_TYPE::FINE;
            ROS_WARN("Switch from coarse to fine search.");
        }
    }

    if (result == map_process::PATH_SEARCH_RESULT::FAIL)
    {
        ROS_ERROR("Failed to plan path to home position.");
        return PLAN_RESULT::FAIL;
    }
    else
    {
        ROS_INFO("Succeed to plan path to home position.");
        return PLAN_RESULT::SUCCEED;
    }
}

/**
 * @brief Visualize the incremental global planning partition.
 *
 */
void FastexExplorationManager::visualizeIncrementalGlobalPlanning(
    const GlobalCoveragePlanner::IncrementalVisualizationData& vis_data)
{
    if (vis_data.independent_pts.empty() && vis_data.segment_pts.empty() &&
        vis_data.segment_edges.empty())
        return;

    visualization_msgs::Marker marker;
    marker = drawSpheres(vis_data.independent_pts, 1.5, eigen_utils::Vec4f(1, 0.84314, 0, 0.8),
                         "independent_pts", 0);
    incremental_global_planning_pub_.publish(marker);
    marker = drawSpheres(vis_data.segment_pts, 1.5, eigen_utils::Vec4f(0.5, 0.80314, 0.9, 0.8),
                         "segment_pts", 0);
    incremental_global_planning_pub_.publish(marker);

    static size_t last_edge_cnt = 0;
    for (size_t i = 0; i < vis_data.segment_edges.size(); ++i)
    {
        const auto& edge = vis_data.segment_edges[i];
        eigen_utils::Vec_Vec3d list1, list2;

        for (size_t j = 0; j + 1 < edge.size(); ++j)
        {
            list1.push_back(edge[j]);
            list2.push_back(edge[j + 1]);
        }

        marker = drawLines(list1, list2, 1.0, eigen_utils::Vec4f(0.5, 0.40314, 0.9, 0.6),
                           "segment_edges", i);
        incremental_global_planning_pub_.publish(marker);
    }

    for (size_t i = vis_data.segment_edges.size(); i < last_edge_cnt; ++i)
    {
        marker =
            drawLines({}, {}, 0.1, eigen_utils::Vec4f(0.5, 0.40314, 0.9, 0.6), "segment_edges", i);
        incremental_global_planning_pub_.publish(marker);
    }

    last_edge_cnt = vis_data.segment_edges.size();
}

/**
 * @brief Update the global top viewpoints
 *
 */
void FastexExplorationManager::updateGlobalTopViewpoints()
{
    // 1.Get the indices of the removed clusters
    static bool first_updated = false;
    std::vector<int> removed_clusters_indices;
    const auto& frontier_manager = map_process_->getFrontierManager();

    if (first_updated)
        frontier_manager->getRemovedActiveClustersIndices(removed_clusters_indices);
    else
        first_updated = true;

    // 2.Get the new global clustered top viewpoints
    ed_->top_vpoints_.clear();
    frontier_manager->getGlobalClusteredTopViewpoints(ed_->top_vpoints_, true);
}

/**
 * @brief Extract an intermediate target from the given path.
 *
 * This function extracts an intermediate target position from the given path based on the specified
 * distance threshold. If the path is empty or the distance threshold is non-positive, the default
 * target position is used. The function also calculates the yaw angle to the target position.
 *
 * @param path The path from which to extract the intermediate target.
 * @param default_target_pos The default target position to use if no valid target is found in the
 * path.
 * @param dist_thresh The distance threshold to determine the intermediate target.
 * @param reset_start_idx Flag to indicate whether to reset the starting index for the search.
 * @param target_pos The extracted intermediate target position.
 * @param target_yaw The yaw angle to the target position.
 */
void FastexExplorationManager::extractIntemediateTarget(
    eigen_utils::Vec_Vec3d& path, const eigen_utils::Vec3d& default_target_pos,
    const double dist_thresh, const bool reset_start_idx, eigen_utils::Vec3d& target_pos,
    double& target_yaw)
{
    // Check if path is empty
    if (path.empty())
    {
        target_pos = default_target_pos;
        target_yaw = process_utils::ProcessUtils::calculateYaw(cur_pos_, target_pos);
        return;
    }

    // Check if dist_thresh is positive
    if (dist_thresh <= 0)
    {
        target_pos = default_target_pos;
        target_yaw = process_utils::ProcessUtils::calculateYaw(cur_pos_, target_pos);
        return;
    }

    double last_len = 0, cur_len = 0;
    bool target_found = false;

    if (reset_start_idx)
        last_tour_id_ = 0;

    // Local term
    if (last_tour_id_ != 0)
    {
        last_len = (path[last_tour_id_] - cur_pos_).norm();
        if (last_len >= dist_thresh)
        {
            target_pos = path[last_tour_id_];
            target_found = true;
        }
    }

    if (!target_found)
    {
        const double step_size = sdf_map_->getResolution();

        for (size_t i = last_tour_id_; i < path.size() - 1; ++i)
        {
            const auto& cur_pt = path[i];
            const auto& next_pt = path[i + 1];

            if ((cur_pt - default_target_pos).norm() < 1e-3)
            {
                target_pos = default_target_pos;
                last_tour_id_ = i;
                target_found = true;
                break;
            }

            cur_len = last_len + (next_pt - cur_pt).norm();
            if (cur_len >= dist_thresh)
            {
                double remaining_dist = dist_thresh - last_len;
                eigen_utils::Vec3d direction = (next_pt - cur_pt).normalized();
                eigen_utils::Vec3d candidate_pos = cur_pt + direction * remaining_dist;

                // check if the candidate viewpoint is obstacle-free
                if (sdf_map_->isInflatedFree(candidate_pos))
                {
                    path.insert(path.begin() + i + 1, candidate_pos);
                    target_pos = candidate_pos;
                    last_tour_id_ = i + 1;
                    target_found = true;
                }
                else
                {
                    // if the candidate is obstructed, try to find another obstacle-free point in
                    // the current segment
                    for (double dist = remaining_dist; dist > 0; dist -= step_size)
                    {
                        candidate_pos = cur_pt + direction * dist;
                        if (sdf_map_->isInflatedFree(candidate_pos))
                        {
                            path.insert(path.begin() + i + 1, candidate_pos);
                            target_pos = candidate_pos;
                            last_tour_id_ = i + 1;
                            target_found = true;
                            break;
                        }
                    }
                }

                if (!target_found)
                {
                    target_pos = next_pt;
                    last_tour_id_ = i + 1;
                    target_found = true;
                }

                break;
            }

            last_len = cur_len;
        }
    }

    // If no target was found, set the target to the default position
    if (!target_found)
        target_pos = default_target_pos;

    // Compute the target yaw
    target_yaw = process_utils::ProcessUtils::calculateYaw(cur_pos_, target_pos);

    // ROS_WARN_STREAM("plan_dist: " << (target_pos - cur_pos_).norm());
}

visualization_msgs::Marker FastexExplorationManager::drawSpheres(const eigen_utils::Vec_Vec3d& list,
                                                                 const double& scale,
                                                                 const eigen_utils::Vec4f& color,
                                                                 const std::string& ns,
                                                                 const int& id)
{
    if (list.empty())
    {
        return vis_utils::marker_utils::makeSphereListMarker(
            "world", ns, id, vis_utils::marker_utils::makeScale(scale, scale, scale),
            vis_utils::marker_utils::toRosColor(color), visualization_msgs::Marker::DELETE);
    }

    visualization_msgs::Marker marker = vis_utils::marker_utils::makeSphereListMarker(
        "world", ns, id, vis_utils::marker_utils::makeScale(scale, scale, scale),
        vis_utils::marker_utils::toRosColor(color));

    vis_utils::marker_utils::appendPoints(marker, list);

    return marker;
}

visualization_msgs::Marker FastexExplorationManager::drawLines(const eigen_utils::Vec_Vec3d& list1,
                                                               const eigen_utils::Vec_Vec3d& list2,
                                                               const double& scale,
                                                               const eigen_utils::Vec4f& color,
                                                               const string& ns, const int& id)
{
    if (list1.empty())
    {
        return vis_utils::marker_utils::makeLineListMarker(
            "world", ns, id, scale, vis_utils::marker_utils::toRosColor(color),
            visualization_msgs::Marker::DELETE);
    }

    visualization_msgs::Marker marker = vis_utils::marker_utils::makeLineListMarker(
        "world", ns, id, scale, vis_utils::marker_utils::toRosColor(color));

    for (size_t i = 0; i < list1.size(); ++i)
    {
        vis_utils::marker_utils::appendLine(marker, list1[i], list2[i]);
    }

    return marker;
}

} // namespace fastex_explorer
