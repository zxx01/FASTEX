/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-31 16:02:31
 * @LastEditTime: 2026-05-31 18:02:26
 * @Description:
 */

#include <thread>

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "bspline/BsplineProcess.h"
#include "exploration_manager/expl_data.h"
#include "exploration_manager/runtime/fast_exploration_runtime_adapter.h"
#include "exploration_manager/fastex_exploration_manager.h"
#include "plan_manage/planner_manager.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "traj_utils/planning_visualization.h"
#include "vis_utils/marker_utils.h"

namespace fastex_explorer
{
void FastExplorationRuntimeAdapter::init(
    ros::NodeHandle& nh,
    const std::shared_ptr<FastexExplorationManager>& fastex_expl_manager,
    const std::shared_ptr<fast_planner::FastPlannerManager>& planner_manager,
    const std::shared_ptr<fast_planner::PlanningVisualization>& visualization)
{
    fastex_expl_manager_ = fastex_expl_manager;
    planner_manager_ = planner_manager;
    visualization_ = visualization;

    bspline_client_ = nh.serviceClient<bspline::BsplineProcess>("/planning/bspline_service");
    expl_data_start_client_ =
        nh.serviceClient<std_srvs::Empty>("/data_log_manager_node/explorer_start");
    expl_data_finish_client_ =
        nh.serviceClient<std_srvs::Empty>("/data_log_manager_node/explorer_finish");
    task_complete_pub_ = nh.advertise<std_msgs::Bool>("/task_complete", 1);
    traj_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/traj", 10);
}

bool FastExplorationRuntimeAdapter::pushBsplineToServer(const bspline::Bspline& bspline)
{
    bspline_client_.waitForExistence();
    bspline::BsplineProcess bsp_process;
    bsp_process.request.bspline = bspline;
    return bspline_client_.call(bsp_process);
}

bool FastExplorationRuntimeAdapter::notifyExplorationStart()
{
    std_srvs::Empty trigger_srv;
    expl_data_start_client_.waitForExistence();
    return expl_data_start_client_.call(trigger_srv);
}

bool FastExplorationRuntimeAdapter::notifyExplorationFinish()
{
    std_srvs::Empty trigger_srv;
    expl_data_finish_client_.waitForExistence();
    return expl_data_finish_client_.call(trigger_srv);
}

void FastExplorationRuntimeAdapter::publishTaskComplete() const
{
    std_msgs::Bool msg;
    msg.data = true;
    task_complete_pub_.publish(msg);
    ROS_INFO("Task completed, sent task completion signal.");
}

void FastExplorationRuntimeAdapter::publishTrajectorySample(const eigen_utils::Vec3d& odom_pos)
{
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 0;
    color.b = 0;
    color.a = 1;

    visualization_msgs::Marker marker = vis_utils::marker_utils::makeSphereListMarker(
        "world", "", traj_marker_id_++, vis_utils::marker_utils::makeScale(0.5, 0.5, 0.5), color);
    vis_utils::marker_utils::appendPoint(marker, odom_pos);
    traj_pub_.publish(marker);
}

void FastExplorationRuntimeAdapter::triggerVisualizationAsync() const
{
    const auto fastex_expl_manager = fastex_expl_manager_;
    const auto planner_manager = planner_manager_;
    const auto visualization = visualization_;

    std::thread planning_vis_thread(
        [fastex_expl_manager]() { fastex_expl_manager->visualizePlanningData(); });

    std::thread runtime_vis_thread([fastex_expl_manager, planner_manager, visualization]() {
        auto info = &planner_manager->local_data_;
        auto ed_ptr = fastex_expl_manager->ed_;

        visualization->drawCubes({ed_ptr->next_grid_pos_}, 1.0, Eigen::Vector4d(0, 0, 1, 0.5),
                                 "next_grid_pos", 0, 7);
        visualization->drawLines(ed_ptr->global_grid_tour_, 0.3, Eigen::Vector4d(0.5, 0, 0, 1),
                                 "global_grid_tour", 0, 7);
        visualization->drawLines(ed_ptr->local_cluster_tour_, 0.3, Eigen::Vector4d(0, 0, 0.5, 0.7),
                                 "local_cluster_tour", 0, 6);
        visualization->drawLines(ed_ptr->local_refined_tour_, 0.5, Eigen::Vector4d(0, 0.5, 0, 0.5),
                                 "local_refine_tour", 0, 6);

        visualization->drawCubes({ed_ptr->path_next_goal_pos_}, 1.0,
                                 Eigen::Vector4d(0, 0.5, 0, 0.5), "path_next_goal", 0, 6);
        visualization->drawSpheres({ed_ptr->traj_next_goal_}, 0.3, Eigen::Vector4d(1, 0, 0, 1),
                                   "traj_next_goal", 0, 6);
        visualization->drawPose(ed_ptr->path_next_goal_pos_,
                                eigen_utils::Vec3d(std::cos(ed_ptr->path_next_goal_yaw_),
                                                   std::sin(ed_ptr->path_next_goal_yaw_), 0),
                                1.0, Eigen::Vector4d(1.0, 0.0, 0.0, 1), "path_next_pose", 0, 6);
        visualization->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1),
                                   false, 0.15, Eigen::Vector4d(1, 1, 0, 1));
    });

    planning_vis_thread.detach();
    runtime_vis_thread.detach();
}
} // namespace fastex_explorer
