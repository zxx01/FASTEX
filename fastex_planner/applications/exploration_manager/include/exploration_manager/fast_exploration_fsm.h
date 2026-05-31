/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-15 10:54:44
 * @LastEditTime: 2026-05-05 00:01:18
 * @Description:
 */
#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "exploration_manager/runtime/fast_exploration_runtime_adapter.h"
#include "exploration_manager/fastex_exploration_manager.h"

namespace fastex_explorer
{
struct FSMParam;
struct FSMData;

enum class EXPL_STATE : int8_t
{
    INIT,
    WAIT_TRIGGER,
    PLAN_TRAJ,
    PUB_TRAJ,
    EXEC_TRAJ,
    FINISH
};

class FastExplorationFSM
{
  private:
    /* planning utils */
    std::shared_ptr<FastexExplorationManager> fastex_expl_manager_;
    std::shared_ptr<fast_planner::FastPlannerManager> planner_manager_;
    FastExplorationRuntimeAdapter::UniquePtr runtime_adapter_;

    std::shared_ptr<FSMParam> fp_;
    std::shared_ptr<FSMData> fd_;

    EXPL_STATE expl_state_;
    fastex_explorer::PLAN_STATE plan_state_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_, frontier_timer_, traj_pub_timer_;
    ros::Subscriber trigger_sub_, odom_sub_, forced_back_to_origin_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, iteration_time_pub_, expl_iter_log_pub_;

    /* helper functions */
    fastex_explorer::PLAN_RESULT callExplorationPlanner();
    void transitState(const EXPL_STATE& new_state, const std::string& pos_call);

    /* ROS functions */
    void trajPubCallback(const ros::TimerEvent& e);

    void FSMCallback(const ros::TimerEvent& e);
    void safetyCallback(const ros::TimerEvent& e);
    void frontierCallback(const ros::TimerEvent& e);
    void triggerCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void manualStopExplorationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  public:
    FastExplorationFSM() = default;
    ~FastExplorationFSM() = default;

    void init(ros::NodeHandle& nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fastex_explorer

#endif
