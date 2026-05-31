#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include "map_process/core/map_process_constants.h"

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include "bspline/BsplineProcess.h"
#include "fastex_msgs/DataLog.h"
#include "file_utils/file_rw.h"
#include "process_utils/process_utils.h"
#include "std_srvs/Empty.h"
#include "time_utils/time_utils.h"

namespace fastex_explorer
{
namespace
{
const struct FastExplorationFsmTimerLogFlags
{
    bool exploration_iteration{false};
    bool exploration_planning{false};
} kTimerLogFlags;
} // namespace

void FastExplorationFSM::init(ros::NodeHandle& nh)
{
    fp_ = std::make_shared<FSMParam>();
    fd_ = std::make_shared<FSMData>();

    /*  Fsm param  */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/auto_start", fp_->auto_start_, false);

    /* Initialize main modules */
    fastex_expl_manager_ = std::make_shared<FastexExplorationManager>();
    fastex_expl_manager_->initialize(nh);
    auto visualization = std::make_shared<fast_planner::PlanningVisualization>(nh);

    planner_manager_ = fastex_expl_manager_->planner_manager_;
    runtime_adapter_ = std::make_unique<FastExplorationRuntimeAdapter>();
    runtime_adapter_->init(nh, fastex_expl_manager_, planner_manager_, visualization);
    expl_state_ = EXPL_STATE::INIT;
    plan_state_ = fastex_explorer::PLAN_STATE::IDLE;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH"};
    fd_->static_state_ = true;
    fd_->trigger_ = false;
    fd_->refresh_plan_ = false;

    /* Ros sub, pub and timer */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
    traj_pub_timer_ =
        nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::trajPubCallback, this);
    frontier_timer_ =
        nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

    trigger_sub_ =
        nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback2, this);
    odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);
    forced_back_to_origin_sub_ = nh.subscribe(
        "/move_base_simple/goal", 1, &FastExplorationFSM::manualStopExplorationCallback, this);

    replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
    new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
    bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
    expl_iter_log_pub_ =
        nh.advertise<fastex_msgs::DataLog>("/data_log_manager_node/expl_iteration_time", 10);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e)
{
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "[FSM]: state: " << fd_->state_str_[static_cast<int>(expl_state_)]);

    switch (expl_state_)
    {
    case EXPL_STATE::INIT:
    {
        // Wait for odometry ready
        if (!fd_->have_odom_)
        {
            ROS_WARN_THROTTLE(1.0, "no odom.");
            return;
        }

        // Go to wait trigger when odom is ok
        transitState(EXPL_STATE::WAIT_TRIGGER, "FSM");
        fastex_expl_manager_->transiteExplorationPhase(fastex_explorer::EXPL_PHASE::EXPL);

        break;
    }

    case EXPL_STATE::WAIT_TRIGGER:
    {
        // Do nothing but wait for trigger
        ROS_WARN_THROTTLE(1.0, "wait for trigger.");
        break;
    }

    case EXPL_STATE::FINISH:
    {
        ROS_WARN_THROTTLE(1.0, "finish exploration.");
        runtime_adapter_->publishTaskComplete();

        break;
    }

    case EXPL_STATE::PLAN_TRAJ:
    {
        // ROS_WARN("Odom: (%f, %f, %f)", fd_->odom_pos_(0), fd_->odom_pos_(1), fd_->odom_pos_(2));

        if (fastex_expl_manager_->getExplorationPhase() == fastex_explorer::EXPL_PHASE::HOME &&
            (fd_->odom_pos_ - eigen_utils::Vec3d(fastex_expl_manager_->ep_->init_x_,
                                                 fastex_expl_manager_->ep_->init_y_,
                                                 fastex_expl_manager_->ep_->init_z_))
                    .cwiseAbs()
                    .maxCoeff() < 1.0)
        {
            fd_->static_state_ = true;
            transitState(EXPL_STATE::FINISH, "FSM");
            break;
        }

        // Plan from static state (hover)
        if (fd_->static_state_)
        {
            fd_->start_pt_ = fd_->odom_pos_;
            fd_->start_vel_ = fd_->odom_vel_;
            fd_->start_acc_.setZero();

            fd_->start_yaw_(0) = fd_->odom_yaw_;
            fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
        }
        else // Replan from non-static state, starting from 'replan_time' seconds later
        {
            fast_planner::LocalTrajData* info = &planner_manager_->local_data_;
            double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

            // Obtain the initial pose from the previous B-spline trajectory.
            // The next start state is the state of the current trajectory evaluated
            // at replan_time_ seconds ahead from the current time.
            fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
            fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
            fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
            fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
            fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
        }

        // Inform traj_server the replanning
        replan_pub_.publish(std_msgs::Empty()); // publish empty message as a trigger

        ++(fastex_expl_manager_->ed_->planning_iter_num_);

        time_utils::Timer::Ptr expl_planning_timer =
            std::make_shared<time_utils::Timer>("expl_planning_timer");
        expl_planning_timer->start();
        fastex_explorer::PLAN_RESULT res = callExplorationPlanner();
        expl_planning_timer->stop(kTimerLogFlags.exploration_iteration, "ms");

        if (res == fastex_explorer::PLAN_RESULT::SUCCEED)
        {
            transitState(EXPL_STATE::PUB_TRAJ, "FSM");

            fastex_msgs::DataLog data_log_msg;
            data_log_msg.iteration_num = fastex_expl_manager_->ed_->planning_iter_num_;
            data_log_msg.start_time =
                file_utils::formatDouble(expl_planning_timer->getStartTime("us") / 1e6, 6);
            data_log_msg.end_time =
                file_utils::formatDouble(expl_planning_timer->getStopTime("us") / 1e6, 6);

            expl_iter_log_pub_.publish(data_log_msg);
        }
        else if (res == fastex_explorer::PLAN_RESULT::NO_FRONTIER)
        {
            ROS_WARN("NO_FRONTIER, PLAN HOME");
            fastex_expl_manager_->transiteExplorationPhase(fastex_explorer::EXPL_PHASE::HOME);
            plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;

            // Inform data_log_manager to finish exploration
            if (!runtime_adapter_->notifyExplorationFinish())
                ROS_ERROR("Failed to call service expl_data_finish");
        }
        else if (res == fastex_explorer::PLAN_RESULT::FAIL)
        {
            // Still in PLAN_TRAJ state, keep replanning
            ROS_WARN("plan fail");
            fd_->static_state_ = true;
        }
        break;
    }

    case EXPL_STATE::PUB_TRAJ:
    {
        double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
        if (dt > 0)
        {
            if (!runtime_adapter_->pushBsplineToServer(fd_->newest_traj_))
                return;

            fd_->static_state_ = false;
            transitState(EXPL_STATE::EXEC_TRAJ, "FSM");

            // Visualization
            runtime_adapter_->triggerVisualizationAsync();
        }
        break;
    }

    case EXPL_STATE::EXEC_TRAJ:
    {
        fast_planner::LocalTrajData* info = &planner_manager_->local_data_;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();

        // Replan if traj is almost fully executed
        double time_to_end = info->duration_ - t_cur;
        if (time_to_end < fp_->replan_thresh1_)
        {
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;
            ROS_WARN("Replan: traj fully executed.");
            return;
        }

        // Replan if next frontier to be visited is covered
        if (fastex_expl_manager_->getExplorationPhase() != fastex_explorer::EXPL_PHASE::HOME &&
            t_cur > fp_->replan_thresh2_ &&
            fastex_expl_manager_->map_process_->getFrontierManager()->isFrontierCovered())
        {
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;
            ROS_WARN("Replan: cluster covered.");
            return;
        }
        // Replan after some time
        if (t_cur > fp_->replan_thresh3_)
        {
            transitState(EXPL_STATE::PLAN_TRAJ, "FSM");
            plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;
            ROS_WARN("Replan: periodic call.");
        }

        break;
    }
    }
}

fastex_explorer::PLAN_RESULT FastExplorationFSM::callExplorationPlanner()
{
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    fastex_explorer::PLAN_RESULT res;

    // if multiple consecutive local planning attempts fail, fall back to global planning
    if (plan_state_ == fastex_explorer::PLAN_STATE::LOCAL_DECISION ||
        plan_state_ == fastex_explorer::PLAN_STATE::TRAJECTORY_PLANNING)
        fd_->continuous_local_times_++;
    else
        fd_->continuous_local_times_ = 0;

    if (fd_->continuous_local_times_ > 2)
    {
        plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;
        fd_->continuous_local_times_ = 0;
    }

    time_utils::Timer::Ptr timer = std::make_shared<time_utils::Timer>("exploration_planning");
    timer->start();

    {
        std::lock_guard<std::mutex> lock(planner_manager_->edt_environment_->sdf_map_->sdf_mutex_);
        if (plan_state_ == fastex_explorer::PLAN_STATE::GLOBAL_DECISION ||
            plan_state_ == fastex_explorer::PLAN_STATE::LOCAL_DECISION)
        {
            fastex_expl_manager_->updateExplorationPreprocessData(fd_->start_pt_);
        }

        res = fastex_expl_manager_->planExploreMotion(
            fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_, plan_state_);
    }

    timer->stop(kTimerLogFlags.exploration_planning, "ms");

    if (fd_->refresh_plan_)
    {
        fd_->refresh_plan_ = false;
        res = fastex_explorer::PLAN_RESULT::FAIL;
    }

    if (res == fastex_explorer::PLAN_RESULT::SUCCEED)
    {
        auto info = &planner_manager_->local_data_;
        info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now()
                                                                    : time_r; // actual start time

        bspline::Bspline bspline;
        bspline.order = planner_manager_->pp_.bspline_degree_;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;
        bspline.duration_time = info->duration_;
        Eigen::MatrixXd pos_pts =
            info->position_traj_
                .getControlPoint(); // extract control points of the position B-spline trajectory
        for (int i = 0; i < pos_pts.rows(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(i, 0);
            pt.y = pos_pts(i, 1);
            pt.z = pos_pts(i, 2);
            bspline.pos_pts.push_back(pt);
        }
        Eigen::VectorXd knots =
            info->position_traj_
                .getKnot(); // extract knot vector of the position B-spline trajectory
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }
        Eigen::MatrixXd yaw_pts =
            info->yaw_traj_
                .getControlPoint(); // extract control points of the yaw B-spline trajectory
        for (int i = 0; i < yaw_pts.rows(); ++i)
        {
            double yaw = yaw_pts(i, 0);
            bspline.yaw_pts.push_back(yaw);
        }
        bspline.yaw_dt =
            info->yaw_traj_.getKnotSpan(); // extract knot span of the yaw B-spline trajectory

        fd_->newest_traj_ = bspline;
    }
    return res;
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e)
{
    static int delay = 0;
    if (++delay < 5 || !fd_->have_odom_)
        return;

    if (expl_state_ == EXPL_STATE::WAIT_TRIGGER || expl_state_ == EXPL_STATE::FINISH)
    {
        if (fastex_expl_manager_->map_process_->updateElements(fd_->odom_pos_, -1) &&
            expl_state_ == EXPL_STATE::WAIT_TRIGGER && fp_->auto_start_ && !fd_->trigger_)
        {
            triggerCallback2(nullptr);
        }
    }
}

void FastExplorationFSM::triggerCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (expl_state_ != EXPL_STATE::WAIT_TRIGGER)
        return;
    fd_->trigger_ = true;
    ROS_INFO("Triggered!");

    if (!runtime_adapter_->notifyExplorationStart())
        ROS_ERROR("Failed to call service expl_data_start");

    transitState(EXPL_STATE::PLAN_TRAJ, "triggerCallback");
    plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e)
{
    if (expl_state_ == EXPL_STATE::EXEC_TRAJ)
    {
        // Check safety and trigger replan if necessary
        double dist;
        bool safe = planner_manager_->checkTrajCollision(dist);
        if (!safe)
        {
            ROS_WARN("Replan: collision detected!");
            transitState(EXPL_STATE::PLAN_TRAJ, "safetyCallback");
            plan_state_ = fastex_explorer::PLAN_STATE::TRAJECTORY_PLANNING;
        }
    }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // receive odometry position
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;
    // receive odometry linear velocity
    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    // receive odometry attitude (orientation)
    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    // convert quaternion to rotation matrix -> to extract yaw; Eigen uses ZYX (RPY) convention
    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0)); // ZYX Euler convention

    fd_->have_odom_ = true;

    fastex_expl_manager_->setCurrentPosition(fd_->odom_pos_);
}

void FastExplorationFSM::transitState(const EXPL_STATE& new_state, const std::string& pos_call)
{
    int pre_s = static_cast<int>(expl_state_);
    expl_state_ = new_state;
    ROS_INFO_STREAM("[" << pos_call << "]: from " << fd_->state_str_[pre_s] << " to "
                        << fd_->state_str_[static_cast<int>(new_state)]);
}

void FastExplorationFSM::manualStopExplorationCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if ((expl_state_ == EXPL_STATE::PLAN_TRAJ || expl_state_ == EXPL_STATE::EXEC_TRAJ) &&
        (std::fabs(msg->pose.position.x - fastex_expl_manager_->ep_->init_x_) < 1.0 &&
         std::fabs(msg->pose.position.y - fastex_expl_manager_->ep_->init_y_) < 1.0))
    {
        if (expl_state_ == EXPL_STATE::PLAN_TRAJ)
            fd_->refresh_plan_ = true;

        ROS_ERROR("-----------------manualStopExploration!----------------");
        transitState(EXPL_STATE::PLAN_TRAJ, "manualStopExploration");
        fastex_expl_manager_->transiteExplorationPhase(fastex_explorer::EXPL_PHASE::HOME);
        plan_state_ = fastex_explorer::PLAN_STATE::GLOBAL_DECISION;

        // Inform data_log_manager to finish exploration
        if (!runtime_adapter_->notifyExplorationFinish())
            ROS_ERROR("Failed to call service expl_data_finish");
    }
}

void FastExplorationFSM::trajPubCallback(const ros::TimerEvent& e)
{
    runtime_adapter_->publishTrajectorySample(fd_->odom_pos_);
}
} // namespace fastex_explorer
