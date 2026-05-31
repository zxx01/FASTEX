#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <bspline/Bspline.h>
#include <string>
#include <utility>
#include <vector>

#include "common_utils/eigen_utils.h"

namespace fastex_explorer
{
template <typename T> using PointMap = eigen_utils::Vec3dMap<T, 3>;

struct FSMParam
{
    double replan_thresh1_;
    double replan_thresh2_;
    double replan_thresh3_;
    double replan_time_; // second
    bool auto_start_;
};

struct FSMData
{
    // FSM data
    bool trigger_, have_odom_, static_state_, refresh_plan_;
    std::vector<std::string> state_str_;

    eigen_utils::Vec3d odom_pos_, odom_vel_; // odometry state
    Eigen::Quaterniond odom_orient_;
    double odom_yaw_;

    eigen_utils::Vec3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    bspline::Bspline newest_traj_;

    int continuous_local_times_;
};

struct ExplorationParams
{
    double init_x_, init_y_, init_z_;
    int drone_id_;

    std::string tsp_dir_; // resource dir of tsp solver
    double relax_time_;   // relax time for yaw bspine

    double plan_dist_;
    double straight_max_dist_;
};

struct ExplorationData
{
    eigen_utils::Vec_Vec3d traveled_trajectory_;

    // frontier data
    eigen_utils::Vec_Vec3d cluster_centroids_;
    std::vector<eigen_utils::Vec_Vec3d> cluster_frontiers_;

    // view point data
    std::vector<std::pair<eigen_utils::Vec3d, double>> top_vpoints_;

    eigen_utils::Vec_Vec3d global_tour_;
    eigen_utils::Vec_Vec3d global_grid_tour_;
    eigen_utils::Vec_Vec3d local_cluster_tour_;
    eigen_utils::Vec_Vec3d local_refined_tour_;

    eigen_utils::Vec3d next_grid_pos_;
    std::vector<int> grid_clusters_ids_;

    eigen_utils::Vec3d chosen_top_vpoint_;
    eigen_utils::Vec3d traj_next_goal_;
    eigen_utils::Vec3d path_next_goal_pos_;
    eigen_utils::Vec_Vec3d next_goal_path_;
    double path_next_goal_yaw_;

    int planning_iter_num_{0};
    double traveled_dist_{0.0};
};

} // namespace fastex_explorer

#endif
