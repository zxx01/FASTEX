/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-02
 * @Description: Centralized parameter structs for the map_process module.
 *     All parameter structs previously scattered across individual headers
 *     are consolidated here for unified configuration management.
 */

#ifndef _MAP_PROCESS_PARAMS_H_
#define _MAP_PROCESS_PARAMS_H_

#include <string>

#include "common_utils/eigen_utils.h"

namespace map_process
{
// ============================================================================
//  Frontier Manager parameters
// ============================================================================

struct FrontierParams
{
    std::string frame_id_{"world"};

    // --- Frontier detection ---
    int min_visible_voxel_num_{10};
    int min_cluster_voxel_num_{20};
    float max_cluster_radius_{5.0f};
    float frontier_filter_ratio_{2.0f};

    // --- Sensor FOV constraints ---
    float upper_fov_angle_{52.0f}; // [deg]
    float lower_fov_angle_{-7.0f}; // [deg]

    // --- Coverage / finish criteria ---
    float min_view_finish_fraction_{0.5f};
    float frontier_clear_threshold_{0.0f};

    // --- Clustering ---
    float frontier_cutoff_min_{0.0f};
    float frontier_cutoff_max_{0.0f};

    // --- Viewpoint candidate generation (cylindrical sampling) ---
    float min_candidate_clearance_{1.0f};
    double candidate_rmin_{0.0};
    double candidate_rmax_{3.0};
    double candidate_hmin_{1.0};
    double candidate_hmax_{3.0};
    double candidate_rstep_{0.5};
    double candidate_hstep_{1.0};
    double candidate_theta_step_{M_PI / 6.0}; // 30°
    double candidate_kr_{0.0};
    double candidate_kh_{0.0};
    double candidate_ktheta_{0.0};

    // --- Viewpoint filtering ---
    int refined_viewpoint_num_{10};
};

// ============================================================================
//  Whole-State RoadMap parameters
// ============================================================================

struct WSRoadMapParams
{
    std::string frame_id_{"world"};
    eigen_utils::Vec3f initial_position_{eigen_utils::Vec3f::Zero()};

    float sample_dist_{1.0f};       // [m] sampling resolution
    float min_interval_{2.0f};      // [m] minimum vertex interval
    float bound_margin_{1.0f};      // [m] margin around bounding box
    float connectable_range_{5.0f}; // [m] max range for new connections
    int connectable_num_{6};        // max neighbours per vertex
};

// ============================================================================
//  Path Searcher parameters
// ============================================================================

struct PathSearcherParams
{
    double vm_{0.0};        // max velocity [m/s]
    double am_{0.0};        // max acceleration [m/s²]
    double yd_{0.0};        // yaw range [rad]
    double ydd_{0.0};       // yaw-rate range [rad/s]
    double alpha_dir_{0.0}; // direction-change weight
    double beta_dir_{0.0};  // direction-continuity weight
};

// ============================================================================
//  Dynamic Expanding Grid parameters
// ============================================================================

struct DynamicExpandingGridParams
{
    std::string frame_id_{"world"};
    double max_straight_dist_{0.0};
    bool is_min_max_set_{false};

    eigen_utils::Vec3d initial_resolution_{eigen_utils::Vec3d::Zero()};
    eigen_utils::Vec3d min_set_{eigen_utils::Vec3d::Zero()};
    eigen_utils::Vec3d max_set_{eigen_utils::Vec3d::Zero()};
};

// ============================================================================
//  History Position Graph parameters
// ============================================================================

struct HistoryPosGraphParams
{
    float min_insert_distance_{4.0f};   // [m] min distance for new history pos
    float min_interval_distance_{1.0f}; // [m] min interval between key pos
    float max_link_distance_{32.0f};    // [m] max edge length
};

// ============================================================================
//  Detection range bounds (used across modules)
// ============================================================================

struct DetectionRangeBounds
{
    eigen_utils::Vec3d min_{eigen_utils::Vec3d::Zero()};
    eigen_utils::Vec3d max_{eigen_utils::Vec3d::Zero()};
};

} // namespace map_process

#endif // !_MAP_PROCESS_PARAMS_H_
