/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-04-30
 * @Description:
 */

#include <algorithm>
#include <cmath>
#include <numeric>

#include "map_process/frontier/frontier_manager.h"

namespace map_process
{
/**
 * @brief Compute key viewpoints for newly generated local frontier clusters.
 *
 * The pipeline performs viewpoint generation, coarse non-maximum suppression,
 * visibility scoring, score thresholding, distance-based non-maximum
 * suppression, and finally a centroid fallback when no valid viewpoint
 * survives.
 */
void FrontierManager::computeKeyViewpointsForClustersWithVisibleScore2()
{
    std::unique_lock<std::shared_mutex> local_lock(local_new_frontier_clusters_info_mutex_);

    auto isCloseToVertical = [](const eigen_utils::Vec3d& normal,
                                double threshold_degrees) -> bool {
        double threshold_radians = threshold_degrees * M_PI / 180.0;
        double pitch = std::atan2(normal.z(), std::hypot(normal.x(), normal.y()));
        return std::abs(std::abs(pitch) - M_PI / 2) < threshold_radians;
    };

    for (auto& cluster : local_new_frontier_clusters_info_ptr_)
    {
        if (isCloseToVertical(cluster->normal_, 30.0))
        {
            cluster->is_dormant_ = true;
            continue;
        }

        if (cluster->is_dormant_)
            continue;

        // Phase 1 — Generate raw viewpoint candidates
        generateViewpointsForClusters(*cluster);

        if (cluster->viewpoints_.empty())
        {
            cluster->is_dormant_ = true;
            continue;
        }

        // Coarse non-maximum suppression
        std::vector<size_t> filtered_candidates_0(cluster->viewpoints_.size());
        std::iota(filtered_candidates_0.begin(), filtered_candidates_0.end(), 0);
        std::vector<size_t> filtered_candidates_0_1 =
            nonMaximumSuppression(cluster->viewpoints_, filtered_candidates_0, 1.5, INT_MAX);
        std::vector<FrontierViewpoint> key_viewpoints_0;
        for (size_t idx : filtered_candidates_0_1)
            key_viewpoints_0.push_back(cluster->viewpoints_[idx]);
        cluster->viewpoints_.swap(key_viewpoints_0);

        std::vector<FrontierUnit> filtered_frontiers;
        filtered_frontiers.reserve(cluster->frontiers_ds_.size());
        for (size_t i = 0; i < cluster->frontiers_ds_.size(); ++i)
            filtered_frontiers.emplace_back(true, cluster->frontiers_ds_[i],
                                            cluster->normals_ds_[i]);

        // Phase 2 — Visibility scoring and filtering
        std::vector<bool> visible_frontiers;
        int max_vis_num = 0;
        int min_vis_num = frontier_params_.min_visible_voxel_num_ *
                          double(cluster->frontiers_ds_.size()) / cluster->frontiers_.size();
        std::vector<FrontierViewpoint> candidate_viewpoints;

        for (FrontierViewpoint& vp : cluster->viewpoints_)
        {
            int vis_num = computeVisibleScore(vp, filtered_frontiers, visible_frontiers);
            if (vis_num >= min_vis_num)
            {
                candidate_viewpoints.push_back(vp);
                max_vis_num = std::max(max_vis_num, vis_num);
            }
        }

        if (candidate_viewpoints.empty())
        {
            cluster->viewpoints_.clear();
            cluster->is_dormant_ = true;
            continue;
        }
        else
        {
            cluster->viewpoints_.swap(candidate_viewpoints);
        }

        // Phase 3 — Adaptive threshold and final non-maximum suppression
        double threshold = computeAdaptiveThreshold(cluster->viewpoints_, -0.5);

        std::vector<size_t> filtered_candidates_1 =
            thresholdFilterViewpoints(cluster->viewpoints_, threshold);

        double avg_dist = computeAverageDistance(cluster->viewpoints_);
        double dist_threshold = std::min(std::max(1.5 * avg_dist, 2.0), 3.0);

        std::sort(
            filtered_candidates_1.begin(), filtered_candidates_1.end(), [&](size_t i, size_t j) {
                return cluster->viewpoints_[i].visib_score_ > cluster->viewpoints_[j].visib_score_;
            });

        std::vector<size_t> filtered_candidates_2 =
            nonMaximumSuppression(cluster->viewpoints_, filtered_candidates_1, dist_threshold,
                                  frontier_params_.refined_viewpoint_num_);

        std::vector<FrontierViewpoint> key_viewpoints;
        for (size_t idx : filtered_candidates_2)
            key_viewpoints.push_back(cluster->viewpoints_[idx]);

        cluster->viewpoints_.swap(key_viewpoints);

        if (cluster->viewpoints_.empty())
        {
            if (tryUseClusterCentroidAsViewpoint(*cluster))
                continue;

            cluster->is_dormant_ = true;
        }
    }
}

/**
 * @brief Generate raw viewpoint candidates for a frontier cluster.
 *
 * The function first tries viewpoint generation from downsampled frontier
 * points and normals. If no valid candidate is found, it falls back to
 * cylindrical sampling around the cluster centroid.
 *
 * @param cluster Frontier cluster to be populated with viewpoint candidates.
 */
void FrontierManager::generateViewpointsForClusters(FrontierClusterInfo& cluster)
{
    eigen_utils::Vec_Vec3d sampling_points, transformed_points, sampling_normals;

    const eigen_utils::Vec_Vec3d& frontiers = cluster.frontiers_ds_;
    const eigen_utils::Vec_Vec3d& normals = cluster.normals_ds_;
    generateViewpointsWithDownsampledFrontiersAndNormals(frontiers, normals, 1.0, 5.0, 3,
                                                         transformed_points, sampling_normals);

    eigen_utils::Vec3i index;
    for (size_t i = 0; i < transformed_points.size(); ++i)
    {
        sdf_map_->posToIndex(transformed_points[i], index);
        if (sdf_map_->isInBox(index) && isSafe(index, frontier_params_.min_candidate_clearance_))
        {
            cluster.viewpoints_.emplace_back(transformed_points[i], sampling_normals[i], 0.0);
        }
    }

    if (cluster.viewpoints_.empty())
    {
        generateCylinderSamplingPoints(
            sampling_points, frontier_params_.candidate_rmin_, frontier_params_.candidate_rmax_,
            frontier_params_.candidate_hmin_, frontier_params_.candidate_hmax_,
            frontier_params_.candidate_hstep_, frontier_params_.candidate_rstep_,
            frontier_params_.candidate_theta_step_, frontier_params_.candidate_kh_,
            frontier_params_.candidate_kr_, frontier_params_.candidate_ktheta_);

        eigen_utils::Vec3d translation = cluster.centroid_;
        eigen_utils::Vec3d origin_dir = eigen_utils::Vec3d(0, 0, 1);
        eigen_utils::Vec3d target_dir = eigen_utils::Vec3d(0, 0, 1);

        transformPoints(sampling_points, transformed_points, translation, origin_dir, target_dir);

        for (const auto& pt : transformed_points)
        {
            sdf_map_->posToIndex(pt, index);
            if (sdf_map_->isInBox(index) &&
                isSafe(index, frontier_params_.min_candidate_clearance_))
            {
                cluster.viewpoints_.emplace_back(pt, cluster.normal_, 0);
            }
        }
    }
}

/**
 * @brief Generate cylindrical sampling points in the local canonical frame.
 *
 * @param points Output sampled points.
 * @param r_min Minimum sampling radius.
 * @param r_max Maximum sampling radius.
 * @param z_min Minimum sampling height.
 * @param z_max Maximum sampling height.
 * @param step_z0 Base step size along the z axis.
 * @param step_r0 Base radial step size.
 * @param angle_step0 Base angular step size.
 * @param k_z Linear growth factor for z-axis step size.
 * @param k_r Linear growth factor for radial step size.
 * @param k_theta Linear growth factor for angular step size.
 */
void FrontierManager::generateCylinderSamplingPoints(eigen_utils::Vec_Vec3d& points, double r_min,
                                                     double r_max, double z_min, double z_max,
                                                     double step_z0, double step_r0,
                                                     double angle_step0, double k_z, double k_r,
                                                     double k_theta)
{
    auto angleStep = [&](double z, double angle_step0, double k_theta) -> double {
        return angle_step0 + k_theta * std::max(z - z_min, 0.0);
    };

    auto stepR = [&](double z, double step_r0, double k_r) -> double {
        return step_r0 + k_r * std::max(z - z_min, 0.0);
    };

    auto stepZ = [&](double z, double step_z0, double k_z) -> double {
        return step_z0 + k_z * std::max(z - z_min, 0.0);
    };

    for (double z = z_min; z <= z_max; z += stepZ(z, step_z0, k_z))
    {
        double current_step_r = stepR(z, step_r0, k_r);
        double current_angle_step = angleStep(z, angle_step0, k_theta);

        for (double r = r_min; r <= r_max; r += current_step_r)
        {
            for (double theta = 0.0; theta < 2.0 * M_PI; theta += current_angle_step)
            {
                double x = r * std::cos(theta);
                double y = r * std::sin(theta);
                points.emplace_back(x, y, z);
            }
        }
    }
}

/**
 * @brief Generate viewpoint samples by stepping outward along frontier normals.
 *
 * @param frontiers Downsampled frontier points.
 * @param normals Frontier normals corresponding to `frontiers`.
 * @param min_dist Minimum sampling distance along each normal.
 * @param max_dist Maximum sampling distance along each normal.
 * @param step_num Number of distance segments between `min_dist` and `max_dist`.
 * @param sample_points Output sampled viewpoint positions.
 * @param sample_normals Output normals associated with sampled positions.
 */
void FrontierManager::generateViewpointsWithDownsampledFrontiersAndNormals(
    const eigen_utils::Vec_Vec3d& frontiers, const eigen_utils::Vec_Vec3d& normals,
    const double min_dist, const double max_dist, const int step_num,
    eigen_utils::Vec_Vec3d& sample_points, eigen_utils::Vec_Vec3d& sample_normals)
{
    sample_points.clear();
    double step = (max_dist - min_dist) / step_num;

    for (size_t i = 0; i < frontiers.size(); ++i)
    {
        eigen_utils::Vec3d frontier = frontiers[i];
        eigen_utils::Vec3d normal = normals[i].normalized();

        for (double dist = min_dist; dist <= max_dist + 1e-3; dist += step)
        {
            eigen_utils::Vec3d point = frontier + dist * normal;
            sample_points.push_back(point);
            sample_normals.push_back(normal);
        }
    }
}

/**
 * @brief Apply a rigid transform to a batch of points.
 *
 * The transform rotates the point set from `initial_direction` to
 * `target_direction`, then translates it by `translation`.
 *
 * @param points_raw Input points in the source frame.
 * @param points_transformed Output points after transformation.
 * @param translation Translation applied after rotation.
 * @param initial_direction Source reference direction.
 * @param target_direction Target reference direction.
 */
void FrontierManager::transformPoints(const eigen_utils::Vec_Vec3d& points_raw,
                                      eigen_utils::Vec_Vec3d& points_transformed,
                                      const eigen_utils::Vec3d& translation,
                                      const eigen_utils::Vec3d& initial_direction,
                                      const eigen_utils::Vec3d& target_direction)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> points_matrix(
        points_raw[0].data(), points_raw.size(), 3);

    eigen_utils::Vec3d axis = initial_direction.cross(target_direction);
    double angle = std::acos(initial_direction.dot(target_direction) /
                             (initial_direction.norm() * target_direction.norm()));
    Eigen::AngleAxisd rotation(angle, axis.normalized());

    Eigen::Affine3d transform = Eigen::Translation3d(translation) * rotation;
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> transformed_points =
        (transform * points_matrix.transpose()).transpose();

    points_transformed.clear();
    points_transformed.reserve(transformed_points.rows());
    for (int i = 0; i < transformed_points.rows(); ++i)
        points_transformed.emplace_back(transformed_points(i, 0), transformed_points(i, 1),
                                        transformed_points(i, 2));
}

/**
 * @brief Use the cluster centroid as a fallback viewpoint when possible.
 *
 * The centroid is accepted only if it is safe and a feasible fine path exists
 * from the current robot position.
 *
 * @param cluster Frontier cluster to receive the fallback viewpoint.
 * @return true if the centroid is accepted as a viewpoint.
 * @return false otherwise.
 */
bool FrontierManager::tryUseClusterCentroidAsViewpoint(FrontierClusterInfo& cluster)
{
    if (!isSafe(cluster.centroid_, frontier_params_.min_candidate_clearance_))
        return false;

    eigen_utils::Vec_Vec3d path;
    double dist;
    PATH_SEARCH_RESULT result = path_searcher_->searchFinePath(cluster.centroid_, current_position_,
                                                               path, dist, -1.0, false);

    if (result != PATH_SEARCH_RESULT::SUCCESS)
        return false;

    cluster.viewpoints_.emplace_back(cluster.centroid_, cluster.normal_, 0.0);
    return true;
}

/**
 * @brief Compute the visibility score of a candidate viewpoint.
 *
 * @param vp Candidate viewpoint to be scored. Its `visib_score_` is updated in
 * place.
 * @param frontiers Frontier units to be tested for visibility.
 * @param visible_frontiers Output visibility flags for each frontier unit.
 * @return Number of visible frontier units.
 */
int FrontierManager::computeVisibleScore(FrontierViewpoint& vp,
                                         const std::vector<FrontierUnit>& frontiers,
                                         std::vector<bool>& visible_frontiers)
{
    visible_frontiers.clear();
    visible_frontiers.reserve(frontiers.size());
    vp.visib_score_ = 0.0;

    for (const auto& ft : frontiers)
    {
        if (!isInFov(vp.pos_, ft.pos_))
        {
            visible_frontiers.push_back(false);
            continue;
        }

        raycaster_->input(vp.pos_, ft.pos_);
        eigen_utils::Vec3i idx;
        bool visib = true;
        while (raycaster_->nextId(idx))
        {
            if (!sdf_map_->isFree(idx))
            {
                visib = false;
                break;
            }
        }

        visible_frontiers.push_back(visib);

        if (visib)
        {
            eigen_utils::Vec3d f2v_dir = (vp.pos_ - ft.pos_).normalized();
            vp.visib_score_ += f2v_dir.dot(ft.normal_);
        }
    }

    return std::count(visible_frontiers.begin(), visible_frontiers.end(), true);
}

/**
 * @brief Compute an adaptive threshold from viewpoint visibility scores.
 *
 * @param viewpoints Candidate viewpoints with precomputed visibility scores.
 * @param k Standard-deviation scaling factor.
 * @return Adaptive score threshold.
 */
double FrontierManager::computeAdaptiveThreshold(const std::vector<FrontierViewpoint>& viewpoints,
                                                 const double k)
{
    if (viewpoints.size() < 2)
        return 0.0;

    double mean = 0.0;
    for (const FrontierViewpoint& vp : viewpoints)
        mean += vp.visib_score_;
    mean /= viewpoints.size();

    double accum = 0.0;
    for (const FrontierViewpoint& vp : viewpoints)
        accum += (vp.visib_score_ - mean) * (vp.visib_score_ - mean);
    double stddev = std::sqrt(accum / (viewpoints.size() - 1));

    return mean + k * stddev;
}

/**
 * @brief Filter viewpoints whose visibility score is below a threshold.
 *
 * @param viewpoints Candidate viewpoints.
 * @param threshold Minimum accepted visibility score.
 * @return Indices of viewpoints that pass the threshold.
 */
std::vector<size_t>
FrontierManager::thresholdFilterViewpoints(const std::vector<FrontierViewpoint>& viewpoints,
                                           const double threshold)
{
    std::vector<size_t> filtered_indices;

    for (size_t i = 0; i < viewpoints.size(); ++i)
    {
        if (viewpoints[i].visib_score_ >= threshold)
            filtered_indices.push_back(i);
    }

    return filtered_indices;
}

/**
 * @brief Compute the mean pairwise distance among viewpoint candidates.
 *
 * @param viewpoints Candidate viewpoints.
 * @return Average Euclidean distance over all viewpoint pairs.
 */
double FrontierManager::computeAverageDistance(const std::vector<FrontierViewpoint>& viewpoints)
{
    if (viewpoints.size() < 2)
        return 0.0;

    double total_dist = 0.0;
    int cnt = 0;

    for (size_t i = 0; i < viewpoints.size(); ++i)
    {
        for (size_t j = i + 1; j < viewpoints.size(); ++j)
        {
            total_dist += (viewpoints[i].pos_ - viewpoints[j].pos_).norm();
            ++cnt;
        }
    }

    return total_dist / cnt;
}

/**
 * @brief Apply distance-based non-maximum suppression to candidate viewpoints.
 *
 * The input indices are assumed to already be ordered by descending priority.
 *
 * @param candidate_viewpoints Full set of candidate viewpoints.
 * @param score_filtered_indices Pre-filtered candidate indices.
 * @param distance_hreshold Minimum allowed spacing between kept viewpoints.
 * @param max_viewpoint_num Maximum number of viewpoints to keep.
 * @return Indices of viewpoints that survive suppression.
 */
std::vector<size_t>
FrontierManager::nonMaximumSuppression(const std::vector<FrontierViewpoint>& candidate_viewpoints,
                                       const std::vector<size_t>& score_filtered_indices,
                                       const double distance_hreshold,
                                       const size_t max_viewpoint_num)
{
    std::vector<size_t> nms_indices;
    nms_indices.reserve(score_filtered_indices.size());

    const double squared_distance_threshold = distance_hreshold * distance_hreshold;
    std::vector<bool> is_suppressed(candidate_viewpoints.size(), false);

    size_t idx_i, idx_j;
    double sq_dist;
    for (size_t i = 0; i < score_filtered_indices.size(); ++i)
    {
        idx_i = score_filtered_indices[i];
        if (is_suppressed[idx_i])
            continue;

        nms_indices.push_back(idx_i);

        if (nms_indices.size() >= max_viewpoint_num)
            break;

        for (size_t j = i + 1; j < score_filtered_indices.size(); ++j)
        {
            idx_j = score_filtered_indices[j];
            if (is_suppressed[idx_j])
                continue;

            sq_dist =
                (candidate_viewpoints[idx_i].pos_ - candidate_viewpoints[idx_j].pos_).squaredNorm();
            if (sq_dist < squared_distance_threshold)
                is_suppressed[idx_j] = true;
        }
    }

    return nms_indices;
}

} // namespace map_process
