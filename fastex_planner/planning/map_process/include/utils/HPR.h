/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-06-04 15:49:44
 * @LastEditTime: 2025-10-28 11:16:59
 * @Description:
 */

#ifndef _HPR_HPP_
#define _HPR_HPP_

#include <cmath>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include "common_utils/eigen_utils.h"

namespace utils
{
enum class KernalType : uint8_t
{
    SPHERICAL_MIRROR,
    EXPONENTIAL
};

template <typename PointT> class HPR
{
  private:
    KernalType kernal_type_; // the kernal type: SPHERICAL_MIRROR or EXPONENTIAL

    float radius_;    // radius of the sphere, only used in SPHERICAL_MIRROR
    float gamma_;     // gamma value, only used in EXPONENTIAL
    float gamma_inv_; // 1 / gamma, only used in EXPONENTIAL

    eigen_utils::Vec3f center_point_;
    typename pcl::PointCloud<PointT>::Ptr input_cloud_;

    /**
     * @brief Project a point onto a sphere
     *
     * @param point The point to be projected
     * @return eigen_utils::Vec3f The projected point
     */
    eigen_utils::Vec3f projectPoint(const eigen_utils::Vec3f& point)
    {
        /** SPHERICAL_MIRROR
         * pi = f(pi) = pi +2(R−||pi||) * ( pi / ||pi||)
         *
         * here:
         *  - ||pi|| is the norm
         *  - pi is the projected_point
         *  - we have the radius (described above)
         *
         */
        eigen_utils::Vec3f projected_point = point - center_point_;
        float norm = projected_point.norm();
        if (norm < 1e-6)
            norm = 0.0001f;

        switch (kernal_type_)
        {
        case KernalType::SPHERICAL_MIRROR:
            projected_point = (2 * radius_ - norm) * projected_point / norm;
            break;
        case KernalType::EXPONENTIAL:
            projected_point = std::pow(norm, gamma_) * projected_point / norm;
            break;
        default:
            throw std::invalid_argument("Invalid kernal type.");
        }

        return projected_point;
    }

    /**
     * @brief Inverse project a point from a sphere
     *
     * @param point The point to be inverse projected
     * @return eigen_utils::Vec3f The original point
     */
    eigen_utils::Vec3f inverseProjectPoint(const eigen_utils::Vec3f& point)
    {
        float norm = point.norm();
        eigen_utils::Vec3f original_point;

        switch (kernal_type_)
        {
        case KernalType::SPHERICAL_MIRROR:
            original_point = (2 * radius_ - norm) * point / norm;
            break;
        case KernalType::EXPONENTIAL:
            original_point = std::pow(norm, gamma_inv_) * point / norm;
            break;
        default:
            throw std::invalid_argument("Invalid kernal type.");
        }

        original_point += center_point_;
        return original_point;
    }

    /**
     * @brief Compute the convex hull of a point cloud
     *
     * @param cloud The point cloud to compute the convex hull
     * @return pcl::PointCloud<PointT>::Ptr The convex hull of the point cloud
     */
    typename pcl::PointCloud<PointT>::Ptr
    computeConvexHull(const typename pcl::PointCloud<PointT>::Ptr& cloud)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
        typename pcl::ConvexHull<PointT> chull;
        chull.setDimension(3);
        chull.setInputCloud(cloud);
        chull.reconstruct(*cloud_hull);
        return cloud_hull;
    }

  public:
    HPR() {}
    ~HPR() {}

    /**
     * @brief Set the kernal type
     *
     * @param kernal_type The kernal type
     */
    void setKernalType(const KernalType kernal_type) { this->kernal_type_ = kernal_type; }

    /**
     * @brief Set the radius of the sphere
     *
     * @param radius The radius of the sphere
     */
    void setRadius(const float radius) { this->radius_ = radius; }

    /**
     * @brief Set the gamma value
     *
     * @param gamma The gamma value
     */
    void setGamma(const float gamma)
    {
        this->gamma_ = gamma;
        this->gamma_inv_ = 1 / gamma_;
    }

    /**
     * @brief Set the center point of the sphere
     *
     * @param center_point The center point of the sphere
     */
    void setCenterPoint(const eigen_utils::Vec3f& center_point)
    {
        this->center_point_ = center_point;
    }

    /**
     * @brief Set the input cloud
     *
     * @param input_cloud The input cloud
     */
    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr& input_cloud)
    {
        if (input_cloud == nullptr)
            throw std::invalid_argument("Input cloud must not be null.");

        this->input_cloud_ = input_cloud->makeShared();
    }

    /**
     * @brief Compute the Hidden Point Removal
     *
     * @param visible_cloud The visible cloud after HPR
     */
    void compute(typename pcl::PointCloud<PointT>::Ptr visible_cloud)
    {
        /**
         * Given a set of points P = {pi |1 ≤ i ≤ n} ⊂ ℜD,
         * which is considered a sampling of a continuous surface S, and a viewpoint (camera
         * position) C, our goal is to determine ∀pi ∈ P whether pi is visible from C.
         */

        /**
         * Step 1:
         * Perform spherical projection
         *
         * Given P and C, we associate with P a coordinate system, in which the viewpoint C is
         * placed at the origin. We seek a function that maps a point pi ∈ P along the ray from C to
         * pi and is monotonically decreasing in ||pi ||. (||·|| is a norm.)
         */

        if (input_cloud_ == nullptr || input_cloud_->empty())
            throw std::runtime_error("Input cloud is empty or null.");

        typename pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>);
        new_cloud->resize(input_cloud_->size());

        for (size_t i = 0; i < input_cloud_->size(); ++i)
        {
            PointT new_point = (*input_cloud_)[i];
            eigen_utils::Vec3f currentVector(new_point.x, new_point.y, new_point.z);
            eigen_utils::Vec3f projected_point = projectPoint(currentVector);
            new_point.x = projected_point.x();
            new_point.y = projected_point.y();
            new_point.z = projected_point.z();
            (*new_cloud)[i] = new_point;
        }

        // Step 2: Compute the convex hull of the projected points
        typename pcl::PointCloud<PointT>::Ptr cloud_hull = computeConvexHull(new_cloud);

        // Step 3: Perform inverse spherical projection
        if (visible_cloud == nullptr)
            visible_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        else
            visible_cloud->clear();

        visible_cloud->resize(cloud_hull->size());

        for (size_t i = 0; i < cloud_hull->size(); ++i)
        {
            PointT visible_point = (*cloud_hull)[i];
            eigen_utils::Vec3f projected_point(visible_point.x, visible_point.y, visible_point.z);
            eigen_utils::Vec3f original_point = inverseProjectPoint(projected_point);
            visible_point.x = original_point.x();
            visible_point.y = original_point.y();
            visible_point.z = original_point.z();
            (*visible_cloud)[i] = visible_point;
        }
    }
};

} // namespace utils

#endif // !_HPR_HPP_
