/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:34
 * @LastEditTime: 2026-05-30 22:31:57
 * @Description:
 */

#ifndef _LOCAL_VIEWPOINT_REFINER_H_
#define _LOCAL_VIEWPOINT_REFINER_H_

#include <memory>
#include <optional>
#include <vector>

#include "common_utils/eigen_utils.h"
#include "map_process/searcher/path_searcher.h"

namespace fastex_explorer
{
class LocalViewpointRefiner
{
  public:
    using UniquePtr = std::unique_ptr<LocalViewpointRefiner>;

    LocalViewpointRefiner(const map_process::PathSearcher::SharedPtr& path_searcher,
                          const double straight_max_dist);
    ~LocalViewpointRefiner() = default;

    bool refineTour(const eigen_utils::Vec3d& cur_pos, const eigen_utils::Vec3d& cur_vel,
                    const eigen_utils::Vec3d& cur_yaw,
                    const std::vector<eigen_utils::Vec_Vec3d>& n_points,
                    const std::vector<std::vector<std::optional<double>>>& n_yaws,
                    eigen_utils::Vec_Vec3d& refined_pts,
                    std::vector<std::optional<double>>& refined_yaws,
                    eigen_utils::Vec_Vec3d& local_refined_tour) const;

  private:
    map_process::PathSearcher::SharedPtr path_searcher_;
    double straight_max_dist_;
};
} // namespace fastex_explorer

#endif // _LOCAL_VIEWPOINT_REFINER_H_
