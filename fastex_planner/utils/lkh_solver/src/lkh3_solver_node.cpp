/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-10-03 15:48:36
 * @LastEditTime: 2024-10-09 17:13:33
 * @Description:
 */

#include <string>
#include <memory>
#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>

#include "lkh_solver/SolveProblem.h"
#include "lkh_solver/lkh_solver.h"

class LKHSloverServer
{
private:
  int drone_id_;
  lkh_solver::LKHSlover::UniquePtr lkh_solver_;
  ros::ServiceServer lkh_server_;

  bool lkhCallback(lkh_solver::SolveProblem::Request &req,
                   lkh_solver::SolveProblem::Response &res)
  {
    return lkh_solver_->solveProblem(req.par_file);
  }

public:
  using SharedPtr = std::shared_ptr<LKHSloverServer>;
  using UniquePtr = std::unique_ptr<LKHSloverServer>;

  LKHSloverServer() {}

  LKHSloverServer(ros::NodeHandle &nh)
  {
    bool is_params_load = true;
    is_params_load &= nh.param("drone_id", drone_id_, 1);
    if (!is_params_load)
      ROS_ERROR("Failed to load LKH server params.");

    lkh_solver_ = std::make_unique<lkh_solver::LKHSlover>();

    lkh_server_ = nh.advertiseService(
        "/solve_lkh_" + std::to_string(drone_id_), &LKHSloverServer::lkhCallback, this);

    ROS_WARN("LKH server %d is ready.", drone_id_);
  }

  ~LKHSloverServer() {}
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lkh_solver");
  ros::NodeHandle nh("~");

  LKHSloverServer::UniquePtr lkh_server = std::make_unique<LKHSloverServer>(nh);

  ros::spin();

  return 0;
}
