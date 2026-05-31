/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-10-09 16:58:52
 * @LastEditTime: 2026-05-30 21:58:18
 * @Description:
 */

#include "lkh_solver/lkh_solver.h"

#include <filesystem>
#include <stdexcept>

#include <ros/package.h>
#include <ros/ros.h>

namespace lkh_solver
{
LKHSlover::LKHSlover()
{
    lkh_executable_ = ros::package::getPath("lkh_solver") + "/bin/LKH";
    if (!std::filesystem::exists(lkh_executable_))
    {
        std::runtime_error("LKH executable not found.");
    }
    else
    {
        std::filesystem::permissions(lkh_executable_,
                                     std::filesystem::perms::owner_exec |
                                         std::filesystem::perms::group_exec |
                                         std::filesystem::perms::others_exec,
                                     std::filesystem::perm_options::add);
    }
}

LKHSlover::~LKHSlover() {}

bool LKHSlover::solveProblem(const std::string& par_file)
{
    std::string command = lkh_executable_ + " " + par_file;
    int ret = system(command.c_str());

    if (ret != 0)
    {
        // ROS_ERROR("Failed to call LKH solver.");
        std::cerr << "Failed to call LKH solver." << std::endl;
        return false;
    }

    // ROS_INFO("LKH server finish");
    // std::cout << "LKH server finish" << std::endl;
    return true;
}
} // namespace lkh_solver
