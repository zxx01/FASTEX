/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-04-14 13:55:57
 * @LastEditTime: 2026-01-31 19:27:00
 * @Description:
 */

#include <ros/ros.h>

#include "data_log_manager/expl_data_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log_manager");
    ros::NodeHandle nh("~");

    data_log_manager::ExplorationDataManager::UniquePtr expl_data_manager =
        std::make_unique<data_log_manager::ExplorationDataManager>();
    expl_data_manager->initialize(nh);

    ros::spin();

    return 0;
}
