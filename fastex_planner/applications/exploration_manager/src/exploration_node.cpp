#include <exploration_manager/fast_exploration_fsm.h>
#include <ros/ros.h>

#include <plan_manage/backward.hpp>
namespace backward
{
backward::SignalHandling sh;
}

using namespace fastex_explorer;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_node");
    ros::NodeHandle nh("~");

    FastExplorationFSM expl_fsm;
    expl_fsm.init(nh);

    ros::spin();

    return 0;
}
