#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "high_level_task_runner");
    ros::NodeHandle nh;

    BehaviorTreeFactory factory;

    // Register nodes

    // load behavior tree xml from argc or argv or ROS param

    // tick tree root periodically

    return 0;
}
