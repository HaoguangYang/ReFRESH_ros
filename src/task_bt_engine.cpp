#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "refresh_ros/bt_refresh_ros_action_node.hpp"
#include "refresh_ros/bt_refresh_module_node.hpp"
#include "refresh_ros/bt_refresh_decider_node.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "high_level_task_runner");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;

    // Register nodes
    BT::RegisterRosAction<BT::ReFRESH_ROS_EX_node>(factory, "ReFRESH_ROS_EX", nh);
    BT::RegisterActionEvaluator<BT::ReFRESH_ROS_EV_node>(factory, "ReFRESH_ROS_EV");
    BT::RegisterRosService<BT::ReFrESH_ROS_ES_node>(factory, "ReFRESH_ROS_ES", nh);
    factory.registerNodeType<BT::ReFRESH_Module>("ReFRESH_Module");
    factory.registerNodeType<BT::ReFRESH_Decider>("ReFRESH_Decider");

    // load behavior tree xml from argc or argv or ROS param

    // tick tree root periodically

    return 0;
}
