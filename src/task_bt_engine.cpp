#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
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
    std::string bt_file;
    ros::param::get("mission_file", bt_file);
    std::cout << "Using mission file: " << bt_file << std::endl;

    double frequency;
    ros::param::get("tick_frequency", frequency);
    std::chrono::microseconds sleep_us = std::chrono::microseconds((unsigned int)(1000000/frequency));

    BT::Tree tree;
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

    // this tree may be created multiple times (upon change of mission), but reuse the same blackboard pointer.
    tree = factory.createTreeFromFile(bt_file, blackboard);
    BT::PublisherZMQ publisher_zmq(tree);

    // tick tree root periodically until terminal state
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    while (status == BT::NodeStatus::RUNNING) {
        tree.sleep( sleep_us );
        ros::spinOnce();
        status = tree.tickRoot();
    }

    // Output final results
    std::string status_str;
    if (status == BT::NodeStatus::SUCCESS) {
        status_str = "SUCCESS";
    } else {
        status_str = "FAILURE";
    }
    ROS_INFO("Exit with status %s.", status_str.c_str());
    return 0;
}
