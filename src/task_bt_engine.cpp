#include <ros/ros.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "refresh_ros/bt_refresh_ros_action_node.hpp"
#include "refresh_ros/bt_refresh_module_node.hpp"
#include "refresh_ros/bt_refresh_control_node.hpp"
#include "refresh_ros/ModuleControl.h"

class TaskBehaviortreeEngine
{
    public:
    
    TaskBehaviortreeEngine(ros::NodeHandle& handle)
    {
        nh_ = &handle;
        // Register nodes
        BT::RegisterRosAction<BT::ReFRESH_ROS_EX_node>(factory_, "ReFRESH_ROS_EX", handle);
        BT::RegisterActionEvaluator<BT::ReFRESH_ROS_EV_node>(factory_, "ReFRESH_ROS_EV");
        BT::RegisterRosService<BT::ReFrESH_ROS_ES_node>(factory_, "ReFRESH_ROS_ES", handle);
        factory_.registerNodeType<BT::ReFRESH_Module>("ReFRESH_Module");
        factory_.registerNodeType<BT::ReFRESH_Decider>("ReFRESH_Decider");
        factory_.registerNodeType<BT::ReFRESH_Reactor>("ReFRESH_Reactor");
        blackboard_ = BT::Blackboard::create();
        status_ = BT::NodeStatus::RUNNING;
        terminalStateNotified_ = false;
    }

    TaskBehaviortreeEngine() = delete;

    virtual ~TaskBehaviortreeEngine()
    {
        delete(guiTracker_);
    }

    void halt()
    {
        tree_.rootNode() -> halt();
    }

    void assemble()
    {
        // TODO: make them dynamically reconfigurable
        std::string bt_file;
        ros::param::get("mission_file", bt_file);
        std::cout << "Using mission file: " << bt_file << std::endl;

        double frequency;
        ros::param::get("tick_frequency", frequency);
        sleep_us_ = std::chrono::microseconds((unsigned int)(1000000/frequency));

        tree_ = factory_.createTreeFromFile(bt_file, blackboard_);
        delete(guiTracker_);
        guiTracker_ = new BT::PublisherZMQ(tree_);
    }

    void spin()
    {
        // the behavior tree engine should not stop until external management node exits.
        while (ros::ok()) {
            tree_.sleep( sleep_us_ );
            ros::spinOnce();
            switch (status_)
            {
                case BT::NodeStatus::SUCCESS:
                    if (!terminalStateNotified_)
                    {
                        ROS_INFO("Task completed with status SUCCESS.");
                        terminalStateNotified_ = true;
                    }
                    break;
                
                case BT::NodeStatus::FAILURE:
                    if (!terminalStateNotified_)
                    {
                        ROS_INFO("Task completed with status FAILURE.");
                        terminalStateNotified_ = true;
                    }
                    break;
                
                case BT::NodeStatus::RUNNING:
                    status_ = tree_.tickRoot();
                    break;

                // IDLE
                default:
                    break;
            }
        }
    }

    protected:

    bool terminalStateNotified_;

    ros::NodeHandle* nh_;

    BT::PublisherZMQ* guiTracker_;

    BT::BehaviorTreeFactory factory_;

    BT::Tree tree_;

    BT::NodeStatus status_;

    BT::Blackboard::Ptr blackboard_;

    std::chrono::microseconds sleep_us_;

    ros::Subscriber haltSub_;
};

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
    factory.registerNodeType<BT::ReFRESH_Reactor>("ReFRESH_Reactor");

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
