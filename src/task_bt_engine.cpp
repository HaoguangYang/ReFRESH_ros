#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "refresh_ros/bt_refresh_ros_action_node.hpp"
#include "refresh_ros/bt_refresh_module_node.hpp"
#include "refresh_ros/bt_refresh_control_node.hpp"
#include "refresh_ros/ModuleControl.h"
#include "refresh_ros/ReFRESHrequest.h"
#include "refresh_ros/ReFRESHtelemetry.h"
#include "refresh_ros/TaskBehaviortreeEngineConfig.h"
#include "std_msgs/Time.h"

class TaskBehaviortreeEngine
{
    public:
    
    TaskBehaviortreeEngine(ros::NodeHandle& handle)
    {
        // Register nodes
        BT::RegisterRosAction<BT::ReFRESH_ROS_EX_node>(factory_, "ReFRESH_ROS_EX", handle);
        BT::RegisterActionEvaluator<BT::ReFRESH_ROS_EV_node>(factory_, "ReFRESH_ROS_EV");
        BT::RegisterRosService<BT::ReFrESH_ROS_ES_node>(factory_, "ReFRESH_ROS_ES", handle);
        factory_.registerNodeType<BT::ReFRESH_Module>("ReFRESH_Module");
        factory_.registerNodeType<BT::ReFRESH_Decider>("ReFRESH_Decider");
        factory_.registerNodeType<BT::ReFRESH_Reactor>("ReFRESH_Reactor");

        // Behaviortree utilities
        blackboard_ = BT::Blackboard::create();
        status_ = BT::NodeStatus::RUNNING;
        terminalStateNotified_ = false;
        guiTracker_ = new BT::PublisherZMQ(tree_);
        bt_file_ = "";

        // ROS utilities
        nh_ = &handle;
        controlServer_ = handle.advertiseService("ReFRESH/"+ros::this_node::getName()+"/module_control",
                                                &TaskBehaviortreeEngine::controlCb, this);
        dynamic_reconfigure::Server<refresh_ros::TaskBehaviortreeEngineConfig>::CallbackType recfgCb_ = 
            boost::bind(&TaskBehaviortreeEngine::reconfigCb, this, ::_1, ::_2);
        recfgServer_.setCallback(recfgCb_);
    }

    TaskBehaviortreeEngine() = delete;

    virtual ~TaskBehaviortreeEngine()
    {
        delete(guiTracker_);
    }

    inline void halt()
    {
        tree_.rootNode() -> halt();
        status_ = BT::NodeStatus::IDLE;
    }

    bool controlCb(refresh_ros::ModuleControl::Request &req,
                    refresh_ros::ModuleControl::Response &res)
    {
        if (req.request.stamp.toNSec() < lastControlStamp_.toNSec() ||
            req.request.module != ros::this_node::getName())
        {
            // do nothing
            res.status.stamp = ros::Time::now();
            res.status.module = ros::this_node::getName();
            res.status.status = statusMap_[tree_.rootNode()->status()];
            return true;
        }
        lastControlStamp_ = req.request.stamp;
        switch (req.request.request)
        {
            case refresh_ros::ReFRESHrequest::SPAWN:
            case refresh_ros::ReFRESHrequest::ON:
                status_ = BT::NodeStatus::RUNNING;
                break;
            
            case refresh_ros::ReFRESHrequest::WAKEUP:
                status_ = tree_.tickRoot();
                break;

            case refresh_ros::ReFRESHrequest::OFF:
            case refresh_ros::ReFRESHrequest::TERM:
            case refresh_ros::ReFRESHrequest::KILL:
                halt();
                break;

            case refresh_ros::ReFRESHrequest::CLEAR:
                status_ = BT::NodeStatus::IDLE;
                break;

            case refresh_ros::ReFRESHrequest::REINIT:
                std::cout << "Reinitializing mission file: " << bt_file_ << std::endl;
                halt();
                // if invalid tree / file, use an empty tree.
                try {
                    tree_ = factory_.createTreeFromFile(bt_file_, blackboard_);
                } catch (...) {
                    std::cout << "Error occured creating tree from file: " << bt_file_ << "! Using an EMPTY TREE instead." << std::endl;
                    tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
                }
                status_ = BT::NodeStatus::RUNNING;
                delete(guiTracker_);
                guiTracker_ = new BT::PublisherZMQ(tree_);
                break;

            default:
                // do nothing
                break;
        }
        res.status.stamp = ros::Time::now();
        res.status.module = ros::this_node::getName();
        res.status.status = statusMap_[tree_.rootNode()->status()];
        return true;
    }

    void reconfigCb(refresh_ros::TaskBehaviortreeEngineConfig &config,
                    uint32_t level)
    {
        if (level & 0x1)
        {
            // needs to rebuild the behavior tree
            bt_file_ = config.mission_file;
            std::cout << "Using mission file: " << bt_file_ << std::endl;
            halt();
            // if invalid tree / file, use an empty tree.
            try {
                tree_ = factory_.createTreeFromFile(bt_file_, blackboard_);
            } catch (...) {
                std::cout << "Error occured creating tree from file: " << bt_file_ << "! Using an EMPTY TREE instead." << std::endl;
                tree_ = factory_.createTreeFromText(empty_tree_xml_, blackboard_);
            }
            status_ = BT::NodeStatus::RUNNING;
            delete(guiTracker_);
            guiTracker_ = new BT::PublisherZMQ(tree_);
        }
        sleep_us_ = std::chrono::microseconds((unsigned int)(1000000/config.tick_frequency));
        return;
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
        halt();
    }

    protected:

    bool terminalStateNotified_;

    ros::NodeHandle* nh_;

    BT::PublisherZMQ* guiTracker_;

    BT::BehaviorTreeFactory factory_;

    BT::Tree tree_;

    BT::NodeStatus status_;

    BT::Blackboard::Ptr blackboard_;

    std::string bt_file_;

    std::chrono::microseconds sleep_us_;

    ros::ServiceServer controlServer_;

    std_msgs::Time::_data_type lastControlStamp_;

    dynamic_reconfigure::Server<refresh_ros::TaskBehaviortreeEngineConfig> recfgServer_;

    std::unordered_map<BT::NodeStatus, refresh_ros::ReFRESHtelemetry::_status_type> statusMap_ = {
        {BT::NodeStatus::IDLE, refresh_ros::ReFRESHtelemetry::READY},
        {BT::NodeStatus::RUNNING, refresh_ros::ReFRESHtelemetry::RUNNING},
        {BT::NodeStatus::SUCCESS, refresh_ros::ReFRESHtelemetry::OFF},
        {BT::NodeStatus::FAILURE, refresh_ros::ReFRESHtelemetry::ERROR}
    };

    const std::string empty_tree_xml_ = R"(
        <root main_tree_to_execute = "MainTree">
            <BehaviorTree ID="MainTree"/>
        </root>
        )";
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "high_level_task_runner");
    ros::NodeHandle nh;

    TaskBehaviortreeEngine bTreeLevel(nh);
    bTreeLevel.spin();

    /*
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
    */

    return 0;
}
