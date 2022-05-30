#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "behaviortree_ros/bt_action_node.hpp"
#include "behaviortree_ros/bt_service_node.hpp"
#include "refresh_ros/HighLevelRequestAction.h"

namespace BT
{
    class ReFRESH_ROS_EX_action :  public RosActionNode<refresh_ros::HighLevelRequestAction>
    {
        public:

            ReFRESH_ROS_EX_action(
                ros::NodeHandle& handle,
                const std::string& node_name,
                const NodeConfiguration & conf) :
                RosActionNode<refresh_ros::HighLevelRequestAction>(handle, node_name, conf)
            {}

            static PortsList providedPorts()
            {
                return {
                    InputPort<std::string>("action_request"),
                    InputPort<std::string>("arguments")
                };
            }

            bool sendGoal(GoalType& goal) override;

    };

    class ReFRESH_ROS_EV_node :  public BT::StatefulActionNode
    {
        public:

            ReFRESH_ROS_EV_node(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
                BT::StatefulActionNode(name, conf), node_(nh) { }

            ReFRESH_ROS_EV_node() = delete;

            virtual ~ReFRESH_ROS_EV_node() = default;

            

            static PortsList providedPorts()
            {
                return {
                    InputPort<std::string>("action_request"),
                    OutputPort<std::string>("arguments")
                };
            }

            bool checkGoal(GoalType& goal) override;

    };
}

#endif