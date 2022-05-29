#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "behaviortree_ros/bt_action_node.hpp"
#include "behaviortree_ros/bt_service_node.hpp"

namespace BT
{
    class ReFrESH_ROS_Action : public RosActionNode
    {
        public:

            ReFrESH_ROS_Action(const srd::string& name);

            virtual ~ReFrESH_ROS_Action() override = default;

            virtual void halt() override;

            static BT::PortsList providedPorts();

        private:

            virtual BT::NodeStatus tick() override;
    };
}

#endif