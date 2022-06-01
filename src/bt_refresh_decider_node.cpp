#include "refresh_ros/bt_refresh_decider_node.hpp"

namespace BT
{
    ReFRESH_Decider::ReFRESH_Decider(const std::string& name):
        ControlNode::ControlNode(name, {})
    {
        setRegistrationID("ReFRESH_Module");
    }

    void ReFRESH_Decider::halt()
    {
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFRESH_Decider::tick()
    {
        return NodeStatus::RUNNING;
    }
}
