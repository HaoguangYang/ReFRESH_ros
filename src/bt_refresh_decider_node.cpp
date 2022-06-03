#include "refresh_ros/bt_refresh_decider_node.hpp"

namespace BT
{
    ReFRESH_Decider::ReFRESH_Decider(const std::string& name):
        ControlNode::ControlNode(name, {}), indActive_(-1)
    {
        setRegistrationID("ReFRESH_Module");
    }

    void ReFRESH_Decider::halt()
    {
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFRESH_Decider::tick()
    {
        if (status() == NodeStatus::IDLE)
        {
            // no module is running yet. tick estimator of each module by sending a tick signal to each idle module.
            moduleCost_.clear();
            size_t ind;
            for (ind = 0; ind < childrenCount(); ind ++)
            {
                BT::NodeStatus status = children_nodes_[ind]->executeTick();
                if (dynamic_cast<BT::ReFRESH_Module*>(children_nodes_[ind]) == nullptr)
                {
                    // not a refresh module. if node status is success then make its cost 1-eps (least considered)
                }
                //moduleCost_.push_back(...)
            }
            // sort moduleCost_
            // take the module with smallest cost, and note its index as indActive_
            // halt all other modules.
            return NodeStatus::RUNNING;
        }
        // one module is already running. tick until its status to turn SUCCESS or FAILURE, and check EV status.
        return NodeStatus::RUNNING;
        // update its EV reading after each tick.
        // Upon terminal state, check EV cost reading.
        // If EV cost <1 (SUCCESS), return SUCCESS.
        // If EV cost >=1 (FAILURE), trigger reconfig.
        // tick inactive modules once, update and sort moduleCost_
        // setActive and tick the min-cost and not last-run module
        // halt all other modules
    }
}
