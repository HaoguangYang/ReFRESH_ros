#include "ReFrESH_ros/bt_refresh_module_node.hpp"

namespace BT
{
    ReFrESH_Module::ReFrESH_Module(const std::string& name):
        ControlNode::ControlNode(name, {}), asyncEV_(false), initialEV_(false)
    {
        setRegistrationID("ReFrESH_Module");
    }

    void ReFrESH_Module::halt()
    {
        // Stop EX
        ControlNode::haltChild(0);
        // Stop EV
        ControlNode::haltChild(1);
        // Stop ES
        ControlNode::haltChild(2);
        initialEV_ = false;
        asyncEV_ = false;
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFrESH_Module::tick()
    {
        // First, run ES to determine if the EX is feasible
        if (status() == NodeStatus::IDLE)
        {
            BT::NodeStatus ESstatus = estimate_();
            if (ESstatus != NodeStatus::SUCCESS)
            {
                if (ESstatus == NodeStatus::IDLE)
                    throw LogicError("A child node must never return IDLE");
                setStatus(NodeStatus::FAILURE);
                if (children_nodes_.size()>=3)
                    ControlNode::haltChild(2);
                return NodeStatus::FAILURE;
            }
            // ES -> SUCCESS: EX is feasible
            setStatus(NodeStatus::RUNNING);
            initialEV_ = true;
            return NodeStatus::RUNNING;
        }

        // Second, run EV to determine if the goal of EX is already reached
        if (initialEV_)
        {
            BT::NodeStatus EVstatus = evaluate_();
            // Goal is already reached, no need to run EX.
            if (EVstatus == NodeStatus::SUCCESS)
            {
                setStatus(NodeStatus::SUCCESS);
                return NodeStatus::SUCCESS;
            }
            if (EVstatus == NodeStatus::IDLE)
                throw LogicError("A child node must never return IDLE");
            // If EV returned running, it is an async node, need to tick every period
            asyncEV_ = (EVstatus == NodeStatus::RUNNING);
            initialEV_ = false;
            return NodeStatus::RUNNING;
        }

        // Third, run EX.
        // Only when EX and EV both returned SUCCESS will the active module return SUCCESS.
        // Otherwise, it returns FAILURE.
        BT::NodeStatus EXstatus = children_nodes_[0]->executeTick();
        switch (EXstatus)
        {
            case NodeStatus::SUCCESS:
                // EX terminal state and reported successful.
                // Is it really a satisfactory result? If not, override with failure state
                if (evaluate_() != NodeStatus::SUCCESS)
                {
                    setStatus(NodeStatus::FAILURE);
                    if (children_nodes_.size()>=2)
                        ControlNode::haltChild(1);
                    return NodeStatus::FAILURE;
                }
                setStatus(NodeStatus::SUCCESS);
                return NodeStatus::SUCCESS;
                break;
            
            case NodeStatus::RUNNING:
                /**
                 * @brief If EV is asynchronous, tick it every cycle as we do with EX.
                 * Limitation: EV should reach a SUCCESS state within one tick after
                 * EX completes, as previously shown. The asyncEV_ implementation
                 * only guarantees #ticks_EV = #ticks_EX+1, including the initial tick.
                 */
                if (asyncEV_)
                {
                    // EV signals SUCCESS ahead of EX. exit here.
                    if (evaluate_() == NodeStatus::SUCCESS)
                    {
                        setStatus(NodeStatus::SUCCESS);
                        ControlNode::haltChild(0);
                        return NodeStatus::SUCCESS;
                    }
                }
                return NodeStatus::RUNNING;
                break;

            case NodeStatus::FAILURE:
                setStatus(NodeStatus::FAILURE);
                if (children_nodes_.size()>=2)
                    ControlNode::haltChild(1);
                return NodeStatus::FAILURE;
                break;

            default:
                throw LogicError("A child node must never return IDLE");
                break;
        }
    }
}
