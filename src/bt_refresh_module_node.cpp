#include "refresh_ros/bt_refresh_module_node.hpp"

namespace BT
{
    void ReFRESH_Module::halt()
    {
        // Stop EX
        ControlNode::haltChild(0);
        // Stop EV
        if (childrenCount()>1)
            ControlNode::haltChild(1);
        // Stop ES
        if (childrenCount()>2)
            ControlNode::haltChild(2);
        //initialEV_ = false;
        asyncEV_ = false;
        prestartES_ = false;
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFRESH_Module::tick()
    {
        // First, run ES to determine if the EX is feasible
        if (status() == NodeStatus::IDLE)
        {
            if (!prestartES_)
            {
                BT::NodeStatus ESstatus = estimate_();
                if (ESstatus == NodeStatus::FAILURE)
                {
                    return NodeStatus::FAILURE;
                }
            }
            // ES -> SUCCESS: EX is feasible as an optimistic estimate.
            setStatus(NodeStatus::RUNNING);

            // Second, run EV to determine if the goal of EX is already reached
            // the unsuccessful result of the first evaluator tick is not important.
            BT::NodeStatus EVstatus = evaluate_();
            // Goal is already reached, no need to run EX.
            if (EVstatus == NodeStatus::SUCCESS)
            {
                return NodeStatus::SUCCESS;
            }
            // If EV returned running, it is an async node, need to tick every period
            asyncEV_ = (EVstatus == NodeStatus::RUNNING);
            //return NodeStatus::RUNNING;
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
                    if (childrenCount()>1)
                        ControlNode::haltChild(1);
                    return NodeStatus::FAILURE;
                }
                // evaluate_() == SUCCESS (successfully completed, we need to check consistency just to be safe)
                if (pCost_ >= 1.0)
                {
                    return NodeStatus::FAILURE;
                }
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
                    BT::NodeStatus evStatus = evaluate_();
                    if (evStatus == NodeStatus::SUCCESS)
                    {
                        ControlNode::haltChild(0);
                        return NodeStatus::SUCCESS;
                    }
                    if (evStatus == NodeStatus::FAILURE)
                    {
                        ControlNode::haltChild(0);
                        return NodeStatus::FAILURE;
                    }
                    // in the process. Need to monitor performance and resource.
                    if (rCost_ >= 1.0 || pCost_ >= 1.0)
                    {
                        ControlNode::haltChild(0);
                        ControlNode::haltChild(1);
                        return NodeStatus::FAILURE;
                    }
                }
                return NodeStatus::RUNNING;
                break;

            case NodeStatus::FAILURE:
                if (childrenCount()>1)
                    ControlNode::haltChild(1);
                return NodeStatus::FAILURE;
                break;

            default:
                throw LogicError("A child node must never return IDLE");
                break;
        }
    }
}
