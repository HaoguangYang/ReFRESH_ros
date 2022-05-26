#include "behavior_tree_ros/bt_refresh_action_module_node.hpp"

namespace BT
{
    ReFrESH_ActionModule::ReFrESH_ActionModule(const std::string& name):
        ControlNode::ControlNode(name, {}), asyncEV_(false), initialEV_(false)
    {
        setRegistrationID("ReFrESH_ActionModule");
    }

    void ReFrESH_ActionModule::halt()
    {
        // Stop EX
        ControlNode::haltChild(0);
        // Stop EV
        ControlNode::haltChild(1);
        // Stop ES
        ControlNode::haltChind(2);
        initialEV_ = false;
        asyncEV_ = false;
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFrESH_ActionModule::tick()
    {
        if (status() == NodeStatus::IDLE)
        {
            BT::NodeStatus ESstatus = estimate();
            if (ESstatus != NodeStatus::SUCCESS)
            {
                setStatus(NodeStatus::FAILURE);
                return NodeStatus::FAILURE;
            }
            // ES -> SUCCESS
            setStatus(NodeStatus::RUNNING);
            initialEV_ = true;
            return NodeStatus::RUNNING;
        }

        if (initialEV_)
        {
            BT::NodeStatus EVstatus = evaluate();
            if (EVstatus == NodeStatus::SUCCESS)
            {
                setStatus(NodeStatus::SUCCESS);
                return NodeStatus::SUCCESS;
            }
            if (EVstatus == NodeStatus::IDLE)
            {
                setStatus(NodeStatus::FAILURE);
                return NodeStatus::FAILURE;
            }
            asyncEV_ = (EVstatus == NodeStatus::RUNNING);
            initialEV_ = false;
            return NodeStatus::RUNNING;
        }

        BT::NodeStatus EXstatus = children_nodes[0]->executeTick();
        if (EXstatus == NodeStatus::SUCCESS)
        {
            if (evaluate() != NodeStatus::SUCCESS)
            {
                setStatus(NodeStatus::FAILURE);
                return NodeStatus::FAILURE;
            }
        }
        if (asyncEV_)
            evaluate();
        if (EXstatus == NodeStatus::IDLE)
            EXstatus = NodeStatus::FAILURE;
        setStatus(EXstatus);
        return EXstatus;
    }

    BT::PortsList ReFrESH_ActionModule::providedPorts()
    {
        return {
            std::string nodeName = name();
            BT::InputPort<std::string>(nodeName.append("Input")),
            BT::OutputPort<std::string>(nodeName.append("States"));
        };
    }
}
