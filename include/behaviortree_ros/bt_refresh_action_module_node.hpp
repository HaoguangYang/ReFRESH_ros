#ifndef BT_REFRESH_ACTION_MODULE_NODE_HPP
#define BT_REFRESH_ACTION_MODULE_NODE_HPP

#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    class ReFrESH_ActionModule : public ControlNode
    {
        public:

            ReFrESH_ActionModule(const srd::string& name);

            virtual ~ReFrESH_ActionModule() override = default;

            inline BT::NodeStatus evaluate()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 2)
                    return children_nodes_[1]->executeTick();
                if (children_count == 1)
                {
                    if (children_nodes_[0]->status()==NodeStatus::SUCCESS)
                        return NodeStatus::SUCCESS;
                }
                return NodeStatus::FAILURE;
            }

            inline BT::NodeStatus estimate()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 3)
                    return children_nodes_[2]->executeTick();
                return NodeStatus::SUCCESS;
            }

            virtual void halt() override;

            static BT::PortsList providedPorts();

        private:

            bool asyncEV_, initialEV_;

            virtual BT::NodeStatus tick() override;
    };
}

#endif