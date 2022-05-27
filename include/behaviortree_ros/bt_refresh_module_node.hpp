#ifndef BT_REFRESH_MODULE_NODE_HPP
#define BT_REFRESH_MODULE_NODE_HPP

#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    class ReFrESH_Module : public ControlNode
    {
        public:

            ReFrESH_Module(const srd::string& name);

            virtual ~ReFrESH_Module() override = default;

            virtual void halt() override;

            static BT::PortsList providedPorts();

        private:

            bool asyncEV_, initialEV_;

            inline BT::NodeStatus evaluate_()
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

            inline BT::NodeStatus estimate_()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 3)
                    return children_nodes_[2]->executeTick();
                return NodeStatus::SUCCESS;
            }

            virtual BT::NodeStatus tick() override;
    };
}

#endif