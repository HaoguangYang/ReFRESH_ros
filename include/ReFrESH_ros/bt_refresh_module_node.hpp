#ifndef BT_REFRESH_MODULE_NODE_HPP
#define BT_REFRESH_MODULE_NODE_HPP

#include <float.h>
#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    class ReFrESH_Module : public ControlNode
    {
        public:

            ReFrESH_Module(const std::string& name);

            virtual ~ReFrESH_Module() override = default;

            virtual void halt() override;

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<float>("PerformanceCost"),
                    BT::InputPort<float>("ResourceCost")
                };
            }

            inline std::tuple<BT::NodeStatus, float, float> assess()
            {
                return std::make_tuple(children_nodes_[0]->status(), pCost_, rCost_);
            }

        private:

            bool asyncEV_, initialEV_;

            /**
             * @brief Performance cost and resource cost. Normalized values [0..1]
             * >=1 values indicate the candidate is infeasible.
             */
            float pCost_, rCost_;

            /**
             * @brief This setting of performance cost and resource cost ensures that
             * modules with default EV and ES are considered last in a candidate set
             * with other modules that have explicitly specified EV and ES parts.
             */
            inline void setSuccess_()
            {
                pCost_ = 1. - FLT_EPSILON ;
                rCost_ = 1. - FLT_EPSILON ;
            }

            inline void setFailure_()
            {
                pCost_ = 1.;
                rCost_ = 1.;
            }

            inline BT::NodeStatus evaluate_()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 2)
                {
                    BT::NodeStatus EVstatus = children_nodes_[1]->executeTick();
                    Optional<float> pmsg = getInput<float>("PerformanceCost");
                    if (!pmsg)
                        throw BT::RuntimeError("EV missing required input [message]: ", pmsg.error());
                    pCost_ = pmsg.value();
                    Optional<float> rmsg = getInput<float>("ResourceCost");
                    if (!rmsg)
                        throw BT::RuntimeError("EV missing required input [message]: ", rmsg.error());
                    rCost_ = rmsg.value();
                    return EVstatus;
                }
                if (children_count == 1)
                {
                    if (children_nodes_[0]->status()==NodeStatus::SUCCESS)
                    {
                        setSuccess_();
                        return NodeStatus::SUCCESS;
                    }
                }
                setFailure_();
                return NodeStatus::FAILURE;
            }

            inline BT::NodeStatus estimate_()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 3)
                {
                    BT::NodeStatus ESstatus = children_nodes_[2]->executeTick();
                    Optional<float> pmsg = getInput<float>("PerformanceCost");
                    if (!pmsg)
                        throw BT::RuntimeError("ES missing required input [message]: ", pmsg.error());
                    pCost_ = pmsg.value();
                    Optional<float> rmsg = getInput<float>("ResourceCost");
                    if (!rmsg)
                        throw BT::RuntimeError("ES missing required input [message]: ", rmsg.error());
                    rCost_ = rmsg.value();
                    return ESstatus;
                }
                setSuccess_();
                return NodeStatus::SUCCESS;
            }

            virtual BT::NodeStatus tick() override;
    };
}

#endif