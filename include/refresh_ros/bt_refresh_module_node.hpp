#ifndef BT_REFRESH_MODULE_NODE_HPP
#define BT_REFRESH_MODULE_NODE_HPP

#include <float.h>
#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    class ReFRESH_Module : public ControlNode
    {
        public:

            ReFRESH_Module(const std::string& name, const BT::NodeConfiguration& config):
                BT::ControlNode(name, config), asyncEV_(false), initialEV_(false),
                pCost_(0.5), rCost_(0.5)
            {}

            virtual ~ReFRESH_Module() override = default;

            virtual void halt() override;

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<float>("performance_cost"),
                    BT::InputPort<float>("resource_cost")
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
                    BT::Result pmsg, rmsg;
                    if ( ! (pmsg = getInput<float>("performance_cost", pCost_)) )
                        throw BT::RuntimeError("EV missing required input [performance_cost]: ", pmsg.error());
                    if ( ! (rmsg = getInput<float>("resource_cost", rCost_)) )
                        throw BT::RuntimeError("EV missing required input [resource_cost]: ", rmsg.error());
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
                if (!initialEV_)
                    setFailure_();
                return NodeStatus::FAILURE;
            }

            inline BT::NodeStatus estimate_()
            {
                const size_t children_count = children_nodes_.size();
                if (children_count >= 3)
                {
                    BT::NodeStatus ESstatus = children_nodes_[2]->executeTick();
                    BT::Result pmsg, rmsg;
                    if ( ! (pmsg = getInput<float>("performance_cost", pCost_)) )
                        throw BT::RuntimeError("ES missing required input [performance_cost]: ", pmsg.error());
                    if ( ! (rmsg = getInput<float>("resource_cost", rCost_)) )
                        throw BT::RuntimeError("ES missing required input [resource_cost]: ", rmsg.error());
                    return ESstatus;
                }
                setSuccess_();
                return NodeStatus::SUCCESS;
            }

            virtual BT::NodeStatus tick() override;
    };
}

#endif