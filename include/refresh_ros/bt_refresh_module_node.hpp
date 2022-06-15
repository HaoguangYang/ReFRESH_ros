#ifndef BT_REFRESH_MODULE_NODE_HPP
#define BT_REFRESH_MODULE_NODE_HPP

#include <float.h>
#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    class ReFRESH_Module : public ControlNode
    {
        public:

            ReFRESH_Module(const std::string& name,
                            const BT::NodeConfiguration& config):
                BT::ControlNode(name, config), asyncEV_(false), prestartES_(false),
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

            /**
             * @brief Public function for the root node to get the EValuator readings
             * 
             * @return std::tuple<BT::NodeStatus, float, float>
             * Module overall status, performance cost, and resource cost.
             */
            inline std::tuple<BT::NodeStatus, float, float> evaluate()
            {
                if (status()==NodeStatus::IDLE)
                {
                    // error calling EV when module is inactive.
                    throw LogicError("Calling EV when module is not running");
                    //return std::make_tuple(NodeStatus::IDLE, 1.0, 1.0);
                }
                return std::make_tuple(status(), pCost_, rCost_);
            }

            /**
             * @brief Public function for the root node to get the EStimator readings.
             * This function does NOT wake up the module node, hence no Halt is needed.
             * 
             * @return std::tuple<BT::NodeStatus, float, float>
             * Estimator returns, performance estimate, resource estimate.
             */
            inline std::tuple<BT::NodeStatus, float, float> estimate()
            {
                if (status()!=NodeStatus::IDLE)
                {
                    throw LogicError("Calling ES when module is running");
                }
                BT::NodeStatus s = estimate_();
                return std::make_tuple(s, pCost_, rCost_);
            }

        private:

            bool asyncEV_;

            bool prestartES_;

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

            /**
             * @brief Private function called to evaluate EX performance and resource.
             * 
             * @return BT::NodeStatus 
             * SUCCESS: EX performance satisfactory
             * FAILURE: EX performance below tolerance. EX will be halted immediately.
             * The restart of EX (and ultimately the entire module) is up to the scheduling
             * of the root node (Decider/Reactor).
             */
            inline BT::NodeStatus evaluate_()
            {
                // if no external evaluator is provided, use a built-in, synchronous implementation.
                if (childrenCount() < 2)
                {
                    if (children_nodes_[0]->status()==NodeStatus::SUCCESS)
                    {
                        setSuccess_();
                        return NodeStatus::SUCCESS;
                    }
                    setFailure_();
                    return NodeStatus::FAILURE;
                }
                BT::NodeStatus EVstatus = children_nodes_[1]->executeTick();
                BT::Result pmsg, rmsg;
                if ( ! (pmsg = getInput<float>("performance_cost", pCost_)) )
                {
                    haltChild(1);
                    throw BT::RuntimeError(
                        "EV missing required input [performance_cost]: ",
                        pmsg.error());
                }
                if ( ! (rmsg = getInput<float>("resource_cost", rCost_)) )
                {
                    haltChild(1);
                    throw BT::RuntimeError(
                        "EV missing required input [resource_cost]: ",
                        rmsg.error());
                }
                if (EVstatus == NodeStatus::IDLE)
                {
                    throw LogicError(
                        "EV node is ticked but still returned IDLE");
                }
                return EVstatus;
            }

            /**
             * @brief Private function called to estimate EX results.
             * 
             * EStimator returned status:
             * FAILURE: this is a hard failure and the module should not be considered a valid candidate in any case.
             * SUCCESS: this module is considered as a valid candidate
             * (N.B. this includes Soft Failure case, where the estimated performance or resource are beyond the limits).
             */
            inline BT::NodeStatus estimate_()
            {
                // if no external estimator is provided, use a built-in, always-true return as the belief.
                if (childrenCount() < 3)
                {
                    setSuccess_();
                    prestartES_ = true;
                    return NodeStatus::SUCCESS;
                }
                BT::NodeStatus ESstatus = children_nodes_[2]->executeTick();
                BT::Result pmsg, rmsg;
                if (ESstatus == NodeStatus::RUNNING)
                {
                    // Not allowed
                    haltChild(2);
                    return NodeStatus::FAILURE;
                }
                if ( ! (pmsg = getInput<float>("performance_cost", pCost_)) )
                {
                    throw BT::RuntimeError(
                        "ES missing required input [performance_cost]: ",
                        pmsg.error());
                }
                if ( ! (rmsg = getInput<float>("resource_cost", rCost_)) )
                {
                    throw BT::RuntimeError(
                        "ES missing required input [resource_cost]: ",
                        rmsg.error());
                }
                if (ESstatus == NodeStatus::IDLE)
                    throw LogicError("ES node is ticked but still returned IDLE");
                return ESstatus;
            }

            virtual BT::NodeStatus tick() override;
    };
}

#endif