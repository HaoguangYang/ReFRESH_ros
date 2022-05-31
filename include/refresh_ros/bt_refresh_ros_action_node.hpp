#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "behaviortree_ros/bt_action_node.hpp"
#include "behaviortree_ros/bt_service_node.hpp"
#include "refresh_ros/HighLevelRequestAction.h"

namespace BT
{
    class ReFRESH_ROS_EX_action :  public RosActionNode<refresh_ros::HighLevelRequestAction>
    {
        public:

            ReFRESH_ROS_EX_action(
                ros::NodeHandle& handle,
                const std::string& node_name,
                const NodeConfiguration & conf) :
                RosActionNode<refresh_ros::HighLevelRequestAction>(handle, node_name, conf)
            {}

            static PortsList providedPorts()
            {
                return {
                    InputPort<std::string>("action_request"),
                    InputPort<std::string>("arguments")
                };
            }

            bool sendGoal(GoalType& goal) override;

    };

    template<class ActionT>
    class ActionEvaluatorNode : public BT::StatefulActionNode
    {
        protected:
            ActionEvaluatorNode(const std::string& name, const BT::NodeConfiguration& conf):
                BT::StatefulActionNode(name, conf)
            {}

        public:
            using ActionType = ActionT;
            using FeedbackType = typename ActionT::_action_feedback_type::_feedback_type::ConstPtr;

            ActionEvaluatorNode() = delete;

            virtual ~RosActionNode() = default;

            /// These ports will be added automatically if this Node is
            /// registered using RegisterReFRESH_EV<DeriveClass>()
            static PortsList providedPorts()
            {
                return {
                    InputPort<FeedbackType>("feedback"),
                    OutputPort<float>("performance_cost"),
                    OutputPort<float>("resource_cost")
                };
            }

            virtual BT::NodeStatus spinOnce() = 0;

            inline BT::NodeStatus spinOnceImpl()
            {
                BT::Result fbRes;
                if ( !(fbRes = getInput<FeedbackType>("feedback", fb_)))
                    throw(BT::RuntimeError("Action Evaluator Node missing required input [feedback]: ", fbRes.error()));
                BT::NodeStatus status = spinOnce();
                setOutput("performance_cost", pCost_);
                setOutput("resource_cost", rCost_);
                setStatus(status);
                return status;
            }

            inline BT::NodeStatus onStart() override
            {
                setStatus(NodeStatus::RUNNING);
                spinOnceImpl();
            }

            /// method invoked by an action in the RUNNING state.
            inline BT::NodeStatus onRunning() override
            {
               spinOnceImpl();
            }
        
        protected:
            FeedbackType fb_;
            float pCost_, rCost_;
    };

    class ReFRESH_ROS_EV_node : public ActionEvaluatorNode<refresh_ros::HighLevelRequestAction>
    {
        public:
            virtual BT::NodeStatus spinOnce() override
            {
                pCost_ = fb_->evaluate.performanceCost;
                rCost_ = fb_->evaluate.resourceCost;
                if (pCost_ >= 1.0)
                    return NodeStatus::RUNNING;
                if (rCost_ >= 1.0)
                    return NodeStatus::RUNNING;
                return NodeStatus::SUCCESS
            }
    };

    class ReFrESH_ROS_ES_node : public RosServiceNode<refresh_ros::ModuleEstimate>
    {
        public:
            /// These ports will be added automatically if this Node is
            /// registered using RegisterReFRESH_EV<DeriveClass>()
            static PortsList providedPorts()
            {
                return {
                    OutputPort<float>("performance_cost"),
                    OutputPort<float>("resource_cost")
                };
            }
    }

}

#endif