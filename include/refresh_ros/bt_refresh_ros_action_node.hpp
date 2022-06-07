#ifndef BT_REFRESH_ROS_ACTION_NODE_HPP
#define BT_REFRESH_ROS_ACTION_NODE_HPP

#include "behaviortree_ros/bt_action_node.hpp"
#include "behaviortree_ros/bt_service_node.hpp"
#include "refresh_ros/HighLevelRequestAction.h"
#include "refresh_ros/ModuleEstimate.h"

namespace BT
{
    class ReFRESH_ROS_EX_node :  public RosActionNode<refresh_ros::HighLevelRequestAction>
    {
        public:

            ReFRESH_ROS_EX_node(
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

            virtual ~ActionEvaluatorNode() = default;

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
                return spinOnceImpl();
            }

            /// method invoked by an action in the RUNNING state.
            inline BT::NodeStatus onRunning() override
            {
               return spinOnceImpl();
            }

            inline void onHalted() override
            {
                // TODO: what to do here?
                return;
            }
        
        protected:
            FeedbackType fb_;
            float pCost_, rCost_;
    };

    /// Method to register the evaluator into a factory.
    /// It gives you the opportunity to set the ros::NodeHandle.
    template <class DerivedT> static
    void RegisterActionEvaluator(BT::BehaviorTreeFactory& factory,
                        const std::string& registration_ID)
    {
        NodeBuilder builder = [](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<DerivedT>( name, config );
        };

        TreeNodeManifest manifest;
        manifest.type = getType<DerivedT>();
        manifest.ports = DerivedT::providedPorts();
        manifest.registration_ID = registration_ID;
        const auto& basic_ports = ActionEvaluatorNode< typename DerivedT::ActionType >::providedPorts();
        manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

        factory.registerBuilder( manifest, builder );
    }

    class ReFRESH_ROS_EV_node : public ActionEvaluatorNode<refresh_ros::HighLevelRequestAction>
    {
        public:
            ReFRESH_ROS_EV_node( const std::string& name, const NodeConfiguration & conf):
                ActionEvaluatorNode<refresh_ros::HighLevelRequestAction>(name, conf) {}

            virtual BT::NodeStatus spinOnce() override;
    };

    class ReFrESH_ROS_ES_node : public RosServiceNode<refresh_ros::ModuleEstimate>
    {
        public:
            ReFrESH_ROS_ES_node( ros::NodeHandle& handle, const std::string& name, const NodeConfiguration & conf):
                RosServiceNode<refresh_ros::ModuleEstimate>(handle, name, conf) {}

            /// These ports will be added automatically if this Node is
            /// registered using RegisterReFRESH_EV<DeriveClass>()
            static PortsList providedPorts()
            {
                return {
                    OutputPort<float>("performance_cost"),
                    OutputPort<float>("resource_cost")
                };
            }

            virtual void sendRequest(RequestType& request) override;

            virtual NodeStatus onResponse(const ResponseType& rep) override;
    };

}

#endif