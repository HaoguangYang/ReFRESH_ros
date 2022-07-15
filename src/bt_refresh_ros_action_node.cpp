#include "refresh_ros/bt_refresh_ros_action_node.hpp"

namespace BT
{
    bool ReFRESH_ROS_EX_node::sendGoal(GoalType &goal)
    {
        if (!getInput<std::string>("action_request", goal.action_request))
        {
            // abort the entire action. Result in a FAILURE
            return false;
        }
        if (!getInput<std::string>("arguments", goal.arguments))
        {
            // abort the entire action. Result in a FAILURE
            return false;
        }
        return true;
    }

    BT::NodeStatus ReFRESH_ROS_EV_node::spinOnce()
    {
        pCost_ = fb_->evaluate.performanceCost;
        rCost_ = fb_->evaluate.resourceCost;
        if (pCost_ >= 1.0)
            return NodeStatus::RUNNING;
        if (rCost_ >= 1.0)
            return NodeStatus::RUNNING;
        return NodeStatus::SUCCESS;
    }

    bool ReFrESH_ROS_ES_node::sendRequest(RequestType &request)
    {
        if (!getInput<std::string>("action_request", request.action_request))
            return false;
        if (!getInput<std::string>("arguments", request.arguments))
            return false;
        return true;
    }

    BT::NodeStatus ReFrESH_ROS_ES_node::onResponse(const ResponseType &rep)
    {
        setOutput("performance_cost", rep.estimate.performanceCost);
        setOutput("resource_cost", rep.estimate.resourceCost);
        if (rep.estimate.performanceCost >= 1.0)
            return NodeStatus::FAILURE;
        if (rep.estimate.resourceCost >= 1.0)
            return NodeStatus::FAILURE;
        return NodeStatus::SUCCESS;
    }
}
