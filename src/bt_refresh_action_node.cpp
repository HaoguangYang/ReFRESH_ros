#include "refresh_ros/bt_refresh_ros_action_node.hpp"

namespace BT
{
    bool ReFRESH_ROS_EX_action::sendGoal(GoalType& goal){
        if( !getInput<std::string>("action_request", goal.action_request) )
        {
            // abourt the entire action. Result in a FAILURE
            return false;
        }
        if( !getInput<std::string>("arguments", goal.arguments) )
        {
            // abourt the entire action. Result in a FAILURE
            return false;
        }
        return true;
    }
}
