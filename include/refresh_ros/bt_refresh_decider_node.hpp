#ifndef BT_REFRESH_DECIDER_NODE_HPP
#define BT_REFRESH_DECIDER_NODE_HPP

#include <float.h>
#include "behaviortree_cpp_v3/control_node.h"
#include "refresh_ros/ModuleEvaluate.h"
#include "refresh_ros/bt_refresh_module_node.hpp"

namespace BT
{
    class ReFRESH_Cost : public refresh_ros::ModuleEvaluate
    {
        public:

            ReFRESH_Cost(const refresh_ros::ModuleEvaluate::ConstPtr& msg);

            inline void setPerformanceWeight(float& pWeight)
            {
                costUpdated_ = false;
                pWeight_ = pWeight;
            }

            inline void setResourceWeight(float& rWeight)
            {
                costUpdated_ = false;
                rWeight_ = rWeight;
            }

            inline void updateWeightedCost() const
            {
                if (costUpdated_)
                    return;
                weightedCost = pWeight_*performanceCost + rWeight_*resourceCost;
                costUpdated_ = true;
            }

            inline void setWeights(float& pWeight, float& rWeight)
            {
                setPerformanceWeight(pWeight);
                setResourceWeight(rWeight);
                updateWeightedCost();
            }

            mutable float weightedCost;

        private:
            mutable bool costUpdated_;
            float pWeight_, rWeight_;
    };

    bool operator<(const ReFRESH_Cost& lhs, const ReFRESH_Cost& rhs)
    {
        lhs.updateWeightedCost();
        rhs.updateWeightedCost();
        return lhs.weightedCost < rhs.weightedCost;
    }

    class ReFRESH_Decider : public ControlNode
    {
        public:

            ReFRESH_Decider(const std::string& name);

            virtual ~ReFRESH_Decider() override = default;

            virtual void halt() override;

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<float>("performance_weight"),
                    BT::InputPort<float>("resource_weight")
                };
            }

        private:

            int indActive_;
            
            std::vector<ReFRESH_Cost> moduleCost_;

            virtual BT::NodeStatus tick() override;
    };
}

#endif