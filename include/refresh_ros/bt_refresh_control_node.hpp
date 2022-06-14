#ifndef BT_REFRESH_CONTROL_NODE_HPP
#define BT_REFRESH_CONTROL_NODE_HPP

#include <float.h>
#include "behaviortree_cpp_v3/control_node.h"
#include "refresh_ros/ModuleEvaluate.h"
#include "refresh_ros/bt_refresh_module_node.hpp"

namespace BT
{
    class ReFRESH_Cost
    {
        public:
            ReFRESH_Cost();

            ReFRESH_Cost(size_t index, float& pCost, float& rCost);

            ReFRESH_Cost(size_t index, const refresh_ros::ModuleEvaluate::ConstPtr& msg);

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

            inline void setPerformanceCost(float& pCost)
            {
                costUpdated_ = false;
                pCost_ = pCost;
            }

            inline void setResourceCost(float& rCost)
            {
                costUpdated_ = false;
                rCost_ = rCost;
            }

            inline void updateWeightedCost() const
            {
                if (costUpdated_)
                    return;
                wCost_ = pWeight_*pCost_ + rWeight_*rCost_;
                costUpdated_ = true;
            }

            inline void updateCosts(float& pCost, float& rCost)
            {
                setPerformanceCost(pCost);
                setResourceCost(rCost);
                updateWeightedCost();
            }

            inline void setWeights(float& pWeight, float& rWeight)
            {
                setPerformanceWeight(pWeight);
                setResourceWeight(rWeight);
                updateWeightedCost();
            }

            inline size_t which() { return ind_; }

            inline void which(size_t ind) { ind_ = ind; }

            inline float weightedCost() const { return wCost_; }

            inline bool resourceFeasible() { return (rCost_ < 1.0); }

            inline bool performanceFeasible() { return (pCost_ < 1.0); }

            inline bool feasible() { return (pCost_ < 1.0 && rCost_ < 1.0); }

        private:
            mutable bool costUpdated_;
            size_t ind_;
            float pWeight_, rWeight_;
            float pCost_, rCost_;
            mutable float wCost_;
    };

    bool operator<(const ReFRESH_Cost& lhs, const ReFRESH_Cost& rhs)
    {
        lhs.updateWeightedCost();
        rhs.updateWeightedCost();
        return lhs.weightedCost() < rhs.weightedCost();
    }

    class ReFRESH_Decider : public ControlNode
    {
        public:
            ReFRESH_Decider(const std::string& name, const BT::NodeConfiguration& config):
                BT::ControlNode(name, config), indActive_(-1), bestPossible_(0)
            { nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0); }

            virtual ~ReFRESH_Decider() override = default;

            virtual void halt() override;

            //virtual bool isDepleted() { return false; }

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<float>("performance_weight"),
                    BT::InputPort<float>("resource_weight"),
                    BT::InputPort<unsigned int>("retries", 3, "Number of retries for each child upon failure"),
                    BT::InputPort<bool>("fallback_no_reconfig")
                };
            }

            BT::NodeStatus turnOnBest();

        private:

            int indActive_;

            int bestPossible_;

            std::vector<unsigned int> nRetry_;

            ReFRESH_Cost activeModuleCost_;

            virtual BT::NodeStatus tick() override;
    };

    class ReFRESH_Reactor : public ControlNode
    {
        public:
            ReFRESH_Reactor(const std::string& name, const BT::NodeConfiguration& config):
                BT::ControlNode(name, config), indActive_(-1), bestPossible_(0)
            { nRetry_ = std::vector<unsigned int>(children_nodes_.size(), 0); }

            virtual ~ReFRESH_Reactor() override = default;

            virtual void halt() override;

            //virtual bool isDepleted() { return false; }

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<float>("performance_weight"),
                    BT::InputPort<float>("resource_weight"),
                    BT::InputPort<unsigned int>("retries", 3, "Number of retries for each child upon failure"),
                    BT::InputPort<bool>("fallback_no_reconfig")
                };
            }

            BT::NodeStatus turnOnBestMitigation();

            BT::NodeStatus turnOnNominal();

        private:

            int indActive_;

            int bestPossible_;

            std::vector<unsigned int> nRetry_;

            ReFRESH_Cost activeModuleCost_;

            virtual BT::NodeStatus tick() override;
    };
}

#endif