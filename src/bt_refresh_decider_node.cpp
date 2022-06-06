#include "refresh_ros/bt_refresh_decider_node.hpp"
#include <float.h>

namespace BT
{
    ReFRESH_Decider::ReFRESH_Decider(const std::string& name):
        ControlNode::ControlNode(name, {}), indActive_(-1)
    {
        setRegistrationID("ReFRESH_Decider");
    }

    void ReFRESH_Decider::halt()
    {
        setStatus(NodeStatus::IDLE);
    }

    BT::NodeStatus ReFRESH_Decider::turnOnBest()
    {
        std::vector<ReFRESH_Cost> moduleCost_;
        int ind;
        float pWeight_ = getInput<float>("performance_weight").value();
        float rWeight_ = getInput<float>("resource_weight").value();
        for (ind = 0; ind < childrenCount(); ind ++)
        {
            // this list is for inactive modules.
            if (ind == indActive_)
                continue;
            BT::NodeStatus childStatus = children_nodes_[ind]->executeTick();
            // this is a fall-back node, anyway.
            if (childStatus == NodeStatus::SUCCESS)
            {
                return NodeStatus::SUCCESS;
            }
            BT::ReFRESH_Module* tryConvert = dynamic_cast<BT::ReFRESH_Module*>(children_nodes_[ind]);
            if (tryConvert == nullptr)
            {
                // not a refresh module. if node status is success then make its cost 1-eps (least considered)
                float pCost_, rCost_;
                if (childStatus == NodeStatus::FAILURE)
                {
                    pCost_ = 1.0;
                    rCost_ = 1.0;
                } else {
                    pCost_ = 1.0-FLT_EPSILON;
                    rCost_ = 1.0-FLT_EPSILON;
                }
                moduleCost_.push_back(ReFRESH_Cost(ind, pCost_, rCost_));
                moduleCost_.back().setWeights(pWeight_, rWeight_);
                continue;
            }
            std::tuple<BT::NodeStatus, float, float> mAssess = tryConvert->assess();
            moduleCost_.push_back(ReFRESH_Cost(ind, std::get<1>(mAssess), std::get<2>(mAssess)));
            moduleCost_.back().setWeights(pWeight_, rWeight_);
        }
        // sort moduleCost_
        std::stable_sort(moduleCost_.begin(), moduleCost_.end());
        // check feasibility in cost-ascending order
        ind = 0;
        bool bestPossibleSet = false;
        while (true)
        {
            if (ind >= moduleCost_.size())
            {
                // no candidate satisfies the requirements
                return NodeStatus::FAILURE;
            }
            if (!moduleCost_[ind].feasible())
            {
                if (!bestPossibleSet && moduleCost_[ind].resourceFeasible())
                {
                    bestPossibleSet = true;
                    bestPossible_ = moduleCost_[ind].which();
                }
                haltChild(moduleCost_[ind].which());
                ind ++;
                continue;
            }
            // take the module with smallest cost, and note its index as indActive_
            int preOn = moduleCost_[ind].which();
            if (children_nodes_[preOn]->status() == NodeStatus::FAILURE)
            {
                // if a solution is feasible, it should NOT has ES returned FAILURE.
                // This rule-out is just to be safe.
                haltChild(preOn);
                ind ++;
                continue;
            }
            indActive_ = preOn;
            ind ++;
            break;
        }
        // halt all other modules.
        for ( ; ind < childrenCount(); ind ++)
        {
            haltChild(moduleCost_[ind].which());
        }
        return NodeStatus::RUNNING;
    }

    BT::NodeStatus ReFRESH_Decider::tick()
    {
        if (status() == NodeStatus::IDLE)
        {
            // no module is running yet. tick estimator of each module by sending a tick signal to each idle module.
            return turnOnBest();
        }
        // one module is already running. tick until its status to turn SUCCESS or FAILURE, and check EV status.
        BT::NodeStatus childStatus = children_nodes_[indActive_]->executeTick();
        // update its EV reading after each tick.
        std::tuple<BT::NodeStatus, float, float> mAssess =
            dynamic_cast<BT::ReFRESH_Module*>(children_nodes_[indActive_])->assess();
        ReFRESH_Cost activeModuleCost =
            ReFRESH_Cost(indActive_, std::get<1>(mAssess), std::get<2>(mAssess));
        float pWeight_ = getInput<float>("performance_weight").value();
        float rWeight_ = getInput<float>("resource_weight").value();
        activeModuleCost.setWeights(pWeight_, rWeight_);
        // Upon terminal state, check EV cost reading.
        // If any child module returns SUCCESS, return SUCCESS.
        if (childStatus == NodeStatus::SUCCESS)
        {
            setStatus(NodeStatus::SUCCESS);
            return NodeStatus::SUCCESS;
        }
        // If EV cost >=1 (or FAILURE), trigger reconfig.
        if (childStatus == NodeStatus::FAILURE || !activeModuleCost.feasible())
        {
            // tick inactive modules once, update and sort moduleCost_
            int lastRun = indActive_;
            BT::NodeStatus bestResultsInCandidates = turnOnBest();
            // indActive_ has been changed, if a viable solution is found. We need to verify it.
            // if status is not FAILURE, the result is valid.
            // get policy when candidate can not provide a satisfactory alternative.
            if (bestResultsInCandidates == NodeStatus::FAILURE)
            {
                // select between useBestPossible, and keepCurrentConfig.
                // setActive and tick the min-cost and not last-run module
            }
            
            // halt all other modules
            if (lastRun != indActive_)
                haltChild(lastRun);
            
            if (isDepleted())
            {
                setStatus(NodeStatus::FAILURE);
                return NodeStatus::FAILURE;
            }
        }
        return NodeStatus::RUNNING;
    }
}
