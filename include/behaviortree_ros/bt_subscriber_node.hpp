#ifndef BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_BT_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/**
 * Base Action to implement a ROS Subscriber
 */
template<class MessageT>
class RosSubscriberNode : public BT::StatefulActionNode
{
protected:

  RosSubscriberNode(ros::NodeHandle& nh, const std::string& name,
                    const BT::NodeConfiguration & conf):
   BT::StatefulActionNode(name, conf), node_(nh) { }

public:

  using BaseClass    = RosSubscriberNode<MessageT>;
  using MessageType  = MessageT;

  RosSubscriberNode() = delete;

  virtual ~RosSubscriberNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("topic_name", "name of the ROS topic"),
      InputPort<unsigned>("buffer_length", 10, "length of message buffer")
    };
  }

  /// Methods to be implemented by the user
  virtual bool isExitCondition() = 0;
  virtual bool isFailure() = 0;

  inline void onMessageCb( const MessageType::ConstPtr& msg ){ lastMsgPtr_ = msg; }

  NodeStatus onStart() override
  {
    BT::Result inRes;
    std::string topic_name;
    if ( !(inRes = getInput<std::string>("topic_name", topic_name)))
        throw(BT::RuntimeError(
          "ROS Action Node missing required input [topic_name]: ", inRes.error()));
    sub_ = node_.subscribe(topic_name, getInput<unsigned>("topic_name").value(),
            boost::bind(&RosSubscriberNode<MessageType>::onMessageCb, this, _1));
  }

  NodeStatus onRunning() override
  {
    if (!isExitCondition())
      return NodeStatus::RUNNING;
    sub_.unregister();
    if (isFailure())
    {
      return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    sub_.shutdown();
    setStatus(NodeStatus::IDLE);
  }

protected:

  MessageType::ConstPtr lastMsgPtr_;

  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  ros::Subscriber sub_;

};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosSubscriber(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name,
                                      const NodeConfiguration& config)
  {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports =
    RosSubscriberNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
