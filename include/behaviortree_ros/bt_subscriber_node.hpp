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
class RosSyncSubscriberNode : public BT::StatefulActionNode
{
protected:

  RosSubscriberNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::SyncActionNode(name, conf), node_(nh) { }

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
      InputPort<std::string>("topic_name", "name of the ROS topic")
    };
  }

  /// Method (to be implemented by the user) to receive the message.
  virtual void msgCb( const MessageType& msg ) = 0;
  /*{
    lastMsg_ = *msg;
  }*/

  /// Methods to be implemented by the user
  virtual bool isExitCondition() = 0;
  virtual bool isFailure() = 0;

  static void _msgCb( const MessageType& msg )
  {
    RosSyncSubscriberNode<MessageType>::msgCb(msg);
  }

  NodeStatus onStart() override
  {
    sub_ = node_.subscribe(getInput("topic_name"), 10,
              boost::bind(&RosSyncSubscriberNode<MessageType>::_msgCb, this, _1));
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
    sub_.unregister();
  }

protected:

  //MessageType lastMsg_;

  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  ros::Subscriber sub_;

};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosService(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
