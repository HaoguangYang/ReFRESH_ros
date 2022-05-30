# ReFRESH (Reconfigurable Framework for Real-time Embedded Software and Hardware)

This is a ROS integration of the ReFRESH framework as a middleware for real-time
self-adaptation of robotic systems on both behavior and composition, involving
both software and hardware.

Based on BehaviorTree.CPP and Actionlib, this ROS package extends the run-time
self-adaptation capability of ROS. The framework of ReFrESH is referred to
(Cui et. al, 2015), where a Evaluator and Estimator are added to the functional
component (Executor) to achieve run-time self-awareness. A decider reconfigures
the system on-the-fly based on the EV and ES outputs. This package further
extends the work of ReFRESH by decoupling behavior and logic concerns. Hence,
it consists of High-Level (behavior) adaptation modules powered by a behavior
tree, and Low-Level (component and logic) adaptation modules integrated with
the native ROS API.

## BehaviorTree.ROS

[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) is a __middleware independent__ framework 
to develop Behavior Trees in C++.

The library is not particularly opinionated about the way Actions and Conditions should be created; this gives
more freedom to the developer, but can also be confusing for those people which are getting started with it.

Consequently, many people in the [ROS](http://www.ros.org) community asked for examples and guidelines;
this repository try to provide some basic examples.

Currently, two wrappers are provided:

- [RosServiceNode](include/behaviortree_ros/bt_service_node.h), which can be used to call
  [ROS Services](http://wiki.ros.org/Services)

- [RosActionNode](include/behaviortree_ros/bt_action_node.h) that, similarly, is a wrapper around
  [actionlib::SimpleActionClient](http://wiki.ros.org/actionlib).


