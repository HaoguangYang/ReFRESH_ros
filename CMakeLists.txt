cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(refresh_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp
    rospy
    std_msgs
    behaviortree_cpp_v3
    actionlib_msgs
    actionlib
    message_generation
    dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
  ModuleEvaluate.msg
  ReFRESHrequest.msg
  ReFRESHtelemetry.msg
)

add_service_files(
   FILES
   AddTwoInts.srv
   ModuleEstimate.srv
   ModuleControl.srv
 )

add_action_files(
   DIRECTORY action
   FILES
   Fibonacci.action
   HighLevelRequest.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

#add dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/TaskBehaviortreeEngine.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS} )

######################################################
# Also build LIBRARIES

add_library(ReFRESHmoduleNode src/bt_refresh_module_node.cpp)
add_dependencies(ReFRESHmoduleNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ReFRESHmoduleNode ${catkin_LIBRARIES})

add_library(ReFRESHactionNode src/bt_refresh_ros_action_node.cpp)
add_dependencies(ReFRESHactionNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ReFRESHactionNode ${catkin_LIBRARIES})

add_library(ReFRESHcontrolNode src/bt_refresh_control_node.cpp)
add_dependencies(ReFRESHcontrolNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ReFRESHcontrolNode ${catkin_LIBRARIES})

######################################################
# Task BT engine

add_executable(ReFRESH_ROS_TaskBT_Engine src/task_bt_engine.cpp)
add_dependencies(ReFRESH_ROS_TaskBT_Engine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(ReFRESH_ROS_TaskBT_Engine
    ${catkin_LIBRARIES}
    ReFRESHmoduleNode
    ReFRESHactionNode
    ReFRESHcontrolNode
)

######################################################
# TESTS

add_executable(test_bt test/test_bt.cpp)
add_dependencies(test_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bt ${catkin_LIBRARIES} )

add_executable(test_bt_server test/test_server.cpp)
add_dependencies(test_bt_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bt_server ${catkin_LIBRARIES} )

######################################################
# INSTALL


######################################################
# EXAMPLES and TOOLS



