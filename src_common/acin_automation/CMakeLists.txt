cmake_minimum_required(VERSION 3.0.2)
project(acin_automation)

find_package(catkin REQUIRED COMPONENTS
  tf2
  rospy
  roscpp
  moveit_core
  moveit_commander
  moveit_ros_planning
  moveit_ros_planning_interface
  message_generation
  sensor_msgs
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  visualization_msgs
  acin_reconstruction
)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  Optimize.srv
  Select.srv
)

add_action_files(
  FILES
  ExecuteScript.action
  PlanViews.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  moveit_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sequence_optimizer src/sequence_optimizer.cpp src/vertex.cpp src/edge.cpp src/node.cpp)
add_dependencies(sequence_optimizer acin_automation_gencpp)
target_link_libraries(sequence_optimizer ${catkin_LIBRARIES})

add_executable(view_planner src/view_planner.cpp)
add_dependencies(view_planner acin_automation_gencpp)
target_link_libraries(view_planner ${catkin_LIBRARIES})

