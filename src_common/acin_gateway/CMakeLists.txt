cmake_minimum_required(VERSION 3.0.2)
project(acin_gateway)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  control_msgs
  sensor_msgs
  ros_acin_robot_support
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller_gateway src/controller_gateway.cpp)
add_executable(sensor_gateway src/sensor_gateway.cpp)

target_link_libraries(controller_gateway ${catkin_LIBRARIES})
target_link_libraries(sensor_gateway ${catkin_LIBRARIES})
