cmake_minimum_required(VERSION 3.0.2)
project(ros_acin_robot_control)
set( CMAKE_VERBOSE_MAKEFILE on )
## Compile as C++11, supported in ROS Kinetic and newer
#-march=native
# set(CMAKE_ROBOT_MODEL_FOLDER kukalbriiwa_model)
# set(CMAKE_ROBOT_MODEL_PATH_FULL "include/${PROJECT_NAME}/models/${CMAKE_ROBOT_MODEL_FOLDER}")
# set(CAMKE_ROBOT_DIM 7)
# set(CAMKE_Traj_N_Knot_ 30)

add_compile_options(-std=c++14 -fPIC -g -O1 -mfpmath=387 -fopenmp -march=x86-64 ) #-fno-stack-protector
MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
string( REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-strength-reduce -pipe -O1 -DFLAG_ROS_COMPILE")# -Og
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -pipe -O3 -fopenmp -DNDEBUG")
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CXX_COMPILER_ID} )
# set(CMAKE_SHARED_LINKER_FLAGS "-lomp -l${EIGEN3_INCLUDE_DIRS}")
if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  ros_acin_robot_support
)
find_package(ros_acin_robot_support)

## System dependencies are found with CMake's conventions
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(Qt5Core 5.1.0 CONFIG REQUIRED)
find_package(Qt5Gui 5.1.0 CONFIG)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES std_msgs geometry_msgs ros_acin_robot_support
# )
#generate_messages(
#DEPENDENCIES std_msgs geometry_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_acin_robot_control ros_acin_robot_support
  CATKIN_DEPENDS message_generation message_runtime geometry_msgs roscpp sensor_msgs std_msgs ros_acin_robot_support
  DEPENDS EIGEN3 ros_acin_robot_support
)

###########
## Build ##
###########

# set up a mapping so that the Release configuration for the Qt imported target is
# used in the COVERAGE CMake configuration.
# set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
# QT5_WRAP_CPP(MOC_Files
#   ../ros_acin_robot_support/include/ros_acin_robot_support/acin_ctrl_UI.cpp
# )
# # ## Declare a C++ library
# add_library(acin_ctrl_UI SHARED
#   ../ros_acin_robot_support/include/ros_acin_robot_support/acin_ctrl_UI.cpp
# )
#
# target_link_libraries(acin_ctrl_UI Qt5::Core Qt5::Widgets)
# set_target_properties(acin_ctrl_UI PROPERTIES LINKER_LANGUAGE CXX)
# add_dependencies(acin_ctrl_UI ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} Eigen3::Eigen Qt5::Core Qt5::Widgets)
# set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")
#
# # set up a mapping so that the Release configuration for the Qt imported target is
# # used in the COVERAGE CMake configuration.
# set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")
# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ../ros_acin_robot_support/include
)

# QT5_WRAP_CPP(MOC_Files
#   include/${PROJECT_NAME}/acin_ctrl_UI.cpp
# )
# # ## Declare a C++ library
# add_library(acin_ctrl_UI SHARED
#   include/${PROJECT_NAME}/acin_ctrl_UI.cpp
# )


# set_target_properties(evalT0E PROPERTIES SOVERSION 1)
# add_library(evalJp SHARED
#   include/${PROJECT_NAME}/models/kukalbriiwa/evalJp.cpp
#   # include/${PROJECT_NAME}/evalJ.cpp
#   # include/${PROJECT_NAME}/evalG.cpp
#   # include/${PROJECT_NAME}/evalC.cpp
#   # include/${PROJECT_NAME}/evalM.cpp
# )
# set_target_properties(evalJp PROPERTIES SOVERSION 1)
# add_library(evalJ SHARED
#   include/${PROJECT_NAME}/models/kukalbriiwa/evalJ.cpp
# )
# set_target_properties(evalJ PROPERTIES SOVERSION 1)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/ros_acin_controller_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_joint_ct_node evalT0E evalJp evalJ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME}_cartesian_ct_node evalT0E evalJp evalJ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME} evalT0E evalJp evalJ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(evalT0E ${catkin_LIBRARIES} Eigen3::Eigen
# )
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_joint_ct_node
#   ${catkin_LIBRARIES} evalT0E evalJp evalJ
# )
# target_link_libraries(${PROJECT_NAME}_cartesian_ct_node
#   ${catkin_LIBRARIES} evalT0E evalJp evalJ
# )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} -lomp Threads::Threads Qt5::Core Qt5::Widgets
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_joint_ct_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_acin_iiwa_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
