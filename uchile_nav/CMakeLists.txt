cmake_minimum_required(VERSION 2.8.3)
project(uchile_nav)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bender_msgs
  bender_srvs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  bender_utils
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.7 REQUIRED COMPONENTS common)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uchile_nav
  CATKIN_DEPENDS actionlib bender_msgs bender_srvs geometry_msgs move_base_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs std_srvs tf bender_utils pcl_ros
  DEPENDS pcl
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${bender_utils_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## goalServer
add_executable(
  goal_server
  src/main.cpp
  src/GoalServer.cpp
  src/GoalServerState.cpp
  src/GoalHandler.cpp
  src/GoalCalculator.cpp
)
add_dependencies(goal_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_server ${catkin_LIBRARIES})

