cmake_minimum_required(VERSION 2.8.3)
project(uchile_navigation_utils)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  uchile_msgs
  uchile_srvs
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
  #INCLUDE_DIRS include
  LIBRARIES uchile_nav
  CATKIN_DEPENDS actionlib uchile_msgs uchile_srvs geometry_msgs move_base_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs std_srvs tf bender_utils pcl_ros
  DEPENDS pcl
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${bender_utils_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# particle filter spreader 
add_executable(amcl_spreader src/amclSpreader.cpp)
add_dependencies(amcl_spreader ${catkin_EXPORTED_TARGETS})
target_link_libraries(amcl_spreader ${catkin_LIBRARIES})

# obstacles cleaner
add_executable(costmap_cleaner src/CostmapCleaner.cpp)
add_dependencies(costmap_cleaner ${catkin_EXPORTED_TARGETS})
target_link_libraries(costmap_cleaner ${catkin_LIBRARIES})

# move base automatic replaner
add_executable(plan_remaker src/PlanRemaker.cpp)
add_dependencies(plan_remaker ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_remaker ${catkin_LIBRARIES})

# robot pose estimate publisher
add_executable(pose_publisher src/BenderMapPose.cpp)
add_dependencies(pose_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_publisher ${catkin_LIBRARIES})

