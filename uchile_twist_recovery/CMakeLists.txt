cmake_minimum_required(VERSION 2.8.3)
project(uchile_twist_recovery)

# Find ROS dependencies
set(THIS_PACKAGE_ROS_DEPS nav_core costmap_2d geometry_msgs pluginlib base_local_planner bender_utils)
find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${bender_utils_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
)

add_library(uchile_twist_recovery src/twist_recovery.cpp)
target_link_libraries(uchile_twist_recovery ${catkin_LIBRARIES})

install(TARGETS uchile_twist_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
