cmake_minimum_required(VERSION 3.0.2)
project(my_navigator)

## Find catkin and other ROS packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
)

## Declare what this package provides to other packages
catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs rospy std_msgs std_srvs
)

## Install Python scripts
install(PROGRAMS
  scripts/hybrid_navigator.py
  scripts/fuzzy_navigator.py
  scripts/test_muxer.py
  scripts/test_make_plan.py
  scripts/check_system.py
  scripts/test_velocity_control.py
  scripts/hybrid_navigator_direct.py
  scripts/test_path_sampling.py
  scripts/test_path_tracking.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DIR}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DIR}
)

## Install parameter files
install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DIR}
)

## Install RViz configuration
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DIR}
)