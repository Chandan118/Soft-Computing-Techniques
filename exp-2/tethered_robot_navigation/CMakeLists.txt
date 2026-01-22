# Specify the minimum version of CMake required to build this package.
cmake_minimum_required(VERSION 3.0.2)

# Declare the name of the project, which must match the package name.
project(tethered_robot_navigation)

#-----------------------------------------------------------------------------
# Find Dependencies
#-----------------------------------------------------------------------------
# Find catkin and all the ROS packages our nodes depend on.
# These dependencies are used to generate messages and other build-time necessities.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

#-----------------------------------------------------------------------------
# Catkin Package Information
#-----------------------------------------------------------------------------
# This function is crucial for defining what this package exports.
# Since we have no custom messages or libraries, this section is simple.
catkin_package(
#  INCLUDE_DIRS include        # No C++ headers to export
#  LIBRARIES tethered_robot_navigation # No libraries to export
#  CATKIN_DEPENDS rospy ...  # Runtime dependencies are in package.xml
#  DEPENDS system_lib
)

#-----------------------------------------------------------------------------
# Install Python Executables
#-----------------------------------------------------------------------------
# This is the modern, correct way to install Python scripts in ROS.
# It makes them executable and places them where ROS can find them.
catkin_install_python(PROGRAMS
  scripts/robot_simulator.py
  scripts/bug_navigator.py
  scripts/check_fusion_quality.py # Also install the checker script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#-----------------------------------------------------------------------------
# Install Other Package Files
#-----------------------------------------------------------------------------
# We must install our launch, config, and rviz files so that tools
# like 'roslaunch' and 'rviz' can find them using $(find tethered_robot_navigation).
install(DIRECTORY
  launch/
  config/
  rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)