# Set the minimum required version of cmake
cmake_minimum_required(VERSION 3.0.2)

# Define the project name
project(paper_nav_experiment)

## Find catkin and all required ROS packages (dependencies)
## This list MUST match the dependencies in your package.xml
find_package(catkin REQUIRED COMPONENTS
  amcl
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## Catkin Package configuration
## This is necessary to specify build-time and run-time dependencies.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES paper_nav_experiment
  CATKIN_DEPENDS amcl geometry_msgs move_base nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

## Specify where to find header files (C++)
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## The section below is for compiling C++ nodes.
## Since this package currently only uses launch files and configuration,
## there are no executables to build. If you add a C++ source file
## in a 'src' directory, you would uncomment and modify the lines below.
#
# add_executable(your_cpp_node_name src/your_cpp_node_name.cpp)
#
# add_dependencies(your_cpp_node_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
# target_link_libraries(your_cpp_node_name
#   ${catkin_LIBRARIES}
# )


## Install all necessary files to the package's share directory
## This is CRITICAL for roslaunch to find your files.

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install parameter files
install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

# Install rviz configuration files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Install Gazebo world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

## Note: If you add Python nodes in a 'nodes' or 'scripts' directory,
## you would add an install rule for them like this:
#
# install(PROGRAMS
#   scripts/your_python_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )