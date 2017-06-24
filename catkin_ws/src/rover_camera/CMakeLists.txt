cmake_minimum_required(VERSION 2.8.3)
project(rover_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

 add_service_files(
   FILES
   ChangeFeed.srv
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
)

catkin_install_python(
  PROGRAMS
  scripts/tf_setup.py
  scripts/feedSwitcher.py
  scripts/reduce_fps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
