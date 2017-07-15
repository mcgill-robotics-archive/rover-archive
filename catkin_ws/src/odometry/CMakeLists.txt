cmake_minimum_required(VERSION 2.8.3)
project(odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  message_runtime
  nav_msgs
  robot_localization
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odometry
  CATKIN_DEPENDS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
#  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geographic_msgs geometry_msgs message_filters message_generation message_runtime nav_msgs roscpp roslint sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
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

## Mark executables and/or libraries for installation
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_odometry.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
catkin_install_python(PROGRAMS
                      scripts/ahrs_manager.py
                      scripts/wheel_odom.py
                      scripts/wheel_odom_test.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
