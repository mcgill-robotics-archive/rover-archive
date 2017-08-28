cmake_minimum_required(VERSION 2.8.3)
project(arm_ros_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  effort_controllers
  gazebo_ros_control
  joint_state_controller
  pid
  position_controllers
  roscpp
  rqt_joint_trajectory_controller
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joint_states_dispatcher src/joint_states_dispatcher.cpp)
target_link_libraries(joint_states_dispatcher ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS joint_states_dispatcher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
