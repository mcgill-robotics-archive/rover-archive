cmake_minimum_required(VERSION 2.8.3)
project(arm_simulation)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros 
  pid
  roscpp
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  gazebo_ros 
  roscpp
)

#Build joint_states_dispatcher

add_executable(joint_states_dispatcher src/joint_states_dispatcher.cpp)
target_link_libraries(joint_states_dispatcher ${catkin_LIBRARIES})

install(TARGETS
  joint_states_dispatcher 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build our plugin
add_library(arm_control_plugin plugins/arm_control_plugin.cc)
target_link_libraries(arm_control_plugin ${Boost_libraries} ${GAZEBO_libraries} ${roscpp_libraries})

install(TARGETS
  arm_control_plugin 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

