cmake_minimum_required(VERSION 2.8.3)
project(ahrs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(lib/include/ahrs)
include_directories(lib/include/common)
include_directories(vendors/sbgCom/src)

add_subdirectory(vendors)
add_subdirectory(lib)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AhrsStatusMessage.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs  
  sensor_msgs  
  std_msgs
)

catkin_package()

###########
## Build ##
###########

file(GLOB AHRS_NODE_SOURCES src/[^.]*.cpp)
file(GLOB AHRS_NODE_PRIVATE_HEADERS src/[^.]*.h)

add_executable(ahrs_node
    ${AHRS_NODE_SOURCES}
    ${AHRS_NODE_PRIVATE_HEADERS}
)

add_dependencies(ahrs_node 
    ahrs_generate_messages_cpp
)

target_link_libraries(
    ahrs_node
    ${catkin_LIBRARIES}
    ahrs_lib
    common
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ahrs_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

