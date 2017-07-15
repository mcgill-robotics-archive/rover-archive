cmake_minimum_required(VERSION 2.8.3)
project(ahrs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder

add_message_files(
  FILES
  AhrsStdMsg.msg
  Ig500nGPS.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CenterWorldFrame.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

if(NOT APPLE)

  find_package(Boost REQUIRED COMPONENTS system thread)

  include_directories(libraries/include/${PROJECT_NAME})
  include_directories(libraries/include/common)
  include_directories(vendors/sbgCom/src)

  add_subdirectory(vendors)
  add_subdirectory(libraries)

  add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
  )

  add_dependencies(${PROJECT_NAME}_node
    ${PROJECT_NAME}_generate_messages_cpp
  )

  target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_lib
    common
  )

  #############
  ## Install ##
  #############

  ## Mark executables and/or libraries for installation
  install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  catkin_install_python(PROGRAMS src/tf_broadcaster.py
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
