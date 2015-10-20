cmake_minimum_required(VERSION 2.8.3)
project(ahrs)

if (NOT APPLE)
    find_package(catkin REQUIRED COMPONENTS
      geometry_msgs
      sensor_msgs
      std_msgs
      message_generation
      roscpp
      tf
    )

    include_directories(
      ${catkin_INCLUDE_DIRS}
    )
endif()

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(libraries/include/ahrs)
include_directories(libraries/include/common)
include_directories(vendors/sbgCom/src)

add_subdirectory(vendors)
add_subdirectory(libraries)

if(NOT APPLE)
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

    catkin_package()
endif()

###########
## Build ##
###########

add_executable(ahrs_node
    src/ahrs_node.cpp
)

if (NOT APPLE)
    add_dependencies(ahrs_node
        ahrs_generate_messages_cpp
    )
endif()

target_link_libraries(
    ahrs_node
    ${catkin_LIBRARIES}
    ahrs_lib
    common
)

if(NOT APPLE)
    #############
    ## Install ##
    #############

    ## Mark executables and/or libraries for installation
    install(TARGETS ahrs_node 
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    #############
    ## Testing ##
    #############
    
    catkin_add_gtest(utest test/ahrs_unit_test.cpp)
    target_link_libraries(utest 
        ${catkin_LIBRARIES} 
        ${BOOST_THREAD_LIBRARIES} 
        ahrs_lib)

endif()
