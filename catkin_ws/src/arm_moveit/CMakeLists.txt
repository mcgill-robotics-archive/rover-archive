cmake_minimum_required(VERSION 2.8.3)
project(arm_moveit)

find_package(catkin REQUIRED COMPONENTS
  arm_description
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  src/joints_as.py DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
