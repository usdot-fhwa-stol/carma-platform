cmake_minimum_required(VERSION 3.0.2)
project(guidance_plugin_validator)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  cav_msgs
  rosnode
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy cav_msgs rosnode std_msgs
)

catkin_install_python(PROGRAMS src/guidance_plugin_validator/main.py src/guidance_plugin_validator/guidance_plugin_validator.py src/guidance_plugin_validator/guidance_plugin_components.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)