cmake_minimum_required(VERSION 2.8.3)
project(mbot_simulation)

find_package(catkin REQUIRED
    roscpp
    angles
    control_toolbox
    controller_interface
    geometry_msgs
    hardware_interface
    kdl_parser
    nav_msgs
    pluginlib
    realtime_tools
    tf
    urdf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
    roscpp
    angles
    control_toolbox
    controller_interface
    geometry_msgs
    hardware_interface
    kdl_parser
    nav_msgs
    pluginlib
    realtime_tools
    tf
    urdf
)

### LIBS
add_library(steered_wheel_base_controller
  ros/src/steered_wheel_base_controller.cpp
)
target_link_libraries(steered_wheel_base_controller
  ${catkin_LIBRARIES}
)
