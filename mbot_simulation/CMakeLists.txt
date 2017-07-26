cmake_minimum_required(VERSION 2.8.3)
project(mbot_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(mbot_head_plugin SHARED ros/src/mbot_head_plugin.cc)
target_link_libraries(mbot_head_plugin ${GAZEBO_libraries})
