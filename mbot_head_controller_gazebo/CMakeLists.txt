cmake_minimum_required(VERSION 2.8.3)
project(mbot_head_controller_gazebo)

find_package(catkin REQUIRED
	roscpp
	std_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 CATKIN_DEPENDS
    roscpp
)

include_directories(/usr/include/)
add_executable(mbot_head_pan_controller ros/src/mbot_head_pan_controller.cpp)
target_link_libraries(mbot_head_pan_controller ${catkin_LIBRARIES})
