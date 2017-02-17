cmake_minimum_required(VERSION 2.8.3)
project(mbot_simulation)

find_package(catkin REQUIRED
    roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS
    roscpp
)
