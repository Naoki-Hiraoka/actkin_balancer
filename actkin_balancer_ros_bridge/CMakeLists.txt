cmake_minimum_required(VERSION 3.0.2)
project(actkin_balancer_ros_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED rtmbuild2)

rtmbuild2_init()

catkin_package(
)

add_subdirectory(rtc)
