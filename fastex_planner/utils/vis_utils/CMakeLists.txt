cmake_minimum_required(VERSION 3.0.2)
project(vis_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  visualization_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs std_msgs visualization_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)