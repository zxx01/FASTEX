cmake_minimum_required(VERSION 3.0.2)
project(test_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  quadrotor_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs quadrotor_msgs roscpp std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
