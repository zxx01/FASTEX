cmake_minimum_required(VERSION 2.8.3)
project(bspline)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)
add_message_files(
  FILES
  Bspline.msg
  )

add_service_files(
  FILES
  BsplineProcess.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library( bspline
    src/non_uniform_bspline.cpp
    )
target_link_libraries( bspline
    ${catkin_LIBRARIES}
    )
