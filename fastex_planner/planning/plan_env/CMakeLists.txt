cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  vis_utils
  message_filters
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp nav_msgs sensor_msgs visualization_msgs vis_utils message_filters pcl_conversions
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env
    src/sdf_map.cpp
    src/edt_environment.cpp
    src/raycast.cpp
    src/map_ros.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )
