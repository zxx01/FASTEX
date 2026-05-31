cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  bspline
  plan_env
  path_searching
  poly_traj
  roscpp
  geometry_msgs
  visualization_msgs
  vis_utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils
 CATKIN_DEPENDS bspline plan_env path_searching poly_traj roscpp geometry_msgs visualization_msgs vis_utils
)

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library( traj_utils
    src/planning_visualization.cpp
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES}
    )
