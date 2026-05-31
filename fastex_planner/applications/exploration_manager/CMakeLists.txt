cmake_minimum_required(VERSION 3.0.2)
project(exploration_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  plan_env
  path_searching
  bspline
  traj_utils
  plan_manage
  file_utils
  time_utils
  process_utils
  map_process
  vis_utils
  lkh_solver
  fastex_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs std_srvs plan_env path_searching bspline traj_utils plan_manage file_utils time_utils process_utils map_process
  vis_utils lkh_solver fastex_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fsm/fast_exploration_fsm.cpp
  src/runtime/fast_exploration_runtime_adapter.cpp
  src/manager/fastex_exploration_manager.cpp
  src/planners/global_coverage_planner.cpp
  src/planners/lkh_interface.cpp
  src/planners/local_frontier_planner.cpp
  src/planners/local_viewpoint_refiner.cpp
  )
target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
  )
