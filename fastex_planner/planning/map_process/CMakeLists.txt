cmake_minimum_required(VERSION 3.0.2)
project(map_process)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  file_utils
  time_utils vis_utils
  process_utils
  vdb_utils
  plan_env
  path_searching
  traj_utils
  fastex_msgs
)

find_package(PCL 1.10 REQUIRED)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found, enabling parallel frontier processing")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
else()
    message(FATAL_ERROR "OpenMP not found, but it is required for parallel processing")
endif()

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    message(STATUS "Eigen3 found: ${EIGEN3_VERSION}")
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_process
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs sensor_msgs nav_msgs message_filters file_utils time_utils vis_utils process_utils vdb_utils plan_env path_searching traj_utils fastex_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_library(map_process
  src/map_process/grid/single_level_grid.cpp
  src/map_process/grid/dynamic_expanding_grid_core.cpp
  src/map_process/grid/dynamic_expanding_grid_graph.cpp
  src/map_process/grid/dynamic_expanding_grid_cost.cpp
  src/map_process/searcher/path_searcher.cpp
  src/map_process/roadmap/whole_state_road_map_core.cpp
  src/map_process/roadmap/whole_state_road_map_graph.cpp
  src/map_process/roadmap/whole_state_road_map_search.cpp
  src/map_process/frontier/frontier_manager.cpp
  src/map_process/frontier/frontier_manager_roadmap.cpp
  src/map_process/frontier/frontier_manager_viewpoints.cpp
  src/map_process/frontier/frontier_manager_visualization.cpp
  src/map_process/core/sensor_cloud_buffer.cpp
  src/map_process/core/map_process.cpp
  src/utils/ikd_Tree.cpp
)
add_dependencies(map_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_process
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)
