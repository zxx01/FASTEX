cmake_minimum_required(VERSION 3.0.2)
project(data_log_manager)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  std_srvs
  fastex_msgs
  file_utils
  time_utils
  process_utils
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES data_log_manager
 CATKIN_DEPENDS roscpp rospy roslib std_msgs nav_msgs std_srvs fastex_msgs file_utils time_utils process_utils
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(data_log_manager
  src/core/exploration_session_tracker.cpp
  src/logging/time_log_channel.cpp
  src/expl_data_manager.cpp
)
add_dependencies(data_log_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(data_log_manager
  ${catkin_LIBRARIES}
)

add_executable(data_log_manager_node
  src/data_log_manager_node.cpp
)
add_dependencies(data_log_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(data_log_manager_node
  ${catkin_LIBRARIES}
  data_log_manager
)

catkin_install_python(PROGRAMS
  scripts/plotting/OdomVisualizer.py
  scripts/plotting/realTimePlot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
