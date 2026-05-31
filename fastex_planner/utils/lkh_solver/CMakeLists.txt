cmake_minimum_required(VERSION 3.0.2)
project(lkh_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SolveProblem.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lkh_solver
  CATKIN_DEPENDS message_runtime roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lkh_solver
  src/lkh_solver.cpp
)
add_dependencies(lkh_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lkh_solver
  ${catkin_LIBRARIES}
)

add_executable(lkh3_solver_node src/lkh3_solver_node.cpp)
add_dependencies(lkh3_solver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lkh3_solver_node
  lkh_solver
  ${catkin_LIBRARIES}
)
