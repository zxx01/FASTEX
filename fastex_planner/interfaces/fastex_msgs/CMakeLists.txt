cmake_minimum_required(VERSION 3.0.2)
project(fastex_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  DataLog.msg
  ExploredVolumeTravedDistTime.msg
  IterationTime.msg
  frontierStatistics.msg
)

add_service_files(
  FILES
  DataLogProcess.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
