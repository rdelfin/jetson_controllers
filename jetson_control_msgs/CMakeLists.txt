cmake_minimum_required(VERSION 2.8.3)
project(jetson_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
