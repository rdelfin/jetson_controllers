cmake_minimum_required(VERSION 2.8.3)
project(jetson_robot_model)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
