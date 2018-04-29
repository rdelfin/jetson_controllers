cmake_minimum_required(VERSION 2.8.3)
project(drive_train)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  jetson_control_msgs
  pca9685_driver
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs sensor_msgs jetson_control_msgs pca9685_driver roscpp rospy tf2 tf2_ros
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_train_controller src/drive_train_controller.cpp)
add_dependencies(drive_train_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_train_controller
  ${catkin_LIBRARIES}
)

add_executable(joystick_converter src/joystick_converter.cpp)
add_dependencies(joystick_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_converter
  ${catkin_LIBRARIES}
)
