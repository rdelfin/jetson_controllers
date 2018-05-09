cmake_minimum_required(VERSION 2.8.3)
project(drive_train)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  jetson_control_msgs
  nav_msgs
  pca9685_driver
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs jetson_control_msgs nav_msgs pca9685_driver roscpp rospy sensor_msgs tf2 tf2_geometry_msgs tf2_ros
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

add_executable(dead_reckoning src/dead_reckoning.cpp)
add_dependencies(dead_reckoning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dead_reckoning
  ${catkin_LIBRARIES}
)

add_executable(speed_estimator src/speed_estimator.cpp)
add_dependencies(speed_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_estimator
  ${catkin_LIBRARIES}
)
