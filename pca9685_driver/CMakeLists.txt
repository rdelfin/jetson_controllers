cmake_minimum_required(VERSION 2.8.3)
project(pca9685_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  jetson_control_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy jetson_control_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pca9685_driver src/pca9685_driver.cpp
                              src/JHPWMPCA9685.cpp)
add_dependencies(pca9685_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pca9685_driver
  ${catkin_LIBRARIES}
)
