cmake_minimum_required(VERSION 2.8.3)
project(jetson_launch)
find_package(catkin REQUIRED)
catkin_package()

## Mark cpp header files for installation
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKING_PACKAGE_SHARE_DESTINATION}/launch)

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)

    #roslaunch_add_file_check(launch/whatever.launch)
endif()
