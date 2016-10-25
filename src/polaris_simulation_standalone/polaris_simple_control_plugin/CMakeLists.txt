cmake_minimum_required(VERSION 2.8.3)
project(polaris_simple_control_plugin)

find_package(catkin REQUIRED COMPONENTS roscpp rospy angles std_srvs geometry_msgs sensor_msgs nav_msgs urdf tf dynamic_reconfigure driver_base)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS roscpp
    DEPENDS gazebo
)


add_library(polaris_simple_control_plugin src/polaris_simple_control_plugin.cpp)
target_link_libraries(polaris_simple_control_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".git" EXCLUDE)

install(DIRECTORY sdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.sdf"
  PATTERN ".git" EXCLUDE)