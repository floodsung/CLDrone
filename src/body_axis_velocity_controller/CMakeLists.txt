cmake_minimum_required(VERSION 2.8.3)
project(body_axis_velocity_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_extras
  geometry_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/
)

add_executable(body_axis_velocity_controller_node src/body_axis_velocity_controller_node.cpp)

target_link_libraries(body_axis_velocity_controller_node
   ${catkin_LIBRARIES}
 )

