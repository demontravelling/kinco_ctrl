cmake_minimum_required(VERSION 3.0.2)
project(kinco_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  roscpp
  rospy
  tf
 
)
 
 
catkin_package(
  CATKIN_DEPENDS  
)

 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

add_executable(kinco_ctrl
  src/kinco_ctrl.cpp
)
add_dependencies(kinco_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinco_ctrl
  ${catkin_LIBRARIES}
)


 

