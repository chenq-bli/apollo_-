cmake_minimum_required(VERSION 3.0.2)
project(gps_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  serial
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/gps_driver_parse.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${EIGEN3_LIBRARIES}
)