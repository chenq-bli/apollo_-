cmake_minimum_required(VERSION 3.0.2)
project(gps)

# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
sensor_msgs
geometry_msgs
serial

)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/gps_driver.cpp )
target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
pthread
)


