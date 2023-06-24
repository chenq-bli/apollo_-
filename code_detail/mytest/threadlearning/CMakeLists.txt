cmake_minimum_required(VERSION 3.0)

project(MySOLIDER)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE )

include_directories(${CMAKE_SOURCE_DIR}/libs/firstlib/include)
add_subdirectory(libs/firstlib)

add_executable(myexe src/main.cpp)
target_link_libraries(myexe 
myfunlib
pthread
)



