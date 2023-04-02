cmake_minimum_required(VERSION 3.0)

project(MySOLIDER)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE )

include_directories(${CMAKE_SOURCE_DIR}/include)


add_executable(myexe src/main.cpp src/manager.cpp src/orderfile.cpp src/student.cpp src/teacher.cpp)




