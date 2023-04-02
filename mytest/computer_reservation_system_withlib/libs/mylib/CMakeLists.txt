cmake_minimum_required(VERSION 3.0)
project(mylibpro)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(myfunlib src/manager.cpp src/orderfile.cpp src/student.cpp src/teacher.cpp)
