cmake_minimum_required(VERSION 3.0)

project(MySOLIDER)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE )

include_directories(${CMAKE_SOURCE_DIR}/libs/mylib/include)
add_subdirectory(libs/mylib)

add_executable(myexe src/main.cpp)
target_link_libraries(myexe 
myfunlib
)



