cmake_minimum_required(VERSION 3.0)

project(template_example)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE Debug)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_node src/main.cpp src/template_vec.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")


