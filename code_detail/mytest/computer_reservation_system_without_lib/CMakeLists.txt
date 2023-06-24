cmake_minimum_required(VERSION 3.0)

project(MySOLIDER)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE )
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

include_directories(${CMAKE_SOURCE_DIR}/include)



add_executable(myexe src/main.cpp src/manager.cpp src/orderfile.cpp src/student.cpp src/teacher.cpp)

#或者set_target_properties(myexe PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#${CMAKE_CURRENT_SOURCE_DIR}表示当前处理的 CMakeLists.txt 文件所在的目录的绝对路径。



