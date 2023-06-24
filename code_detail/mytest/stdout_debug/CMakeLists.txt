cmake_minimum_required(VERSION 3.0)

project(stdout_debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -DDEBUG")
#-D 选项可以在编译时定义一个宏,使用 -DDEBUG 选项可以定义一个名为 DEBUG 的宏

set(CMAKE_BUILD_TYPE BUG)

# find_package(Boost REQUIRED COMPONENTS system)

# catkin_package(
#   CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime
# )


include_directories(${CMAKE_SOURCE_DIR}/include

)

add_executable(${PROJECT_NAME}_node src/main.cpp src/StudentManager.cpp)
set_target_properties(${PROJECT_NAME}_node  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${PROJECT_NAME}_node  
pthread
mysqlclient
absl_strings
)
# add_dependencies(${PROJECT_NAME}_node def_gpsmsg_generate_messages_cpp)


