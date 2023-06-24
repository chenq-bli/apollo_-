cmake_minimum_required(VERSION 3.0)

project(MySOLIDER)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_BUILD_TYPE BUG)

# find_package(Boost REQUIRED COMPONENTS system)

# catkin_package(
#   CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime
# )

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_myexe  src/main.cpp src/myfun.cpp)
# set_target_properties(${PROJECT_NAME}_node  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
target_link_libraries(${PROJECT_NAME}_myexe 
pthread
)
# add_dependencies(${PROJECT_NAME}_node def_gpsmsg_generate_messages_cpp)


add_executable(myeigen src/eigentest.cpp)
add_executable(mycoordinate src/coordinate_transformation.cpp)
add_executable(myfile_operate src/file_operate.cpp)


