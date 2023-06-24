cmake_minimum_required(VERSION 3.0.2)
project(def_gpsmsg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs

)

add_message_files(FILES gps.msg)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_talker src/talker.cpp)
add_dependencies(${PROJECT_NAME}_talker def_gpsmsg_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_talker
   ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_listener src/listener.cpp)
add_dependencies(${PROJECT_NAME}_listener def_gpsmsg_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_listener
   ${catkin_LIBRARIES}
)







