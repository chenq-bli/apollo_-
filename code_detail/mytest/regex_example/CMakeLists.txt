# cmake_minimum_required(VERSION 3.0)

# project(regex_example)

# # 添加源代码文件
# add_executable(regex_example regex_example.cpp)

# # 指定头文件搜索路径
# target_include_directories(regex_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# # 指定链接 Boost 库
# target_link_libraries(regex_example boost_regex)
# set_target_properties(regex_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
# # 指定编译选项
# target_compile_options(regex_example PRIVATE -Wall -Wextra -pedantic)

cmake_minimum_required(VERSION 3.0)

project(regex_example)

# 添加源代码文件
# 查找 Boost 库
find_package(Boost REQUIRED COMPONENTS regex thread)

add_executable(regex_example regex_example.cpp)

# 指定头文件搜索路径
target_include_directories(regex_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})
set_target_properties(regex_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
# 指定链接 Boost 库
target_link_libraries(regex_example PRIVATE ${Boost_LIBRARIES})
# 指定编译选项
target_compile_options(regex_example PRIVATE -Wall -Wextra -pedantic)