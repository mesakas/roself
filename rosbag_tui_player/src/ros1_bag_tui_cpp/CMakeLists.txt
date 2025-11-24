cmake_minimum_required(VERSION 3.0.2)
project(rosbag_tui_cpp)

## 开启 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# 或者你更喜欢的话用这一句也可以（不用上面三句）：
# add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
)

find_package(Curses REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)

add_executable(ros1_bag_tui_player src/main.cpp)
target_link_libraries(ros1_bag_tui_player
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

# 输出到 catkin_ws/bin
set_target_properties(ros1_bag_tui_player PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY         ${CMAKE_BINARY_DIR}/../bin
  RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_BINARY_DIR}/../bin
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../bin
)

install(TARGETS ros1_bag_tui_player
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
