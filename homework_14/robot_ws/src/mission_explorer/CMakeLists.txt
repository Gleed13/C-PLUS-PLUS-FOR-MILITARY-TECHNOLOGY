cmake_minimum_required(VERSION 3.20)
project(mission_explorer)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(underground_world REQUIRED)

# Чиста логіка карти без ROS-типів - її можна тестувати без ROS-рантайму.
add_library(map_memory src/map_memory.cpp)
target_include_directories(map_memory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_memory ${underground_world_TARGETS})

add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_link_libraries(mission_explorer_node
  map_memory
  rclcpp::rclcpp
  ${underground_world_TARGETS}
)

install(TARGETS mission_explorer_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS map_memory DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(map_memory_test test/map_memory_test.cpp)
  target_link_libraries(map_memory_test map_memory)
endif()

ament_package()
