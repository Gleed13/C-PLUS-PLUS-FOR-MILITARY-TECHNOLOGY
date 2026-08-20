cmake_minimum_required(VERSION 3.20)
project(payload_action)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(underground_world REQUIRED)

add_executable(payload_action_node src/payload_action_node.cpp)
target_link_libraries(payload_action_node
  rclcpp::rclcpp
  ${underground_world_TARGETS}
)

install(TARGETS payload_action_node DESTINATION lib/${PROJECT_NAME})

ament_package()
