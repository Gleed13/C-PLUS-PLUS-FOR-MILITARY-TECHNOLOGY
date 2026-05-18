cmake_minimum_required(VERSION 3.20)

project(mission_processor CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(mission_processor_cli src/main.cpp)
add_library(ballistics src/StrategyFactory.cpp)

target_include_directories(ballistics PUBLIC include)
target_link_libraries(mission_processor_cli PRIVATE ballistics)

target_compile_options(mission_processor_cli PRIVATE -Wall -Wextra -pedantic)