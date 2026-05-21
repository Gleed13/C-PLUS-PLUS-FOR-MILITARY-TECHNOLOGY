cmake_minimum_required(VERSION 3.20)

project(telemetry_check_project CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(telemetry_check src/main.cpp)
add_library(telemetry STATIC src/telemetry.cpp)

target_include_directories(telemetry PUBLIC include)
target_link_libraries(telemetry_check PRIVATE telemetry)

target_compile_options(telemetry_check PRIVATE -Wall -Wextra -pedantic)