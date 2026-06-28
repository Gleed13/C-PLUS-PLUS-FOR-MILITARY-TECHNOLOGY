cmake_minimum_required(VERSION 3.20)

project(mission_processor CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(uart_checker src/main.cpp
    src/features/uart_opener.cpp)

target_include_directories(uart_checker PUBLIC include)

target_compile_options(uart_checker PRIVATE -Wall -Wextra -pedantic)