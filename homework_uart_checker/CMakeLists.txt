cmake_minimum_required(VERSION 3.20)

project(mission_processor CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)

FetchContent_MakeAvailable(json)

add_executable(mission_processor_full_cli src/main.cpp)

target_include_directories(mission_processor_full_cli PUBLIC include)
target_link_libraries(mission_processor_full_cli PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(mission_processor_full_cli PRIVATE -Wall -Wextra -pedantic)