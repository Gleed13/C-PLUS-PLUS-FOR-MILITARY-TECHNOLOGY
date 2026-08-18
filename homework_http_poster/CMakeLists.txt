cmake_minimum_required(VERSION 3.20)

project(http_poster CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)

FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.18.5
)

set(HTTPLIB_REQUIRE_OPENSSL OFF CACHE BOOL "" FORCE)
set(HTTPLIB_COMPILE OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(json httplib)

find_package(Threads REQUIRED)

add_executable(http_poster src/main.cpp)

target_link_libraries(http_poster PRIVATE nlohmann_json::nlohmann_json httplib::httplib Threads::Threads)

target_compile_options(http_poster PRIVATE -Wall -Wextra -pedantic)
