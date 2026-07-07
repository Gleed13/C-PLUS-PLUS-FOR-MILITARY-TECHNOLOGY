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

add_executable(uart_checker src/main.cpp
    src/abstractions/BackgroundService.cpp
    src/strategies/StrategyFactory.cpp
    src/strategies/FileConfigLoader.cpp
    src/strategies/JsonConfigLoader.cpp
    src/strategies/UartTargetProvider.cpp
    src/strategies/AnalyticalBallisticSolver.cpp
    src/strategies/TableBallisticSolver.cpp
    src/strategies/MockCheckerController.cpp
    src/strategies/GpioController.cpp
    src/strategies/ThreadSafeTargetProvider.cpp
    src/features/DroneMotionProfile.cpp
    src/features/DroneMovementStates.cpp
    src/features/TrajectoryEstimator.cpp
    src/features/TargetSelector.cpp
    src/features/SimulationRecorder.cpp
    src/features/UartBridge.cpp
    src/features/MissionProcessor.cpp
    src/utils/ControlUtils.cpp
    src/utils/UartUtils.cpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIODCXX REQUIRED IMPORTED_TARGET libgpiodcxx)

target_include_directories(uart_checker PUBLIC include)
target_link_libraries(uart_checker PRIVATE PkgConfig::GPIODCXX)
target_link_libraries(uart_checker PRIVATE nlohmann_json::nlohmann_json)

target_compile_options(uart_checker PRIVATE -Wall -Wextra -pedantic)