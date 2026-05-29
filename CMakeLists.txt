cmake_minimum_required(VERSION 3.16)
project(CANBusSimulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile options
add_compile_options(-Wall -Wextra -Wpedantic)

# Include directories
include_directories(include)

# Simulator library (shared between main and tests)
add_library(can_sim
    src/CANFrame.cpp
    src/CANBus.cpp
    src/ECUNode.cpp
    src/EngineECU.cpp
    src/ABSECU.cpp
    src/DashboardECU.cpp
    src/SignalParser.cpp
    src/FaultInjector.cpp
)
target_include_directories(can_sim PUBLIC include)

# Main executable
add_executable(can_simulator src/main.cpp)
target_link_libraries(can_simulator PRIVATE can_sim pthread)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(can_tests
    tests/test_canframe.cpp
    tests/test_canbus.cpp
    tests/test_signalparser.cpp
    tests/test_faultinjector.cpp
)
target_link_libraries(can_tests PRIVATE can_sim pthread GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(can_tests)
