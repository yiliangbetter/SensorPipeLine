cmake_minimum_required(VERSION 3.20)
project(SensorPipeline VERSION 0.1.0 LANGUAGES CXX)

# ============================================================================
# C++20 Required for Concepts and constexpr enhancements
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Generate compile_commands.json for IDE/tooling support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# C++ clang-tidy
set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
# ============================================================================
# Build Type Configuration
# ============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Release build flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Debug build flags
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")

# ============================================================================
# Compiler Warnings
# ============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wconversion
        -Wshadow
    )
elseif(MSVC)
    add_compile_options(/W4 /WX)
endif()

# ============================================================================
# Include Directories
# ============================================================================
include_directories(${PROJECT_SOURCE_DIR}/include)

# ============================================================================
# Phase 1: Type-Safe Units & Frames
# ============================================================================

# Example executable
add_executable(phase1_demo 
    examples/phase1_demo.cpp
)

# Compile-time tests
add_executable(compile_time_tests 
    tests/compile_time_tests.cpp
)

# ============================================================================
# Enable Testing
# ============================================================================
enable_testing()

add_test(NAME CompileTimeTests COMMAND compile_time_tests)
add_test(NAME Phase1Demo COMMAND phase1_demo)

# ============================================================================
# Installation (optional)
# ============================================================================
install(DIRECTORY include/ DESTINATION include/sensor_pipeline)
install(TARGETS phase1_demo compile_time_tests
        RUNTIME DESTINATION bin)

# ============================================================================
# Print Configuration
# ============================================================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "Sensor Pipeline - Configuration")
message(STATUS "========================================")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")
message(STATUS "")
