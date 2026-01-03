#!/bin/bash
# Set default build type to Release
BUILD_TYPE="Release"

# Check for argument to set build type
if [[ "$1" == "debug" || "$1" == "Debug" ]]; then
    BUILD_TYPE="Debug"
fi
# Exit immediately if a command exits with a non-zero status
set -e

# Set build directory
BUILD_DIR="build"

# Create build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run CMake to configure the project
cmake -S .. -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=$BUILD_TYPE

# Build the project using all available CPU cores
cmake --build .  -- -j"$(nproc || sysctl -n hw.ncpu)"

echo "Build completed successfully."

