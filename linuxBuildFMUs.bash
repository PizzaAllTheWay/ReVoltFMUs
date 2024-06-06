#!/bin/bash

# Check if the build directory exists
if [ -d "build" ]; then
    echo "build folder exists, clearing it..."
    # Clear the contents of the build directory
    rm -rf build/*
else
    # If not, create the build directory
    echo "Creating build directory..."
    mkdir build
fi

# Change to the build directory
cd build

# Run CMake to configure the project
cmake ..

# Build the project
cmake --build . --config Release
