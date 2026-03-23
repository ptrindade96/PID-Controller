cmake_minimum_required(VERSION 3.10)

# Project name and language
project(PIDController LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the library
add_library(PIDController src/PIDController.cpp)

# Include directories for library consumers
target_include_directories(
    PIDController
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Optionally, add compiler warnings
target_compile_options(PIDController
    PRIVATE
        -Wall -Wextra -Wpedantic
)

# Add examples
add_executable(example examples/example.cpp)

# Link example with PIDController library
target_link_libraries(example PRIVATE PIDController)