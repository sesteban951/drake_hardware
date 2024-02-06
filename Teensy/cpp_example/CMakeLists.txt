# specify the minimum version of CMake required to build the project
cmake_minimum_required(VERSION 3.1)

# project name
project(comms)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# executables
add_executable(s src/main.cpp src/SerialNode.cpp)
