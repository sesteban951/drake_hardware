cmake_minimum_required(VERSION 3.1)

project(comms)

add_executable(s src/server.cpp)
add_executable(c src/client.cpp)
