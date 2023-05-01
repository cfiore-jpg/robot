cmake_minimum_required(VERSION 3.24)

project(RobotNavigation)

set(CMAKE_CXX_STANDARD 17)

add_library(RobotNavigation SHARED src/Map.cpp src/Robot.cpp)
