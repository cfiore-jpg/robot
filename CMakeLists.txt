cmake_minimum_required(VERSION 3.24)

project(RobotNavigation)

set(CMAKE_CXX_STANDARD 17)

add_library(RobotNavigation SHARED src/Map.cpp src/Object.cpp src/Robot.cpp)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RobotNavigation ${OpenCV_LIBS})
