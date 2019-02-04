cmake_minimum_required(VERSION 2.8)
project(fisheye_undistort)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


message(OpenCV:${OpenCV_INCLUDE_DIRS})

include_directories(include)

#set(HEADER_FILES include/undistorter.hpp)

#set(SRC_FILES src/undistorter.cpp)


#add_executable(fisheye_undistort main.cpp ${SRC_FILES})
add_executable(fisheye_undistort main.cpp)
target_link_libraries(fisheye_undistort ${OpenCV_LIBS})
add_executable(pinhole_undistort pinhole.cpp)
target_link_libraries(pinhole_undistort ${OpenCV_LIBS})


