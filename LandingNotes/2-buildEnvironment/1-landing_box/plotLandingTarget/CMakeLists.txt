cmake_minimum_required(VERSION 3.14)
project(plotLandingTarget)
set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR "D:\\OPENCV\\mingGw-build\\install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#include_directories(D:\\OPENCV\\opencv-4.1.0-win64\\opencv\\build\\include)
#include_directories(D:\\OPENCV\\opencv-4.1.0-win64\\opencv\\build\\include\\opencv)
#include_directories(D:\\OPENCV\\opencv-4.1.0-win64\\opencv\\build\\include\\opencv2)

add_executable(plotLandingTarget main.cpp)
target_link_libraries(plotLandingTarget ${OpenCV_LIBS})