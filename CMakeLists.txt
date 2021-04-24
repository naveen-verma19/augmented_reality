cmake_minimum_required(VERSION 3.17)
project(proj4)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /Users/naveen/Documents/Study/cv/opencv/build_opencv)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(proj4 main.cpp Drawer.cpp Drawer.h CameraCalibrate.cpp CameraCalibrate.h Shape.h)
target_link_libraries( proj4 ${OpenCV_LIBS})