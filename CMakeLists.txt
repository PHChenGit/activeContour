cmake_minimum_required(VERSION 3.15)
project(RvlCourse)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# include_directories("/usr/local/include/opencv4/opencv2/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(active_contour active_contour.cpp)
target_link_libraries(active_contour PUBLIC ${OpenCV_LIBS} PUBLIC "/usr/local/lib")
