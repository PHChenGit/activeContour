cmake_minimum_required(VERSION 3.15)
project(RvlCourse)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# include_directories("/usr/local/include/opencv4/opencv2/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(RvlCourse_hello rvl_hello.cpp)
target_link_libraries(RvlCourse_hello PUBLIC "/usr/local/lib")

add_executable(RvlCourse_cv show_cv.cpp)
target_link_libraries(RvlCourse_cv PUBLIC ${OpenCV_LIBS})
# target_link_libraries(RvlCourse ${OpenCV_LIBS})