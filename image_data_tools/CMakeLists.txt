cmake_minimum_required(VERSION 3.1.0)
project(sensor_clib)

find_package(OpenCV 3 REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(getImagePixel src/getImagePixel.cpp)
target_link_libraries(getImagePixel  ${OpenCV_LIBS})
