cmake_minimum_required(VERSION 3.1.0)
project(sensor_clib)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package (GeographicLib REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        third_lib/Geographic/include
)

LINK_DIRECTORIES(third_lib/Geographic)
LINK_LIBRARIES(libGeographicc)


#add_executable(check_rtk_radar src/check_rtk_radar.cpp)
#target_link_libraries(check_rtk_radar libGeographicc ${OpenCV_LIBS})

add_executable(match_rtk_radar src/match_rtk_radar.cpp)
target_link_libraries(match_rtk_radar libGeographicc ${OpenCV_LIBS})
