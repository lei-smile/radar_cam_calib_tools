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

#add_library(cam2RTK_calib SHARED src/cam2RTK_calib.cpp)
#add_dependencies(cam2RTK_calib  ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(cam2RTK_calib src/cam2RTK_calib.cpp)
target_link_libraries(cam2RTK_calib libGeographicc ${OpenCV_LIBS})

add_executable(radar_static_calib_yaw src/radar_static_calib_yaw.cpp)
target_link_libraries(radar_static_calib_yaw ${GeographicLib_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable (radar_reproject src/radar_reproject.cpp)
target_link_libraries (radar_reproject ${OpenCV_LIBS})

add_executable (radar_calib_check src/radar_calib_check.cpp)
target_link_libraries (radar_calib_check ${OpenCV_LIBS})


#add_executable (radar_select src/radar_select.cpp)
#target_link_libraries (radar_select ${OpenCV_LIBS})

#add_executable(getImagePixel src/getImagePixel.cpp)
#target_link_libraries(getImagePixel  ${OpenCV_LIBS})
