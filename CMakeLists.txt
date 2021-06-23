cmake_minimum_required(VERSION 3.15)
project(3DSMC_ARMarker_Voxel_Carving)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
set(LIBRARIES  ${OpenCV_LIBS})

set(COMMON_SOURCES common/common.cpp common/volume.cpp)
include_directories(common)

add_executable(image_calibrator 01_data_acquisition/image_callibrator/main.cpp)
add_executable(linear_scale_generator 01_data_acquisition/linear_scale_generator/main.cpp)
add_executable(voxel_carving 02_voxel_carving/main.cpp ${COMMON_SOURCES})
add_executable(marching_cubes 03_marching_cubes/marching_cubes.cpp ${COMMON_SOURCES} 03_marching_cubes/simple_mesh.cpp 03_marching_cubes/simple_mesh.h 03_marching_cubes/marching_cubes.h)
add_executable(aruco_test aruco_markers_detection/opecv-test-2/test.cpp)
add_executable(aruco_markers_detection aruco_markers_detection/opecv-test-2/main.cpp)

target_link_libraries(image_calibrator ${LIBRARIES})
target_link_libraries(linear_scale_generator ${LIBRARIES})
target_link_libraries(voxel_carving ${LIBRARIES})
target_link_libraries(marching_cubes ${LIBRARIES})
target_link_libraries(aruco_test ${LIBRARIES})
target_link_libraries(aruco_markers_detection ${LIBRARIES})
