cmake_minimum_required(VERSION 3.15)
project(3DSMC_ARMarker_Voxel_Carving)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-O3 -fopenmp")
add_compile_options(-DOpenCV_DIR=3rd_party_libs/opencv3.4/include)

find_package(OpenCV REQUIRED)
set(LIBRARIES  ${OpenCV_LIBS})

set(COMMON_SOURCES common/common.cpp common/volume.cpp 02_voxel_carving/voxel_carving.cpp common/image.cpp)
include_directories(common)
include_directories(02_voxel_carving)

add_executable(image_calibrator 01_data_acquisition/image_callibrator/main.cpp)
add_executable(linear_scale_generator 01_data_acquisition/linear_scale_generator/main.cpp)
add_executable(voxel_carving 02_voxel_carving/main.cpp ${COMMON_SOURCES})
add_executable(marching_cubes 03_marching_cubes/simple_marching_cubes.cpp ${COMMON_SOURCES} 03_marching_cubes/simple_mesh.cpp 03_marching_cubes/main.cpp 03_marching_cubes/surface/implicit_surface.cpp 02_voxel_carving/voxel_carving.cpp 03_marching_cubes/projected_marching_cubes.cpp)
add_executable(texture_backprojection 04_texture_backprojection/backproject.cpp ${COMMON_SOURCES})
add_executable(aruco_test aruco_markers_detection/opecv-test-2/main.cpp)
add_executable(aruco_markers_detection aruco_markers_detection/opecv-test-2/main.cpp)

target_link_libraries(image_calibrator ${LIBRARIES})
target_link_libraries(linear_scale_generator ${LIBRARIES})
target_link_libraries(voxel_carving ${LIBRARIES})
target_link_libraries(marching_cubes ${LIBRARIES})
target_link_libraries(texture_backprojection ${LIBRARIES})
target_link_libraries(aruco_test ${LIBRARIES})
target_link_libraries(aruco_markers_detection ${LIBRARIES})
