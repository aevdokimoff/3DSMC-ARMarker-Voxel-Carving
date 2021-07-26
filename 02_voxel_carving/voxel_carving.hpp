#pragma once
#include "common.h"
#include "volume.h"
#include <functional>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>

using namespace cv;

/**
 * Generates regular square 3D point grid.
 * @param resolution -- number of spaces between points (number of points - 1).
 * @param side_length -- length of the grid in one dimension.
 */
Volume generate_point_cloud(u32 resolution, f32 side_length);

/**
 * Performs voxel carving algorithm on the given 3D grid.
 * @param path_to_runs -- path to the root folder with 2D images.
 * @param carve_in_parallel -- boolean indicating should computations be parallelized or not.
 * @param output_result_image -- debug option, should the resulting image be saved.
 */
void voxel_carve(Volume *volume, const char *path_to_runs, bool carve_in_parallel, bool output_result_image);

/**
 * Performs voxel carving based on one series of 2d images.
 * @param run_path -- path to the folder with a series of 2D images.
 * @param carve_in_parallel -- boolean indicating should computations be parallelized or not.
 * @param onProcess -- a callback with processing of one 2d image.
 */
void process_using_single_run(const char* run_path, Matx44d projection_mat,
                              bool carve_in_parallel,
                              const std::function<void (const char*, uint ind, Matx44d, Matx44d, s32)> &onProcess);
