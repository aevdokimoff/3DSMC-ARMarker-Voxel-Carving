#pragma once
#include "common.h"
#include "volume.h"
#include <functional>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>

using namespace cv;

Volume generate_point_cloud(u32 resolution, f32 side_length);
void voxel_carve(Volume *volume, const char *path_to_runs, bool carve_in_parallel, bool output_result_image);
void process_using_single_run(const char* run_path, Matx44d projection_mat,
                              bool carve_in_parallel,
                              const std::function<void (const char*, uint ind, Matx44d, Matx44d, s32)> &onProcess);
