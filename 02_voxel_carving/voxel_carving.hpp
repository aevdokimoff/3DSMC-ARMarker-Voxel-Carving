#pragma once
#include "common.h"
#include "volume.h"
#include <functional>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>

#define IDX3D(x,y,z, res) ((z*res*res)+(y*res)+x)
#define IDX2D(x,y, res)   ((y*res)+x)

using namespace cv;

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

Volume<bool> generate_point_cloud(u32 resolution, f32 side_length);
void voxel_carve(Volume<bool> *volume, const char* path_to_runs, bool carve_in_parallel = false, bool output_result_image = false);
void process_using_single_run(const char* run_path, Matx44d projection_mat,
                              bool carve_in_parallel, const std::function<void (const char*, Matx44d, Matx44d)> &onProcess);
Matx44d getProjectionMatrix();
Vec3d project_point_to_screen_space(Vec3d pos, Matx44d extrinsic, Matx44d intrinsic);
Vec3d project_screen_point_to_3d(Vec3d pos, const Matx44d extrinsic, const Matx44d intrinsic);