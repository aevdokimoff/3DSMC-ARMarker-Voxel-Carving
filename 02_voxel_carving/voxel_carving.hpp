#pragma once
#include "common.h"
#include "volume.h"
#include <functional>

#define IDX3D(x,y,z, res) ((z*res*res)+(y*res)+x)
#define IDX2D(x,y, res)   ((y*res)+x)

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

Volume<bool> generate_point_cloud(u32 resolution, f32 side_length);
void voxel_carve(Volume<bool> *volume, const char* path_to_runs, bool carve_in_parallel = false, bool output_result_image = false);
void process_using_single_run(const char* run_path, mat4x4 projection_mat,
                              bool carve_in_parallel, const std::function<void (const char*, mat4x4, mat4x4)>& onProcess);
mat4x4 getProjectionMatrix();
v3 project_point_to_screen_space(v3 pos, mat4x4 extrinsic, mat4x4 intrinsic);