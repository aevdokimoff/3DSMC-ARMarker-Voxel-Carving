#pragma once
#include "../common/common.cpp"

struct Point_Cloud {
    u32 resolution;
    f32 side_length;
    Voxel* voxels;
};

Point_Cloud voxel_carve(u32 res, f32 side_length, const char* path_to_runs,
                        bool carve_in_parallel = false);
