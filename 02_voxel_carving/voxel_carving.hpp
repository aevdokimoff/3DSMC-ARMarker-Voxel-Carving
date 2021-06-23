#pragma once
#include <stdint.h>

typedef float     f32;
typedef double    f64;
typedef uint8_t    u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t     s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

struct v3 {
    f32 x;
    f32 y;
    f32 z;
};


struct Voxel {
    v3 position;
    f32 value;
};


struct Point_Cloud {
    u32 resolution;
    f32 side_length;
    Voxel* voxels;
};

Point_Cloud voxel_carve(u32 res, f32 side_length, const char* path_to_runs,
                        bool carve_in_parallel = false);
