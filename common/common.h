#ifndef INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H
#define INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H

#include <cmath>
#include <cstdint>
#include <opencv2/core/mat.hpp>

using namespace cv;

typedef float  f32;
typedef double f64;
typedef uint8_t    u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t     s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

Matx33d generate_z_rot_mat(f32 angle);

#endif