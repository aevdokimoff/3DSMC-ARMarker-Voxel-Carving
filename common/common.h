#ifndef INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H
#define INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H

#include <cmath>
#include <cstdint>
#include <opencv2/core/mat.hpp>

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

struct v3 {
    f32 x;
    f32 y;
    f32 z;

    v3() = default;

    v3(f32 _x, f32 _y, f32 _z) : x(_x), y(_y), z(_z) {}

    explicit v3(cv::Vec3d _vector) {
        x = _vector[0];
        y = _vector[1];
        z = _vector[2];
    }

    cv::Vec3d toVec3d();
};

struct v4 {
    f32 x;
    f32 y;
    f32 z;
    f32 w;

    v4() = default;

    v4(f32 _x, f32 _y, f32 _z, f32 _w) : x(_x), y(_y), z(_z), w(_w) {}

    explicit v4(cv::Vec4d _vector) {
        x = _vector[0];
        y = _vector[1];
        z = _vector[2];
        w = _vector[3];
    }

    cv::Vec4d toVec4d();
};

union mat4x4 {
    struct {
        f32 values[16];
    };
    union {
        f32 cols[4];
        v4  v;
    } rows[4];

    mat4x4() = default;
};

union mat3x3 {
    struct {
        f32 values[9];
    };
    union {
        f32 cols[3];
        v3  v;
    } rows[3];
};

v3 operator-(v3 a, v3 b);

v3 operator+(v3 a, v3 b);

v4 operator*(f32 f, v4 b);

v3 operator*(f32 f, v3 b);

v3 operator*(v3 b, f32 f);

f32 dot(v3 v1, v3 v2);

f32 dot(v4 v1, v4 v2);

v3 operator*(mat3x3 m, v3 vec);

v4 operator*(mat4x4 m, v4 vec);

v3 normalize(v3 v);

v3 cross(v3 b, v3 c);

mat4x4 generate_identity_4x4();

mat3x3 generate_z_rot_mat(f32 angle);

v4 to_v4(v3 v);

#endif