#ifndef INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H
#define INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H

#include <cmath>
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include "../3rd_party_libs/ftb/types.hpp" // for the short types
#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062

struct v2 {
    f32 x;
    f32 y;
};

struct V2_Triangle {
    v2 a;
    v2 b;
    v2 c;
};

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
v3 operator*(mat3x3 m, v3 vec);
v4 operator*(mat4x4 m, v4 vec);

f32 dot(v3 v1, v3 v2);
f32 dot(v4 v1, v4 v2);
v3 normalize(v3 v);
v3 cross(v3 b, v3 c);

mat4x4 generate_identity_4x4();
mat3x3 generate_z_rot_mat(f32 angle);
mat4x4 generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar);
mat4x4 generate_our_proj_mat();
mat4x4 generate_look_at_mat(v3 eye, v3 center, v3 up);
mat4x4 generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation);
v2 project_point_to_screen_space(v3 pos, mat4x4 camera_mat, mat4x4 proj_mat);

v4 to_v4(v3 v);
void print(mat4x4 m);

v3 get_cam_pos_for_run(const char* run_path);

#endif
