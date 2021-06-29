#include "common.h"
#include <cstring>

cv::Vec3d v3::toVec3d() {
        return cv::Vec3d(x, y, z);
};

cv::Vec4d v4::toVec4d() {
    return cv::Vec4d(x, y, z, w);
};

v3 operator-(v3 a, v3 b) {
    return {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z,
    };
}

v3 operator+(v3 a, v3 b) {
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z,
    };
}

v4 operator*(f32 f, v4 b) {
    return {
        f * b.x,
        f * b.y,
        f * b.z,
        f * b.w,
    };
}

v3 operator*(f32 f, v3 b) {
    return {
        f * b.x,
        f * b.y,
        f * b.z,
    };
}

v3 operator*(v3 b, f32 f) {
    return {
        f * b.x,
        f * b.y,
        f * b.z,
    };
}

f32 dot(v3 v1, v3 v2) {
    return v1.x * v2.x +
           v1.y * v2.y +
           v1.z * v2.z;
}

f32 dot(v4 v1, v4 v2) {
    return v1.x * v2.x +
           v1.y * v2.y +
           v1.z * v2.z +
           v1.w * v2.w;
}

v3 operator*(mat3x3 m, v3 vec) {
    return {
        dot(m.rows[0].v, vec),
        dot(m.rows[1].v, vec),
        dot(m.rows[2].v, vec),
    };
}

v4 operator*(mat4x4 m, v4 vec) {
    return {
        dot(m.rows[0].v, vec),
        dot(m.rows[1].v, vec),
        dot(m.rows[2].v, vec),
        dot(m.rows[3].v, vec),
    };
}

v3 normalize(v3 v) {
    f32 squared_len = dot(v, v);
    if (squared_len == 0)
        return { 0, 0, 0 };
    return 1.0/sqrt(squared_len) * v;
}

v3 cross(v3 b, v3 c) {
    return {
        b.y*c.z - c.y*b.z,
        c.x*b.z - b.x*c.z,
        b.x*c.y - c.x*b.y
    };
}

mat4x4 generate_identity_4x4() {
    mat4x4 mat;
    memset(&mat, 0, sizeof(mat));
    mat.rows[0].v.x = 1;
    mat.rows[1].v.y = 1;
    mat.rows[2].v.z = 1;
    mat.rows[3].v.w = 1;

    return mat;
}

mat3x3 generate_z_rot_mat(f32 angle) {
    mat3x3 mat;
    memset(&mat, 0, sizeof(mat));

    mat.rows[0].cols[0] =  cos(angle);
    mat.rows[0].cols[1] = -sin(angle);
    mat.rows[1].cols[0] =  sin(angle);
    mat.rows[1].cols[1] =  cos(angle);

    mat.rows[2].cols[2] = 1;

    return mat;

}

v4 to_v4(v3 v) {
    return {
        v.x, v.y, v.z, 1
    };
}