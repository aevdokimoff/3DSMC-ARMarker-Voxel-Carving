#include <common.h>
#include "common.h"

v3 operator-(v3 a, v3 b) {
    return {
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
}

v3 operator+(v3 a, v3 b) {
    return {
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
}

v4 operator*(f32 f, v4 b) {
    return {
        .x = f * b.x,
        .y = f * b.y,
        .z = f * b.z,
        .w = f * b.w,        
    };
}

v3 operator*(f32 f, v3 b) {
    return {
        .x = f * b.x,
        .y = f * b.y,
        .z = f * b.z,
    };
}

v3 operator*(v3 b, f32 f) {
    return {
        .x = f * b.x,
        .y = f * b.y,
        .z = f * b.z,
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
        .x = dot(m.rows[0].v, vec),
        .y = dot(m.rows[1].v, vec),
        .z = dot(m.rows[2].v, vec),
    };
}

v4 operator*(mat4x4 m, v4 vec) {
    return {
        .x = dot(m.rows[0].v, vec),
        .y = dot(m.rows[1].v, vec),
        .z = dot(m.rows[2].v, vec),
        .w = dot(m.rows[3].v, vec),
    };
}

v3 normalize(v3 v) {
    f32 squared_len = dot(v, v);
    if (squared_len == 0)
        return { 0, 0, 0 };
    return 1.0/sqrt(squared_len) * v;
}

v3 cross(v3 b, v3 c) {
    v3 result{};

    result.x = b.y*c.z - c.y*b.z;
    result.y = c.x*b.z - b.x*c.z;
    result.z = b.x*c.y - c.x*b.y;

    return result;
}
