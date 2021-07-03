#include "common.h"
#include <cstring>
#include <cstdio>

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

mat4x4 generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar) {
    // assert(abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

    f32 const tanHalfFovy = tan(fovy / 2.0f);

    mat4x4 result;
    memset(&result, 0, sizeof(result));

    result.rows[0].cols[0] = 1.0 / (aspect * tanHalfFovy);
    result.rows[1].cols[1] = 1.0 / (tanHalfFovy);
    result.rows[2].cols[2] = zFar / (zNear - zFar);
    result.rows[3].cols[2] = -1.0;
    result.rows[2].cols[3] = -(zFar * zNear) / (zFar - zNear);
    return result;
}

mat4x4 generate_our_proj_mat() {
    f32 y_fov = 0.3503711;
    f32 aspect = 1.509804;
    mat4x4 proj_mat = generate_proj_mat(y_fov, aspect, 0.3, 200);
    return proj_mat;
}

mat4x4 generate_look_at_mat(v3 eye, v3 center, v3 up) {
    v3 f = normalize(eye - center);
    v3 s = normalize(cross(f, up));
    v3 u = cross(s, f);

    mat4x4 result = generate_identity_4x4();
    result.rows[0].cols[0] = s.x;
    result.rows[0].cols[1] = s.y;
    result.rows[0].cols[2] = s.z;
    result.rows[1].cols[0] = u.x;
    result.rows[1].cols[1] = u.y;
    result.rows[1].cols[2] = u.z;
    result.rows[2].cols[0] =-f.x;
    result.rows[2].cols[1] =-f.y;
    result.rows[2].cols[2] =-f.z;
    result.rows[0].cols[3] =-dot(s, eye);
    result.rows[1].cols[3] =-dot(u, eye);
    result.rows[2].cols[3] = dot(f, eye);

    return result;
}

mat4x4 generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation) {
    obj_rotation = obj_rotation * PI / 180.0f;
    v3 eye    {offset_horiz, 0, offset_vert};
    v3 center {0, 0, 0};
    v3 up     {0, 0, 1};

    mat3x3 rot_z = generate_z_rot_mat(obj_rotation);
    eye = rot_z * eye;


    return generate_look_at_mat(eye, center, up);
}


v2 project_point_to_screen_space(v3 pos, mat4x4 view_mat, mat4x4 proj_mat) {

    v4 our_proj_mat[3]; // 3x4
    our_proj_mat[0] = {proj_mat.rows[0].cols[0], 0, 0, 0};
    our_proj_mat[1] = {0, proj_mat.rows[1].cols[1], 0, 0};
    our_proj_mat[2] = {0, 0, 1, 0};

    v4 our_temp[3]; //[3x4]  = our_proj_mat * view_mat
    our_temp[0] = {
        our_proj_mat[0].x * view_mat.rows[0].cols[0],
        our_proj_mat[0].x * view_mat.rows[0].cols[1],
        our_proj_mat[0].x * view_mat.rows[0].cols[2],
        our_proj_mat[0].x * view_mat.rows[0].cols[3],
    };
    our_temp[1] = {
        our_proj_mat[1].y * view_mat.rows[1].cols[0],
        our_proj_mat[1].y * view_mat.rows[1].cols[1],
        our_proj_mat[1].y * view_mat.rows[1].cols[2],
        our_proj_mat[1].y * view_mat.rows[1].cols[3],
    };
    our_temp[2] = {
        view_mat.rows[2].cols[0],
        view_mat.rows[2].cols[1],
        view_mat.rows[2].cols[2],
        view_mat.rows[2].cols[3],
    };

    v4 pos_v4 = to_v4(pos);
    v3 intermediate = { // [3x1] = our_tmp (3x4) * pos (4x1)
        dot(our_temp[0], pos_v4),
        dot(our_temp[1], pos_v4),
        dot(our_temp[2], pos_v4)
    };


    intermediate = (-1.0f /intermediate.z) * intermediate;

    return {
        intermediate.x,
        intermediate.y
    };

}

v4 to_v4(v3 v) {
    return {
        v.x, v.y, v.z, 1
    };
}

v3 get_cam_pos_for_run(const char* run_path) {
    char file_path[1024];
    sprintf(file_path, "%s/cam_info.txt", run_path);

    v3 cam_pos {};

    FILE* conf_file = fopen(file_path, "r");
    if (!conf_file) {
        fprintf(stderr, "ERROR: The cam info file was not found in %s.\n", file_path);
        return {};
    }
    fscanf(conf_file, "x_dist = %f cm\n", &cam_pos.x);
    fscanf(conf_file, "z_dist = %f cm\n", &cam_pos.z);
    fclose(conf_file);

    cam_pos.x /= -100; // camera should be looking into x direction, so make it negative
    cam_pos.z /=  100;

    return cam_pos;
}

void print(mat4x4 m) {
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[0].cols[0], m.rows[0].cols[1], m.rows[0].cols[2], m.rows[0].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[1].cols[0], m.rows[1].cols[1], m.rows[1].cols[2], m.rows[1].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[2].cols[0], m.rows[2].cols[1], m.rows[2].cols[2], m.rows[2].cols[3]);
    printf(" % 5.3f % 5.3f % 5.3f % 5.3f\n", m.rows[3].cols[0], m.rows[3].cols[1], m.rows[3].cols[2], m.rows[3].cols[3]);
}
