#include "common.h"
#include <cstring>
#include <cstdio>
#include <opencv2/core/core.hpp>

Matx33d generate_z_rot_mat(f32 angle) {
    Matx33d mat;
    mat(0, 0) =  cos(angle);
    mat(0, 1) = -sin(angle);
    mat(1, 0) =  sin(angle);
    mat(1, 1) =  cos(angle);
    mat(2, 2) = 1;

    return mat;
}

Matx44d generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar) {
    // NOTE(Felix): This function was taken from and adaped from glm:
    // https://github.com/g-truc/glm/blob/b3f87720261d623986f164b2a7f6a0a938430271/glm/ext/matrix_clip_space.inl#L233
    f32 const tanHalfFovy = tan(fovy / 2.);

    Matx44d result;
    memset(&result, 0, sizeof(result));

    result(0, 0) = 1.0 / (aspect * tanHalfFovy);
    result(1, 1) = 1.0 / (tanHalfFovy);
    result(2, 2) = zFar / (zNear - zFar);
    result(3, 2) = -1.0;
    result(2, 3) = -(zFar * zNear) / (zFar - zNear);
    return result;
}

Matx44d generate_our_proj_mat() {
    f32 y_fov = 0.3503711;
    f32 aspect = 1.509804;
    Matx44d proj_mat = generate_proj_mat(y_fov, aspect, 0.3, 200);
    return proj_mat;
}

Matx44d generate_look_at_mat(const Vec3d &eye, const Vec3d &center, const Vec3d &up) {
    // NOTE(Felix): This function was taken from glm and adapted
    // https://github.com/g-truc/glm/blob/b3f87720261d623986f164b2a7f6a0a938430271/glm/ext/matrix_transform.inl#L99
    Vec3d f = normalize(eye - center);
    Vec3d s = normalize(f.cross(up));
    Vec3d u = s.cross(f);

    Matx44d result = Matx44d::eye();
    result(0, 0) = s[0];
    result(0, 1) = s[1];
    result(0, 2) = s[2];
    result(1, 0) = u[0];
    result(1, 1) = u[1];
    result(1, 2) = u[2];
    result(2, 0) = -f[0];
    result(2, 1) = -f[1];
    result(2, 2) = -f[2];
    result(0, 3) = -s.dot(eye);
    result(1, 3) = -u.dot(eye);
    result(2, 3) = f.dot(eye);

    return result;
}

Matx44d generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation) {
    obj_rotation = obj_rotation * M_PI / 180.0f;
    Vec3d eye    {offset_horiz, 0, offset_vert};
    Vec3d center {0, 0, 0};
    Vec3d up     {0, 0, 1};

    Matx33d rot_z = generate_z_rot_mat(obj_rotation);
    eye = rot_z * eye;

    return generate_look_at_mat(eye, center, up);
}


Vec2d project_point_to_screen_space(const Vec3d &pos, Matx44d view_mat, Matx44d proj_mat) {
    Matx<double, 3, 4> our_projection_matrix;
    our_projection_matrix(0, 0) = proj_mat(0, 0);
    our_projection_matrix(1, 1) = proj_mat(1, 1);
    our_projection_matrix(2, 2) = 1;

    Matx<double, 3, 4> ourTemp = our_projection_matrix * view_mat;

    Vec4d pos4d(pos[0], pos[1], pos[2], 1);
    Vec3d intermediate = ourTemp * pos4d;

    intermediate = -intermediate / intermediate[2];

    return Vec2d(intermediate[0], intermediate[1]);

}

Vec3d get_cam_pos_for_run(const char* run_path) {
    char file_path[1024];
    sprintf(file_path, "%s/cam_info.txt", run_path);

    Vec3d cam_pos {};

    FILE* conf_file = fopen(file_path, "r");
    if (!conf_file) {
        fprintf(stderr, "ERROR: The cam info file was not found in %s.\n", file_path);
        return {};
    }
    fscanf(conf_file, "x_dist = %lf cm\n", &cam_pos[0]);
    fscanf(conf_file, "z_dist = %lf cm\n", &cam_pos[2]);
    fclose(conf_file);

    cam_pos[0] /= -100; // camera should be looking into x direction, so make it negative
    cam_pos[2] /=  100;

    return cam_pos;
}

Vec3d project_screen_point_to_3d(const Vec3d &pos, const Matx44d &extrinsic, const Matx44d &intrinsic) {
    Matx<double, 4, 3> our_intrinsic_inv;
    our_intrinsic_inv(0, 0) = 1 / intrinsic(0, 0);
    our_intrinsic_inv(1, 1) = 1 / intrinsic(1, 1);
    our_intrinsic_inv(2, 2) = 1;

    Vec4d intermediate = extrinsic.inv() * our_intrinsic_inv * pos;
    return Vec3d(intermediate[0], intermediate[1], intermediate[2]);
}
