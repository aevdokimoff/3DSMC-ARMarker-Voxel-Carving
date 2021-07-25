#ifndef INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H
#define INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H

#include <cmath>
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include "../3rd_party_libs/ftb/types.hpp" // for the short types

using namespace cv;

struct V2_Triangle {
    Vec2d a;
    Vec2d b;
    Vec2d c;
};

struct Cam_Info {
    Vec3d camera_position;
    s32 horiz_pixel_offset;
};

Matx33d generate_z_rot_mat(f32 angle);

Matx44d generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar);
Matx44d generate_our_proj_mat();
Matx44d generate_look_at_mat(const Vec3d& eye, const Vec3d& center, const Vec3d& up);
Matx44d generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation);
Vec2d project_point_to_screen_space(const Vec3d& pos, Matx44d camera_mat, Matx44d proj_mat);
Cam_Info get_cam_info_for_run(const char* run_path);
Vec3d project_screen_point_to_3d(const Vec3d &pos, const Matx44d &extrinsic, const Matx44d &intrinsic);

#endif
