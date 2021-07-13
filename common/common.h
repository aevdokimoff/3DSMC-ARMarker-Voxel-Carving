#ifndef INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H
#define INC_3DSMC_ARMARKER_VOXEL_CARVING_COMMON_H

#include <cmath>
#include <cstdint>
#include <opencv2/core/mat.hpp>
#include "../3rd_party_libs/ftb/types.hpp" // for the short types

using namespace cv;

Matx33d generate_z_rot_mat(f32 angle);

Matx44d generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar);
Matx44d generate_our_proj_mat();
Matx44d generate_look_at_mat(Vec3d eye, Vec3d center, Vec3d up);
Matx44d generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation);
Vec2d project_point_to_screen_space(Vec3d pos, Matx44d camera_mat, Matx44d proj_mat);
Vec3d get_cam_pos_for_run(const char* run_path);

#endif