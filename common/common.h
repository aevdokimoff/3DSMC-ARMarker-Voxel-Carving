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

/**
 * Generates 3x3 rotation matrix around the z axis
 * @param angle of rotation
 */
Matx33d generate_z_rot_mat(f32 angle);

/**
 * Generates extrinsic matrix.
 */
Matx44d generate_proj_mat(f32 fovy, f32 aspect, f32 zNear, f32 zFar);

/**
 * Generates extrinsic matrix for camera the used for data acquisition.
 */
Matx44d generate_our_proj_mat();

/**
 * Generates a Look-At matrix.
 */
Matx44d generate_look_at_mat(const Vec3d& eye, const Vec3d& center, const Vec3d& up);

/**
 * Generates intrinsic matrix.
 */
Matx44d generate_view_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation);

/**
 * Projects a given 3D point to the screen space given the intrinsic and the extrinsic matrices.
 * @return Vec2d -- screen space coordinates of the point
 */
Vec2d project_point_to_screen_space(const Vec3d& pos, Matx44d camera_mat, Matx44d proj_mat);

/**
 * Reads camera information from the given file
 */
Cam_Info get_cam_info_for_run(const char* run_path);

/**
 * Given a homogeneous coordinates of point in the screen space, extrinsic and intrinsic matrices returns
 * a 3d coordinates of some point in 3D world space that is projected onto the given 2d point.
 */
Vec3d project_screen_point_to_3d(const Vec3d &pos, const Matx44d &extrinsic, const Matx44d &intrinsic);

#endif
