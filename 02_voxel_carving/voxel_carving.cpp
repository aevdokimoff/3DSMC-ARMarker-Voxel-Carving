#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <cstdio>
#include <cstring>
#include <cmath>
#include <omp.h>
#include "../3rd_party_libs/stb/stb_image.h"
#include "../3rd_party_libs/stb/stb_image_write.h"

#include "voxel_carving.hpp"

u32 degree_progress_run_1 = 0;
u32 degree_progress_run_2 = 0;

Volume<bool> generate_point_cloud(u32 resolution, f32 side_length) {
    Volume<bool> volume(
            cv::Vec3d(-side_length / 2, -side_length / 2, 0),
            cv::Vec3d(side_length / 2, side_length / 2, side_length),
            resolution);
    std::fill(volume.vol.begin(), volume.vol.end(), true);
    return volume;
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

mat4x4 perspectiveRH_ZO(f32 fovy, f32 aspect, f32 zNear, f32 zFar) {
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

mat4x4 generate_extrinsic_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation) {
    obj_rotation = -obj_rotation * M_PI / 180.0f;
    v3 eye    {offset_horiz, 0, offset_vert};
    v3 center {0, 0, 0};
    v3 up     {0, 0, 1};

    mat3x3 rot_z = generate_z_rot_mat(obj_rotation);
    eye = rot_z * eye;


    return generate_look_at_mat(eye, center, up);
}

v3 project_point_to_screen_space(v3 pos, mat4x4 extrinsic, mat4x4 intrinsic) {

    v4 our_intrinsic[3]; // 3x4
    our_intrinsic[0] = {intrinsic.rows[0].cols[0], 0, 0, 0};
    our_intrinsic[1] = {0, intrinsic.rows[1].cols[1], 0, 0};
    our_intrinsic[2] = {0, 0, 1, 0};

    v4 our_temp[3]; //[3x4]  = our_intrinsic * extrinsic
    our_temp[0] = {
        our_intrinsic[0].x * extrinsic.rows[0].cols[0],
        our_intrinsic[0].x * extrinsic.rows[0].cols[1],
        our_intrinsic[0].x * extrinsic.rows[0].cols[2],
        our_intrinsic[0].x * extrinsic.rows[0].cols[3],
    };
    our_temp[1] = {
        our_intrinsic[1].y * extrinsic.rows[1].cols[0],
        our_intrinsic[1].y * extrinsic.rows[1].cols[1],
        our_intrinsic[1].y * extrinsic.rows[1].cols[2],
        our_intrinsic[1].y * extrinsic.rows[1].cols[3],
    };
    our_temp[2] = {
        extrinsic.rows[2].cols[0],
        extrinsic.rows[2].cols[1],
        extrinsic.rows[2].cols[2],
        extrinsic.rows[2].cols[3],
    };

    v4 pos_v4 = to_v4(pos);
    v3 intermediate = { // [3x1] = our_tmp (3x4) * pos (4x1)
        dot(our_temp[0], pos_v4),
        dot(our_temp[1], pos_v4),
        dot(our_temp[2], pos_v4)
    };
    
    
    intermediate = (-1.0f /intermediate.z) * intermediate;

    return intermediate;

}

void carve_using_singe_image(Volume<bool> *volume, const char* image_path, mat4x4 view_mat, mat4x4 proj_mat, bool output_result_image = false) {
    int image_width, image_height;
    int n;
    auto* pixels = (Pixel*)stbi_load(image_path, &image_width, &image_height, &n, 0);
    if (!pixels) {
        fprintf(stderr, "Error opening image: %s\n", image_path);
        return;
    }

    Pixel* output_pixels;
    if (output_result_image) {
        output_pixels = (Pixel*)stbi_load(image_path, &image_width, &image_height, &n, 0);
    }

    for (int z = 0; z < volume->dz; ++z) {
        for (int y = 0; y < volume->dy; ++y) {
            for (int x = 0; x < volume->dx; ++x) {
                v3 p = project_point_to_screen_space(v3(volume->pos(x, y, z)), view_mat, proj_mat);

                int p_x = (p.x + 1.0f) / 2.0f * image_width;
                int p_y = (p.y + 1.0f) / 2.0f * image_height;


                bool outside = pixels[IDX2D(p_x, p_y, image_width)].r < 150;
                if (outside) {
                    volume->set(x, y, z, 0);
                }

                if (output_result_image) {
                    int thickness = 1;
                    static const Pixel green {   0, 255, 0 };
                    static const Pixel red   { 255,   0, 0 };

                    for (int j = p_y - thickness/2; j <= p_y+thickness/2; ++j) {
                        for (int i = p_x - thickness/2; i <= p_x+thickness/2; ++i) {
                            if (j >= 0 && j < image_height && i >= 0 && i < image_width) {
                                if (outside) {
                                    output_pixels[j * image_width + i] = red;
                                } else {
                                    output_pixels[j * image_width + i] = green;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (output_result_image) {
        char file_name[1024];
        sprintf(file_name, "%s_carved.png", image_path);
        stbi_write_png(file_name, image_width, image_height, 3, output_pixels, 0);
    }
}

void process_using_single_run(const char* run_path, mat4x4 projection_mat,
                              bool carve_in_parallel, const std::function<void (const char*, mat4x4, mat4x4)> &onProcess)
{
    char file_path[1024];
    sprintf(file_path, "%s/cam_info.txt", run_path);

    f32 x_dist;
    f32 z_dist;

    FILE* conf_file = fopen(file_path, "r");
    if (!conf_file) {
        fprintf(stderr, "The cam info file was not found in %s.\n", file_path);
        return;
    }
    fscanf(conf_file, "x_dist = %f cm\n", &x_dist);
    fscanf(conf_file, "z_dist = %f cm\n", &z_dist);
    fclose(conf_file);

    x_dist /= 100;
    z_dist /= 100;

    u32 thread_count = (carve_in_parallel) ? omp_get_max_threads() : 1;

#pragma omp parallel for num_threads(thread_count)
    for (int degrees = 0; degrees < 36; degrees ++) {
        degrees *= 10;

        printf("\r %03d deg", degrees);

        mat4x4 view_mat = generate_extrinsic_mat(x_dist, z_dist, degrees);

        sprintf(file_path, "%s/bw/%03d.jpg", run_path, degrees);
        onProcess(file_path, view_mat, projection_mat);
    }
    printf("\rDone processing run %s\n", run_path);
}

mat4x4 getProjectionMatrix() {
    f32 y_fov = 0.3503711;
    f32 aspect = 1.509804;
    f32 zNear = 0.3;
    f32 zFar = 200;
    return perspectiveRH_ZO(y_fov, aspect, zNear, zFar);
}

void voxel_carve(Volume<bool> *volume, const char* path_to_runs, bool carve_in_parallel, bool output_result_image) {
    static char file_path1[1024];
    static char file_path2[1024];

    mat4x4 proj_mat = getProjectionMatrix();

    u32 num_threads = (carve_in_parallel) ? 2 : 1;

    printf("Progress in runs:\n");

    auto voxel_carve = [&](const char* file_path, mat4x4 view_mat, mat4x4 projection_mat) {
        carve_using_singe_image(volume, file_path, view_mat, projection_mat, output_result_image);
    };

    sprintf(file_path1, "%s/run_1", path_to_runs);
    process_using_single_run(file_path1, proj_mat, carve_in_parallel, voxel_carve);

    sprintf(file_path2, "%s/run_2", path_to_runs);
    process_using_single_run(file_path2, proj_mat, carve_in_parallel, voxel_carve);

    delete volume;
}