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
            side_length / (float) resolution,
            cv::Vec3d(-side_length / 2, -side_length / 2, 0),
            cv::Vec3d(side_length / 2, side_length / 2, side_length),
            resolution);
    std::fill(volume.vol.begin(), volume.vol.end(), true);
    return volume;
}

Matx44d generate_look_at_mat(const Vec3d &eye, const Vec3d &center, const Vec3d &up) {
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

Matx44d perspectiveRH_ZO(f32 fovy, f32 aspect, f32 zNear, f32 zFar) {
    // assert(abs(aspect - std::numeric_limits<T>::epsilon()) > static_cast<T>(0));

    f32 const tanHalfFovy = tan(fovy / 2.0f);

    Matx44d result;
    memset(&result, 0, sizeof(result));

    result(0, 0) = 1.0 / (aspect * tanHalfFovy);
    result(1, 1) = 1.0 / (tanHalfFovy);
    result(2, 2) = zFar / (zNear - zFar);
    result(3, 2) = -1.0;
    result(2, 3) = -(zFar * zNear) / (zFar - zNear);
    return result;
}

Matx44d generate_extrinsic_mat(f32 offset_horiz, f32 offset_vert, f32 obj_rotation) {
    obj_rotation = -obj_rotation * M_PI / 180.0f;
    Vec3d eye    {offset_horiz, 0, offset_vert};
    Vec3d center {0, 0, 0};
    Vec3d up     {0, 0, 1};

    Matx33d rot_z = generate_z_rot_mat(obj_rotation);
    eye = rot_z * eye;


    return generate_look_at_mat(eye, center, up);
}

Vec3d project_point_to_screen_space(Vec3d pos, const Matx44d extrinsic, const Matx44d intrinsic) {
    Matx<double, 3, 4> our_intrinsic;
    our_intrinsic(0, 0) = intrinsic(0, 0);
    our_intrinsic(1, 1) = intrinsic(1, 1);
    our_intrinsic(2, 2) = 1;

    Matx<double, 3, 4> ourTemp = our_intrinsic * extrinsic; //[3x4]  = our_intrinsic * extrinsic

    Vec4d pos4d(pos[0], pos[1], pos[2], 1);
    Vec3d intermediate = ourTemp * pos4d;

    intermediate = -intermediate / intermediate[2];

    return intermediate;
}

Vec3d project_screen_point_to_3d(Vec3d pos, const Matx44d extrinsic, const Matx44d intrinsic) {
    Matx<double, 4, 3> our_intrinsic_inv;
    our_intrinsic_inv(0, 0) = 1 / intrinsic(0, 0);
    our_intrinsic_inv(1, 1) = 1 / intrinsic(1, 1);
    our_intrinsic_inv(2, 2) = 1;

    Vec4d intermediate = extrinsic.inv() * our_intrinsic_inv * pos;
    return Vec3d(intermediate[0], intermediate[1], intermediate[2]);
}

void carve_using_singe_image(Volume<bool> *volume, const char* image_path, const Matx44d &view_mat, const Matx44d &proj_mat, bool output_result_image = false) {
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
                Vec3d p = project_point_to_screen_space(volume->pos(x, y, z), view_mat, proj_mat);

                int p_x = (p[0] + 1.0f) / 2.0f * image_width;
                int p_y = (p[1] + 1.0f) / 2.0f * image_height;


                bool outside = pixels[IDX2D(p_x, p_y, image_width)].r < 150;
                if (outside) {
                    volume->set(x, y, z, false);
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
        stbi_image_free(output_pixels);
    }
    stbi_image_free(pixels);
}

void process_using_single_run(const char* run_path, Matx44d projection_mat,
                              bool carve_in_parallel, const std::function<void (const char*, Matx44d, Matx44d)> &onProcess)
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
    for (int degrees_it = 0; degrees_it < 36; degrees_it++) {
        int degrees = degrees_it * 10;

        printf("\r %03d deg", degrees);

        Matx44d view_mat = generate_extrinsic_mat(x_dist, z_dist, degrees);

        sprintf(file_path, "%s/bw/%03d.jpg", run_path, degrees);
        onProcess(file_path, view_mat, projection_mat);
    }
    printf("\rDone processing run %s\n", run_path);
}

Matx44d getProjectionMatrix() {
    f32 y_fov = 0.3503711;
    f32 aspect = 1.509804;
    f32 zNear = 0.3;
    f32 zFar = 200;
    return perspectiveRH_ZO(y_fov, aspect, zNear, zFar);
}

void voxel_carve(Volume<bool> *volume, const char* path_to_runs, bool carve_in_parallel, bool output_result_image) {
    static char file_path1[1024];
    static char file_path2[1024];

    Matx44d proj_mat = getProjectionMatrix();

    u32 num_threads = (carve_in_parallel) ? 2 : 1;

    printf("Progress in runs:\n");

    auto voxel_carve = [&](const char* file_path, Matx44d view_mat, Matx44d projection_mat) {
        carve_using_singe_image(volume, file_path, view_mat, projection_mat, output_result_image);
    };

    sprintf(file_path1, "%s/run_1", path_to_runs);
    process_using_single_run(file_path1, proj_mat, carve_in_parallel, voxel_carve);

    sprintf(file_path2, "%s/run_2", path_to_runs);
    process_using_single_run(file_path2, proj_mat, carve_in_parallel, voxel_carve);
}