#include <stdio.h>
#include <string.h>
#include <math.h>
#include <omp.h>

#include "voxel_carving.hpp"
#include "common.h"
#include "image.h"

Volume generate_point_cloud(u32 resolution, f32 side_length) {
    Volume volume(
            cv::Vec3d(-side_length / 2, -side_length / 2, 0),
            cv::Vec3d(side_length / 2, side_length / 2, side_length),
            resolution);
    std::fill(volume.vol.begin(), volume.vol.end(), 1.);
    return volume;
}

void carve_using_singe_image(Volume *volume, const char* image_path, mat4x4 view_mat, mat4x4 proj_mat, bool output_result_image = false) {
    Image image = load_image(image_path);
    Image output_image;

    if (output_result_image) {
        output_image = load_image(image_path);
    }

    for (int z = 0; z < volume->dz; ++z) {
        for (int y = 0; y < volume->dy; ++y) {
            for (int x = 0; x < volume->dx; ++x) {
                v2 p = project_point_to_screen_space(v3(volume->pos(x, y, z)), view_mat, proj_mat);

                int p_x = (p.x + 1.0f) / 2.0f * image.width;
                int p_y = (p.y + 1.0f) / 2.0f * image.height;


                bool outside = image.at(p_x, p_y).r < 150;
                if (outside) {
                    volume->set(x, y, z, 0);
                }

                if (output_result_image) {
                    int thickness = 1;
                    static const Pixel green {   0, 255, 0 };
                    static const Pixel red   { 255,   0, 0 };

                    for (int j = p_y - thickness/2; j <= p_y+thickness/2; ++j) {
                        for (int i = p_x - thickness/2; i <= p_x+thickness/2; ++i) {
                            if (j >= 0 && j < image.height && i >= 0 && i < image.width) {
                                if (outside) {
                                    output_image.at(i, j) = red;
                                } else {
                                    output_image.at(i, j) = green;
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
        store_image_as_png(output_image, file_name);
    }
}

void carve_using_single_run(Volume *volume, const char* run_path, mat4x4 projection_mat,
                            bool carve_in_parallel, bool output_result_image = false)
{
    v3 cam_pos = get_cam_pos_for_run(run_path);
    u32 thread_count = (carve_in_parallel) ? omp_get_max_threads() : 1;

#pragma omp parallel for num_threads(thread_count)
    for (int degrees = 0; degrees < 360; degrees += 10) {
        char file_path[1024];

        printf("\r %03d deg", degrees);
        fflush(stdout);

        mat4x4 view_mat = generate_view_mat(cam_pos.x, cam_pos.z, degrees);

        sprintf(file_path, "%s/bw/%03d.jpg", run_path, degrees);
        carve_using_singe_image(volume, file_path,
                                view_mat, projection_mat, output_result_image);


    }
    printf("\rDone processing run %s\n", run_path);
}


void voxel_carve(Volume *volume, u32 res, f32 side_length, const char* path_to_runs, bool carve_in_parallel) {
    static char file_path1[1024];
    static char file_path2[1024];

    mat4x4 proj_mat = generate_our_proj_mat();

    printf("Progress in runs:\n");

    sprintf(file_path1, "%s/run_1", path_to_runs);
    carve_using_single_run(volume, file_path1, proj_mat, carve_in_parallel, true);

    sprintf(file_path2, "%s/run_2", path_to_runs);
    carve_using_single_run(volume, file_path2, proj_mat, carve_in_parallel, true);
}


int main(int argc, char *argv[]) {
    u32 resolution = 100; // 100 vertices per dimension
    f32 sideLength = 0.1; // 10 cm

    Volume volume = generate_point_cloud(resolution, sideLength);
    voxel_carve(&volume, resolution, sideLength, "./01_data_acquisition/images/obj_duck", true);

    return 0;
}
