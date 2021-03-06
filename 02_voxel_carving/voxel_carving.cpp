#include <cstdio>
#include <omp.h>
#include "voxel_carving.hpp"
#include "common.h"
#include "image.h"

Volume generate_point_cloud(u32 resolution, f32 side_length)
{
    Volume volume(
            Vec3d(-side_length / 2, -side_length / 2, 0),
            Vec3d(side_length / 2, side_length / 2, side_length),
            resolution);
    std::fill(volume.vol.begin(), volume.vol.end(), 0);
    return volume;
}

void carve_using_singe_image(Volume *volume, const char* image_path, uint ind,
                             const Matx44d &view_mat, const Matx44d &proj_mat,
                             s32 horiz_pixel_offset, bool output_result_image) {
    Image image = load_image(image_path);
    Image output_image{};

    if (output_result_image) {
        output_image = load_image(image_path);
    }

    for (int z = 0; z < volume->dz; ++z) {
        for (int y = 0; y < volume->dy; ++y) {
            for (int x = 0; x < volume->dx; ++x) {

                Vec2d p = project_point_to_screen_space(volume->pos(x, y, z), view_mat, proj_mat);

                if (p[0] < -1 || p[0] > 1 ||
                    p[1] < -1 || p[1] > 1)
                {
                    volume->set(x, y, z, volume->get(x, y, z) + 1);
                    continue;
                }

                int p_x = (p[0] + 1.) / 2. * image.width - horiz_pixel_offset;
                int p_y = (p[1] + 1.) / 2. * image.height;

                bool outside = image.at(p_x, p_y).r < 150;
                if (outside) volume->set(x, y, z, volume->get(x, y, z) + 1);

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
        free_image(output_image);
    }
    free_image(image);
}

void process_using_single_run(const char* run_path, Matx44d projection_mat,
                              bool carve_in_parallel, const std::function<void (const char*, uint, Matx44d, Matx44d, s32)> &onProcess)
{
    Cam_Info cam_info = get_cam_info_for_run(run_path);
    u32 thread_count = (carve_in_parallel) ? omp_get_max_threads() : 1;

    #pragma omp parallel for num_threads(thread_count)
    for (int degrees_it = 0; degrees_it < 36; degrees_it++) {
        int degrees = degrees_it * 10;

        printf("\r %03d deg", degrees);
        fflush(stdout);

        Matx44d view_mat = generate_view_mat(cam_info.camera_position[0], cam_info.camera_position[2], degrees);

        char image_path[1024];
        sprintf(image_path, "%s/bw/%03d.jpg", run_path, degrees);
        onProcess(image_path, degrees_it, view_mat, projection_mat, cam_info.horiz_pixel_offset);
    }
    printf("\rDone processing run %s\n", run_path);
}

void voxel_carve(Volume *volume, const char *path_to_runs, bool carve_in_parallel, bool output_result_image)
{
    static char file_path1[1024];
    static char file_path2[1024];

    Matx44d proj_mat = generate_our_proj_mat();

    printf("Progress in runs:\n");
    fflush(stdout);

    auto voxel_carve = [&](const char* file_path, uint ind, Matx44d view_mat,
                           Matx44d projection_mat, s32 horiz_pixel_offset)
    {
        carve_using_singe_image(volume, file_path, ind, view_mat, projection_mat,
                                horiz_pixel_offset, output_result_image);
    };

    sprintf(file_path1, "%s/run_1", path_to_runs);
    process_using_single_run(file_path1, proj_mat, carve_in_parallel, voxel_carve);

    sprintf(file_path2, "%s/run_2", path_to_runs);
    process_using_single_run(file_path2, proj_mat, carve_in_parallel, voxel_carve);
}
