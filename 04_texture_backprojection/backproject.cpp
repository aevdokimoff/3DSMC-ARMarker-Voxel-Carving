#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../3rd_party_libs/ftb/print.hpp"
#include "../3rd_party_libs/ftb/arraylist.hpp"
#include "../3rd_party_libs/ftb/hashmap.hpp"
#include "../3rd_party_libs/ftb/macros.hpp"

#include "common.h"
#include "image.h"

#include "mesh_stuff.hpp"

#define min(x, y)     (((x) < (y)) ? (x) : (y))
#define min3(x, y, z) (min(min((x), (y)), (z)))
#define max(x, y)     (((x) > (y)) ? (x) : (y))
#define max3(x, y, z) (max(max((x), (y)), (z)))

void find_path_of_best_image(v3 normal, const char* runs_path,
                             char** out_image_path, mat4x4* out_camera_mat) {

    char cam_info_path[1024];
    sprintf(cam_info_path, "%s/run_1/", runs_path);
    v3 cam_pos_run_1 = get_cam_pos_for_run(cam_info_path);
    sprintf(cam_info_path, "%s/run_2/", runs_path);
    v3 cam_pos_run_2 = get_cam_pos_for_run(cam_info_path);

    f32 max_dot = -1;
    s32 max_dot_run = -1;
    s32 max_dot_degrees = -1;

    for (int degrees = 0; degrees < 360; degrees += 10) {
        f32 obj_rotation = degrees * PI / 180.0f;

        v3 cam_dir_run_1 = normalize(generate_z_rot_mat(obj_rotation) * cam_pos_run_1);
        v3 cam_dir_run_2 = normalize(generate_z_rot_mat(obj_rotation) * cam_pos_run_2);

        f32 t = dot(normal, cam_dir_run_1);
        if (t > max_dot) {
            max_dot = t;
            max_dot_run = 1;
            max_dot_degrees = degrees;
        }

        t = dot(normal, cam_dir_run_2);
        if (t > max_dot) {
            max_dot = t;
            max_dot_run = 2;
            max_dot_degrees = degrees;
        }
    }

    f32 max_dot_offset_horiz =
        (max_dot_run == 1)
        ? cam_pos_run_1.x
        : cam_pos_run_2.x;
    f32 max_dot_offset_vert =
        (max_dot_run == 1)
        ? cam_pos_run_1.z
        : cam_pos_run_2.z;

    *out_camera_mat = generate_view_mat(max_dot_offset_horiz,
                                              max_dot_offset_vert,
                                              max_dot_degrees);
    print_to_string(out_image_path, "run_%d/rgb/%03d.jpg", max_dot_run, max_dot_degrees);

}

v3 find_barycentric_coords(v2 point, V2_Triangle t) {
    // source: https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Conversion_between_barycentric_and_Cartesian_coordinates
    v3 result;
    result.x =
        ((t.b.y - t.c.y)*(point.x - t.c.x) + (t.c.x - t.b.x)*(point.y - t.c.y)) /
        ((t.b.y - t.c.y)*(t.a.x   - t.c.x) + (t.c.x - t.b.x)*(t.a.y   - t.c.y));
    result.y =
        ((t.c.y - t.a.y)*(point.x - t.c.x) + (t.a.x - t.c.x)*(point.y - t.c.y)) /
        ((t.b.y - t.c.y)*(t.a.x   - t.c.x) + (t.c.x - t.b.x)*(t.a.y   - t.c.y));
    result.z = 1 - result.x - result.y;
    return result;
}

inline void fill_uv_coord_with_color(v2 uv_coord, v3 barycentric_coord, V2_Triangle camera_space_triangle,
                                     Image uv_image, Image camera_image)
{
    v2 camera_space_pixel {
        camera_space_triangle.a.x * barycentric_coord.x + camera_space_triangle.b.x * barycentric_coord.y + camera_space_triangle.c.x * barycentric_coord.z,
        camera_space_triangle.a.y * barycentric_coord.x + camera_space_triangle.b.y * barycentric_coord.y + camera_space_triangle.c.y * barycentric_coord.z,
    };
    uv_image.at((u32)uv_coord.x, (u32)uv_coord.y)
        = camera_image.at((u32)camera_space_pixel.x, (u32)camera_space_pixel.y);
}

void create_image_texture(const char* runs_path, const char* obj_path,
                          const char* image_out_path, u32 out_res) {
    Mesh* mesh = load_obj(obj_path);

    mat4x4 proj_mat = generate_our_proj_mat();
    Image image_texture = create_black_image(out_res, out_res);

    // for every face
#pragma omp parallel for
    for (s32 i = 0; i < mesh->indices.count; i += 3) {
        Image camera_image;
        printf("face: %u %u %u | %u\n",
               mesh->indices[i+0],
               mesh->indices[i+1],
               mesh->indices[i+2],
               mesh->indices.count);


        v3 average_normal =
            normalize(mesh->vertices[i+0].normal +
                      mesh->vertices[i+1].normal +
                      mesh->vertices[i+2].normal);


        char* best_image_path;
        mat4x4 camera_mat;
        char file_path[1024];
        find_path_of_best_image(average_normal, runs_path, &best_image_path, &camera_mat);
        sprintf(file_path, "%s/%s", runs_path, best_image_path);
        printf("loading: %s\n", file_path);
        fflush(stdout);

        camera_image = load_image(file_path);
        
        V2_Triangle image_space_triangle;
        image_space_triangle.a = {
            mesh->vertices[i+0].uv_coord.x * (f32)image_texture.width,
            mesh->vertices[i+0].uv_coord.y * (f32)image_texture.height
        };
        image_space_triangle.b = {
            mesh->vertices[i+1].uv_coord.x * (f32)image_texture.width,
            mesh->vertices[i+1].uv_coord.y * (f32)image_texture.height
        };
        image_space_triangle.c =  {
            mesh->vertices[i+2].uv_coord.x * (f32)image_texture.width,
            mesh->vertices[i+2].uv_coord.y * (f32)image_texture.height
        };

        V2_Triangle camera_space_triangle {
            project_point_to_screen_space(mesh->vertices[i+0].pos, camera_mat, proj_mat),
            project_point_to_screen_space(mesh->vertices[i+1].pos, camera_mat, proj_mat),
            project_point_to_screen_space(mesh->vertices[i+2].pos, camera_mat, proj_mat)
        };
        camera_space_triangle.a.x =(camera_space_triangle.a.x + 1.0f) / 2.0f * camera_image.width;
        camera_space_triangle.a.y =(camera_space_triangle.a.y + 1.0f) / 2.0f * camera_image.height;
        camera_space_triangle.b.x =(camera_space_triangle.b.x + 1.0f) / 2.0f * camera_image.width;
        camera_space_triangle.b.y =(camera_space_triangle.b.y + 1.0f) / 2.0f * camera_image.height;
        camera_space_triangle.c.x =(camera_space_triangle.c.x + 1.0f) / 2.0f * camera_image.width;
        camera_space_triangle.c.y =(camera_space_triangle.c.y + 1.0f) / 2.0f * camera_image.height;

        // printf("Image space triangle");
        u32 u_start = (u32)min3(image_space_triangle.a.x, image_space_triangle.b.x, image_space_triangle.c.x);
        u32 v_start = (u32)min3(image_space_triangle.a.y, image_space_triangle.b.y, image_space_triangle.c.y);
        u32 width   = (u32)(ceil(max3(image_space_triangle.a.x, image_space_triangle.b.x, image_space_triangle.c.x) - u_start));
        u32 height  = (u32)(ceil(max3(image_space_triangle.a.y, image_space_triangle.b.y, image_space_triangle.c.y) - v_start));

        // color in vertices first
        {
            fill_uv_coord_with_color(image_space_triangle.a, {1,0,0}, camera_space_triangle, image_texture, camera_image);
            fill_uv_coord_with_color(image_space_triangle.b, {0,1,0}, camera_space_triangle, image_texture, camera_image);
            fill_uv_coord_with_color(image_space_triangle.c, {0,0,1}, camera_space_triangle, image_texture, camera_image);

        }

        for (u32 y = v_start; y < v_start+height; ++y) {
            for (u32 x = u_start; x < u_start+width; ++x) {
                for (u8 sub_v = 0; sub_v < 2; ++sub_v) {
                    for (u8 sub_u = 0; sub_u < 2; ++sub_u) {
                        v2 pixel {
                            (f32)x + sub_u,
                            (f32)y + sub_v
                        };

                        v3 b_coords = find_barycentric_coords(pixel, image_space_triangle);
                        if (b_coords.x > 0 && b_coords.y > 0 && b_coords.z > 0) {
                            // we are in the triangle, yey
                            fill_uv_coord_with_color({(f32)x, (f32)y}, b_coords, camera_space_triangle, image_texture, camera_image);
                            goto next_pixel;
                        }
                    }
                }
            next_pixel:;
            }
        }
        free_image(camera_image);
    }

    store_image_as_png(image_texture, image_out_path);
}

void print_usage() {
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  texture_backprojection <runs_path> <object_path> <out_file> <resolution>\n");
    fprintf(stderr, "Where:\n");
    fprintf(stderr, "  - runs_path  :: The path to where the runs directory is.\n");
    fprintf(stderr, "  - obj_path   :: The path to where the .obj file is.\n");
    fprintf(stderr, "  - out_file   :: The image_texture that should be genereated (will be png).\n");
    fprintf(stderr, "  - resolution :: The resolution of image_texture (will be square).\n");
    fprintf(stderr, "Example:\n");
    fprintf(stderr, "  texture_backprojection \"01_data_acquisition/images/obj_duck\" \"marching_cubes_result.obj\" \\\n");
    fprintf(stderr, "                         \"texture.png\" 4096\n");
}

s32 main(s32 arg_count, char* arg_values[]) {
    if (arg_count != 5) {
        print_usage();
        return 1;
    }

    s32 res = atoi(arg_values[4]);
    if (res < 1) {
        print_usage();
        return 1;
    }
    
    create_image_texture(arg_values[1], arg_values[2], arg_values[3], res);
    return 0;
}
