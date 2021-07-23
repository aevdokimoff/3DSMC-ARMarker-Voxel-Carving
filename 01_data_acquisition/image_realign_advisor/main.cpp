#include <stdio.h>

#include "../3rd_party_libs/ftb/platform.hpp"

// #ifdef FTB_WINDOWS
// #  include <direct.h> // for _mkdir
// #else
// #  include <sys/stat.h> // for mkdir
// #endif

#include "../common/image.h"
#include "../3rd_party_libs/ftb/print.hpp"


// void create_directory(const char* path) {
// #ifdef FTB_WINDOWS
//     _mkdir(path);
// #else
//     mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
// #endif
// }

// enum struct Scoring_Method {
//     Left_Right_Measure,
//     Area_Measure
// };

// void realign_single_image_pair(const char* in_bw_path, const char* out_bw_path,
//                                const char* in_rgb_path, const char* out_rgb_path,
//                                s32 x_offset)
// {
//     Image in_bw_img = load_image(in_bw_path);
//     Image out_bw_img = create_black_image(in_bw_img.width, in_bw_img.height);
//     Image in_rgb_img = load_image(in_rgb_path);
//     Image out_rgb_img = create_black_image(in_bw_img.width, in_bw_img.height);

//     if (x_offset < 0) {
//         u32 in_bw_img_x = -x_offset;
// #pragma omp parallel for num_threads(2)
//         for (s32 y = 0; y < in_bw_img.height; y++) {
//             memcpy(&out_bw_img.pixels[out_bw_img.width*y],
//                    &in_bw_img.pixels[in_bw_img.width*y+in_bw_img_x],
//                    sizeof(Pixel) * (in_bw_img.width - in_bw_img_x));

//             memcpy(&out_rgb_img.pixels[out_rgb_img.width*y],
//                    &in_rgb_img.pixels[in_rgb_img.width*y+in_bw_img_x],
//                    sizeof(Pixel) * (in_rgb_img.width - in_bw_img_x));
//         }
//     } else if (x_offset > 0) {
// #pragma omp parallel for num_threads(2)
//         for (s32 y = 0; y < in_bw_img.height; y++) {
//             memcpy(&out_bw_img.pixels[out_bw_img.width*y+x_offset],
//                    &in_bw_img.pixels[in_bw_img.width*y],
//                    sizeof(Pixel) * (in_bw_img.width - x_offset));

//             memcpy(&out_rgb_img.pixels[out_bw_img.width*y+x_offset],
//                    &in_rgb_img.pixels[in_bw_img.width*y],
//                    sizeof(Pixel) * (in_bw_img.width - x_offset));
//         }
//     }


//     store_image_as_png(out_bw_img, out_bw_path);
//     store_image_as_png(out_rgb_img, out_rgb_path);

//     free_image(in_bw_img);
//     free_image(in_rgb_img);
//     free_image(out_bw_img);
//     free_image(out_rgb_img);
// }

// s32 compute_area_measure(const char* image_path) {
//     Image i = load_image(image_path);

//     s32 measure = 0;
//     s32 image_center = i.width / 2;

//     for (u32 y = 0; y < i.height; ++y) {
//         for (u32 x = 0; x < i.width; ++x) {
//             if (i.at(x, y).r > 150) {
//                 measure += image_center - x;
//             }
//         }
//     }

//     return measure;
// }

s32 compute_left_right_measure(const char* image_path) {
    Image i = load_image(image_path);

    u32 min_x = i.width;
    u32 max_x = 0;
    
    for (u32 y = 0; y < i.height; ++y) {
        for (u32 x = 0; x < i.width; ++x) {
            if (i.at(x, y).r > 150) {
                min_x = MIN(x, min_x);
                max_x = MAX(x, max_x);
            }
        }
    }

    s32 result = (i.width/2) - (min_x + ((max_x - min_x) / 2));
    // println("Computed offset for image %s was %d", image_path, result);
    return result; 
}

s32 compute_offset_for_single_image(const char* image_path) {
    return compute_left_right_measure(image_path);
}

s32 compute_offset_for_one_run(const char* run_path) {
    s32 sum_of_offsets = 0;

    for (int degrees = 0; degrees < 360; degrees += 10) {
        char bw_image_path[1024];
        sprintf(bw_image_path, "%sbw/%03d.jpg",  run_path, degrees);
        
        sum_of_offsets += compute_offset_for_single_image(bw_image_path);
    }

    println("Offset for run %s was %d", run_path, sum_of_offsets / 36);
    return sum_of_offsets / 36;
}

void realign_one_run(const char* run_path) {
    s32 offset = compute_offset_for_one_run(run_path);
    // s32 offset = -90;

    // char new_bw_dir [1024];
    // char new_rgb_dir[1024];
    // const char* suffix = ((method == Scoring_Method::Left_Right_Measure) ? "LR" : "A");
    // sprintf(new_bw_dir,  "%srealigned_bw_%s",  run_path, suffix);
    // sprintf(new_rgb_dir, "%srealigned_rgb_%s", run_path, suffix);
    // create_directory(new_bw_dir);
    // create_directory(new_rgb_dir);

    // for (int degrees = 0; degrees < 360; degrees += 10) {
    //     char old_bw [1024];
    //     char old_rgb[1024];
    //     char new_bw [1024];
    //     char new_rgb[1024];

    //     sprintf(old_bw,  "%sbw/%03d.jpg",  run_path, degrees);
    //     sprintf(old_rgb, "%srgb/%03d.jpg", run_path, degrees);
    //     sprintf(new_bw,  "%s/%03d.jpg",    new_bw_dir, degrees);
    //     sprintf(new_rgb, "%s/%03d.jpg",    new_rgb_dir, degrees);

    //     realign_single_image_pair(old_bw, new_bw, old_rgb, new_rgb, offset);
    // }
}

void realign_obj(const char* obj_path) {
    char dir[1024];
    sprintf(dir, "%srun_1/", obj_path);
    realign_one_run(dir);
    sprintf(dir, "%srun_2/", obj_path);
    realign_one_run(dir);
}


int main(int arg_count, char* command_line_args[]) {
    init_printer();
    
    realign_obj(command_line_args[1]);
    // realign_obj("D:/Code/voxel_carving/01_data_acquisition/images/obj_owl/", Scoring_Method::Left_Right_Measure);
    // realign_obj("D:/Code/voxel_carving/01_data_acquisition/images/obj_duck/");
}
