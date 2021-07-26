#include <stdio.h>

#include "../3rd_party_libs/ftb/platform.hpp"
#include "../common/image.h"
#include "../3rd_party_libs/ftb/print.hpp"

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

void realign_obj(const char* obj_path) {
    char dir[1024];
    sprintf(dir, "%srun_1/", obj_path);
    compute_offset_for_one_run(dir);
    sprintf(dir, "%srun_2/", obj_path);
    compute_offset_for_one_run(dir);
}


int main(int arg_count, char* command_line_args[]) {
    init_printer();
    realign_obj(command_line_args[1]);
}
