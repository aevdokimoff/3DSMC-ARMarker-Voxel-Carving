#include <stdio.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../3rd_party_libs/stb/stb_image.h"
#include "../../3rd_party_libs/stb/stb_image_write.h"

const float scale_from = 0;
const float scale_to   = 95;

const int height_in_px = 300;
const int  width_in_px = 3800*2;

typedef char unsigned byte;
struct Pixel {
    byte r;
    byte g;
    byte b;
};
Pixel* image;

#define IDX(X, Y) ((X) + (Y) * width_in_px)
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

void draw_horiz_line(int start_x, int end_x, int line_y, int thickness) {
    int half_thickness = thickness/2;
    for (int y = line_y - half_thickness; y <= line_y + half_thickness; ++y) {
        int inner_y = MAX(0, y);
        inner_y = MIN(inner_y, height_in_px);

        for (int x = start_x; x <= end_x; ++x) {
            int inner_x = MAX(0, x);
            inner_x = MIN(inner_x, width_in_px);

            image[IDX(inner_x, inner_y)].r = 0;
            image[IDX(inner_x, inner_y)].g = 0;
            image[IDX(inner_x, inner_y)].b = 0;
        }
    }
}

void draw_vert_line(int start_y, int end_y, int line_x, int thickness) {
    int half_thickness = thickness / 2;

    for (int y = start_y; y <= end_y; ++y) {
        int inner_y = MAX(0, y);
        inner_y = MIN(inner_y, height_in_px);

        for (int x = line_x - half_thickness; x <= line_x + half_thickness; ++x) {
            int inner_x = MAX(0, x);
            inner_x = MIN(inner_x, width_in_px);

            image[IDX(inner_x, inner_y)].r = 0;
            image[IDX(inner_x, inner_y)].g = 0;
            image[IDX(inner_x, inner_y)].b = 0;
        }
    }
}

int main(int argc, char *argv[]) {
    size_t image_size_in_bytes = (height_in_px*width_in_px) * sizeof(image[0]);
    image = (Pixel*)malloc(image_size_in_bytes);
    memset(image, 0xff, image_size_in_bytes);


    int base_line_y = height_in_px - (height_in_px/10);

    draw_horiz_line(0, width_in_px-1, base_line_y, 10);

    for (int i = -190; i <= 190; ++i) {
        printf("\rTicks progress: %.2f %%", (i+190.0)/380*100);

        int x = (int)(((i + 190.0) / 380.0) * width_in_px + 0.5); // +0.5 to round
        int line_height    = 50;
        int line_thickness = 2;

        // maybe draw horiz line
        if (i % 20 == 0) {
            int x_end = (int)(((i+10 + 190.0) / 380.0) * width_in_px + 0.5);
            draw_horiz_line(x, x_end, base_line_y-15, 30);
        }

        if (i == 0) {
            line_height = 140;
            line_thickness = 8;
        } else if (i % 90 == 0) {
            line_height = 100;
            line_thickness = 6;
        } else if (i % 10 == 0) {
            line_height = 80;
            line_thickness = 4;
        } else if (i % 5 == 0) {
            line_height = 68;
        }

        draw_vert_line(base_line_y-line_height, base_line_y, x, line_thickness);
    }
    printf("\n");
    

    stbi_write_png("scale.png", width_in_px, height_in_px, 3, image, 0);

    return 0;
}
