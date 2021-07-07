#pragma once
#include "common.h"

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

struct Image {
    s32 width;
    s32 height;
    Pixel* pixels;

    Pixel& at(u32 x, u32 y) const {
        return pixels[(x + y * width)];
    }

    ~Image();
};

Image load_image(const char* path);
Image create_black_image(s32 width, s32 height);
void  store_image_as_png(const Image& image, const char* path);
void free_image(Image);
