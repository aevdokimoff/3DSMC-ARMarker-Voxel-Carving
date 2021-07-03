#pragma once
#include "../3rd_party_libs/ftb/types.hpp" // for the short types

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

struct Image {
    s32 width;
    s32 height;
    Pixel* pixels;

    Pixel& at(u32 x, u32 y) {
        return pixels[(x + y * width)];
    }
};

Image load_image(const char* path);
Image create_black_image(s32 width, s32 height);
void  store_image_as_png(Image image, const char* path);
void free_image(Image);
