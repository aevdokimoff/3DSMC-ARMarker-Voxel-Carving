#pragma once
#include "../3rd_party_libs/ftb/types.hpp" // for the short types

struct Pixel {
    u8 r;
    u8 g;
    u8 b;
};

/**
 * Image structure. Has width, height and an array of pixels of size width*height.
 */
struct Image {
    s32 width;
    s32 height;
    Pixel* pixels;

    Pixel& at(u32 x, u32 y) const {
        return pixels[(x + y * width)];
    }
};

/**
 * Loads an image from the png ot jpg file with given path.
 */
Image load_image(const char* path);

/**
 * Creates an empty image of the given size.
 */
Image create_black_image(s32 width, s32 height);

/**
 * Write an image to the png file with given path.
 */
void  store_image_as_png(const Image& image, const char* path);

/**
 * Deletes the image and frees memory used by it
 */
void free_image(Image);
