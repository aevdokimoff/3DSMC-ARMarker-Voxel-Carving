#include <cstdlib>
#include <cstdio>

#include "image.h"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../3rd_party_libs/stb/stb_image.h"
#include "../3rd_party_libs/stb/stb_image_write.h"

Image create_black_image(s32 width, s32 height) {
    size_t pixels_byte_size = sizeof(Pixel) * width * height;
    Image ret {
        .width = width,
        .height = height,
        .pixels = (Pixel*)malloc(pixels_byte_size)
    };
    memset(ret.pixels, 0, pixels_byte_size);
    return ret;
};

void store_image_as_png(const Image &image, const char* path) {
    stbi_write_png(path, image.width,
                   image.height, 3,
                   image.pixels, 0);
}

Image load_image(const char* path) {
    Image image{};
    int n;
    image.pixels = (Pixel*)stbi_load(path, &image.width,
                                     &image.height, &n, 3);
    if (!image.pixels) {
        fprintf(stderr, "ERROR: The image '%s' could not be opened.\n", path);
    }
    return image;
}

Image::~Image() {
    stbi_image_free(pixels);
}
