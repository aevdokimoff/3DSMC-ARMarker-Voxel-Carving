#include "voxel_carving.hpp"

int main(int argc, char *argv[]) {
    Volume<bool> volume = generate_point_cloud(100, 0.1);
    voxel_carve(&volume, "./01_data_acquisition/images/obj_owl");
    return 0;
}