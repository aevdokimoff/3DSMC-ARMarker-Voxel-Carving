#include "simple_marching_cubes.h"
#include "surface/implicit_surface.h"
#include <string>

#define USE_IMPLICIT_SURFACE false
#define TORUS true

#define OBJECT Object::owl

#define WITH_VOXEL_CARVING true
#define SAVE_VOLUME false
#define IN_PARALLEL true
#define SAVE_RESULT_IMAGE false

#define LOW_RESOLUTION false

enum Object {owl, duck, unicorn};

Volume<bool> getImplicitVolume(bool torus = true) {
    ImplicitSurface* surface;
    if (torus) {
        surface = new Torus(Vec3d(0.5, 0.5, 0.5), 0.4, 0.1);
    } else {
    	surface = new Sphere(Vec3d(0.5, 0.5, 0.5), 0.4);
    }

    unsigned int resolution = 50;
    Volume<bool> volume(Vec3d(-0.1, -0.1, -0.1), Vec3d(1.1, 1.1, 1.1), resolution);
    for (int x = 0; x < volume.getDimX(); x++)
    {
        for (int y = 0; y < volume.getDimY(); y++)
        {
            for (int z = 0; z < volume.getDimZ(); z++)
            {
                Vec3d p = volume.pos(x, y, z);
                double val = surface->Eval(p);
                volume.set(x, y, z, val <= 0);
            }
        }
    }
    delete surface;
    return volume;
}

int main(int argc, char *argv[]) {
    std::string name;
    switch (OBJECT) {
        case Object::owl:
            name = "owl";
            break;
        case Object::duck:
            name = "duck";
            break;
        case Object::unicorn:
            name = "unicorn";
            break;
        default:
            cout << "Failed to interpret the object";
            return 1;
    }

    std::string outputFile = "./results/simple_" + name + (LOW_RESOLUTION ? "_low_resolution" : "") + ".off";

    Volume<bool> volume;
    if (USE_IMPLICIT_SURFACE)
    {
        volume = getImplicitVolume(TORUS);
        name = TORUS ? "torus" : "sphere";
    }
    else if (WITH_VOXEL_CARVING)
    {
        int resolution = LOW_RESOLUTION ? 20 : 100;
        volume = generate_point_cloud(resolution, 0.1);
        voxel_carve(&volume, ("../01_data_acquisition/images/obj_" + name).data(), IN_PARALLEL, SAVE_RESULT_IMAGE);
        if (SAVE_VOLUME) {
            volume.writeToFile("./results/volume_" + name + ".txt");
            volume.writePointCloudToFile("./results/point_cloud_" + name + ".ply");
        }
    }
    else
    {
        volume.readFromFile("./results/volume_" + name + ".txt");
    }

    SimpleMesh mesh;
//    ProjectedMarchingCubes marchingCubes(&volume, "../01_data_acquisition/images/obj_" + name, IN_PARALLEL);
    SimpleMarchingCubes marchingCubes(&volume, IN_PARALLEL);
    marchingCubes.processVolume(&mesh);

    if (!mesh.writeMesh(outputFile))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        return -1;
    }
    return 0;
}
