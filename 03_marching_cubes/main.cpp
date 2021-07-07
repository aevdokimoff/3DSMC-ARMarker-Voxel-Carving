#include "simple_marching_cubes.h"
#include "surface/implicit_surface.h"
#include <string>

enum Object { owl, duck, unicorn};

Volume<bool> getImplicitVolume(bool torus = true) {
    ImplicitSurface* surface;
    if (torus) {
        surface = new Torus(Vec3d(0.5, 0.5, 0.5), 0.4, 0.1);
    } else {
    	surface = new Sphere(Vec3d(0.5, 0.5, 0.5), 0.4);
    }

    unsigned int resolution = 50;
    Volume<bool> volume(1.2 / resolution, Vec3d(-0.1, -0.1, -0.1), Vec3d(1.1, 1.1, 1.1), resolution);
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
    Object object = Object::owl;
    std::string name;
    switch (object) {
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
    std::string outputFile = "./results/silhouette_" + name + ".off";
    Volume<bool> volume = generate_point_cloud(200, 0.1);
    voxel_carve(&volume, ("../01_data_acquisition/images/obj_" + name).data(), false, false);
    volume.writeToFile("./results/volume_" + name + ".txt");

    SimpleMesh mesh;
    ProjectedMarchingCubes marchingCubes(&volume, "../01_data_acquisition/images/obj_owl");
    //SimpleMarchingCubes marchingCubes(&volume);
    marchingCubes.processVolume(&mesh);

    if (!mesh.writeMesh(outputFile))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        return -1;
    }
    return 0;
}
