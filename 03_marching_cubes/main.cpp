#include "marching_cubes.h"
#include "surface/implicit_surface.h"
#include <string>

int main(int argc, char *argv[]) {
    ImplicitSurface* surface;
//	surface = new Sphere(Vec3d(0.5, 0.5, 0.5), 0.4);
//	std::string outputfile = "test_sphere.off";
    surface = new Torus(Vec3d(0.5, 0.5, 0.5), 0.4, 0.1);
    std::string outputFile = "test_torus.off";

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
                volume.set(x, y, z, val);
            }
        }
    }

    SimpleMesh mesh;
    SimpleMarchingCubes marchingCubes(&volume);
    marchingCubes.processVolume(&mesh);

    if (!mesh.writeMesh(outputFile))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        return -1;
    }

    delete surface;

    return 0;
}
