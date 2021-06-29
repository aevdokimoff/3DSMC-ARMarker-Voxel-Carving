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
    Volume vol(Vec3d(-0.1,-0.1,-0.1), Vec3d(1.1,1.1,1.1), resolution, resolution, resolution, 1);
    for (int x = 0; x < vol.getDimX(); x++)
    {
        for (int y = 0; y < vol.getDimY(); y++)
        {
            for (int z = 0; z < vol.getDimZ(); z++)
            {
                Vec3d p = vol.pos(x, y, z);
                double val = surface->Eval(p);
                vol.set(x,y,z, val);
            }
        }
    }

    SimpleMesh mesh;
    SimpleMarchingCubes marchingCubes(&vol);
    marchingCubes.processVolume(0.0f, &mesh);

    if (!mesh.writeMesh(outputFile))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        return -1;
    }

    delete surface;

    return 0;
}
