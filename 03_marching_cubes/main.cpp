#include "simple_marching_cubes.h"
#include "surface/implicit_surface.h"
#include <string>
#include <chrono>

using namespace chrono;

#define USE_IMPLICIT_SURFACE false
#define TORUS true

#define OBJECT Object::duck
#define MARCHING_CUBES MarchingCubesType::simple

#define WITH_VOXEL_CARVING true
#define SAVE_VOLUME false
#define IN_PARALLEL true
#define SAVE_RESULT_IMAGE false

#define LOW_RESOLUTION false

enum Object {owl, duck, unicorn};
enum MarchingCubesType {simple, threshold, projected};

Volume getImplicitVolume(bool torus = true) {
    ImplicitSurface* surface;
    if (torus) {
        surface = new Torus(Vec3d(0.5, 0.5, 0.5), 0.4, 0.1);
    } else {
    	surface = new Sphere(Vec3d(0.5, 0.5, 0.5), 0.4);
    }

    unsigned int resolution = 50;
    Volume volume(Vec3d(-0.1, -0.1, -0.1), Vec3d(1.1, 1.1, 1.1), resolution);
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
    std::string algorithm;
    switch (MARCHING_CUBES)
    {
        case simple:
            algorithm = "simple_";
            break;
        case threshold:
            algorithm = "threshold_";
            break;
        case projected:
            algorithm = "projected_";
            break;
        default:
            cout << "Failed to interpret the algorithm";
            return 1;
    }
    switch (OBJECT)
    {
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

    std::string full_name = algorithm + name + (IN_PARALLEL ? "_parallel" : "") + (LOW_RESOLUTION ? "_low_resolution" : "");
    cout << full_name << endl;
    std::string outputFile = "./results/" + full_name + ".off";
    std::string timingOutput = "./results/" + full_name + ".txt";
    std::ofstream timingFile(timingOutput);
    if (!timingFile.is_open()) {
        cout << "Couldn't open file " + timingOutput + ".";
        return 1;
    }

    Volume volume;
    if (USE_IMPLICIT_SURFACE)
    {
        volume = getImplicitVolume(TORUS);
        name = TORUS ? "torus" : "sphere";
    }
    else if (WITH_VOXEL_CARVING)
    {
        int resolution = LOW_RESOLUTION ? 20 : 200;
        volume = generate_point_cloud(resolution, 0.13);
        auto start_voxel_carve = chrono::steady_clock::now();
        voxel_carve(&volume, ("../01_data_acquisition/images/obj_" + name).data(), IN_PARALLEL, SAVE_RESULT_IMAGE);
        auto end_voxel_carve = chrono::steady_clock::now();
        timingFile << "Voxel carving finished in: "
                << duration_cast<chrono::milliseconds>(end_voxel_carve - start_voxel_carve).count() << std::endl;

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
    MarchingCubes *marchingCubes;
    switch (MARCHING_CUBES)
    {
        case simple:
            marchingCubes = new SimpleMarchingCubes(&volume, IN_PARALLEL);
            break;
        case threshold:
            marchingCubes = new ThresholdMarchingCubes(&volume, 2, IN_PARALLEL);
            break;
        case projected:
            marchingCubes = new ProjectedMarchingCubes(&volume, "../01_data_acquisition/images/obj_" + name, IN_PARALLEL);
            break;
    }
    auto start_marching_cubes = chrono::steady_clock::now();
    marchingCubes->processVolume(&mesh);
    auto end_marching_cubes = chrono::steady_clock::now();
    timingFile << "Marching cubes finished in: "
               << duration_cast<chrono::milliseconds>(end_marching_cubes - start_marching_cubes).count() << std::endl;

    if (!mesh.writeMesh(outputFile))
    {
        std::cout << "ERROR: unable to write output file!" << std::endl;
        timingFile.close();
        return -1;
    }
    timingFile.close();

    return 0;
}
