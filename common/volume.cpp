#include "volume.h"
#include <cmath>
#include <fstream>

template <typename T>
Volume<T>::Volume(double _sideLength, const Vec3d& min_, const Vec3d& max_, uint resolution) :
    sideLength(sideLength), v_min(min_), v_max(max_)
{
    diag = v_max - v_min;
	dx = resolution;
	dy = resolution;
	dz = resolution;
	vol = std::vector<T>((dx + 1) * (dy + 1) * (dz + 1));

	compute_ddx_dddx();
}


//! Computes spacing in x,y,z-directions.
template <typename T>
void Volume<T>::compute_ddx_dddx()
{
	ddx = 1.0f / (dx - 1);
	ddy = 1.0f / (dy - 1);
	ddz = 1.0f / (dz - 1);

	dddx = (v_max[0] - v_min[0]) / (dx - 1);
	dddy = (v_max[1] - v_min[1]) / (dy - 1);
	dddz = (v_max[2] - v_min[2]) / (dz - 1);

	if (dz == 1)
	{
		ddz = 0;
		dddz = 0;
	}

	diag = v_max - v_min;
}

//! Sets minimum extension
template <typename T>
void Volume<T>::SetMin(const Vec3d& min_)
{
    v_min = min_;
	diag = v_max - v_min;
}

//! Sets maximum extension
template <typename T>
void Volume<T>::SetMax(const Vec3d& max_)
{
    v_max = max_;
	diag = v_max - v_min;
}

template <typename T>
bool Volume<T>::correctVoxel(int x, int y, int z)
{
    return (x >= 0) && (y >= 0) && (z >= 0) && (x < dx - 1) && (y < dy - 1) && (z < dz - 1);
}

template <typename T>
bool Volume<T>::writeToFile(const std::string &filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    outFile << v_min[0] << ' ' << v_min[1] << ' ' << v_min[2] << std::endl; // double
    outFile << v_max[0] << ' ' << v_max[1] << ' ' << v_max[2] << std::endl; // double
    outFile << diag[0] << ' ' << diag[1] << ' ' << diag[2] << std::endl; // double
    outFile << dx << ' ' << dy << ' ' << dz << std::endl; //uint
    outFile << ddx << ' ' << ddy << ' ' << ddz << std::endl; //double
    outFile << dddx << ' ' << dddy << ' ' << dddz << std::endl; //double
    outFile << sideLength << ' ' << minValue << ' ' << maxValue << std::endl; //double
    outFile << vol.size() << std::endl; // int
    for (auto value: vol) outFile << value << ' '; //bool

    outFile.close();
    return true;
}

template <typename T>
bool Volume<T>::writePointCloudToFile(const std::string &filename)
{
    uint size = 0;
    for (int x = 0; x <= dx; x++)
        for (int y = 0; y <= dy; y++)
            for (int z = 0; z <= dz; z++)
                if (vol[getPosFromTuple(x, y, z)])
                    size++;
    std::ofstream outfile(filename);
    outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
    outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << size << "\n";
    outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
    outfile << "property list uchar int vertex_indices\n" << "end_header\n";
    for (int x = 0; x <= dx; x++)
        for (int y = 0; y <= dy; y++)
            for (int z = 0; z <= dz; z++)
                if (vol[getPosFromTuple(x, y, z)])
                    outfile << posX(x) << " " << posY(y) << " " << posZ(z) << std::endl;
    outfile.close();
}

template <typename T>
bool Volume<T>::readFromFile(const std::string &filename)
{
    std::ifstream inFile(filename);
    if (!inFile.is_open()) return false;

    inFile >> v_min[0] >> v_min[1] >> v_min[2]; // double
    inFile >> v_max[0] >> v_max[1] >> v_max[2]; // double
    inFile >> diag[0] >> diag[1] >> diag[2]; // double
    inFile >> dx >> dy >> dz; //uint
    inFile >> ddx >> ddy >> ddz; //double
    inFile >> dddx >> dddy >> dddz; //double
    inFile >> sideLength >> minValue >> maxValue; //double
    int size;
    inFile >> size;
    for (int i = 0; i < size; i++)
    {
        int value;
        inFile >> value;
        vol.push_back(value);
    }

    inFile.close();
    return true;
}

template <typename T>
void print(const Volume<T>& volume) {
    for (u32 z = 0; z < volume.dz; ++z) {
        for (u32 y = 0; y < volume.dy; ++y) {
            for (u32 x = 0; x < volume.dx; ++x) {
                Vec3d position = volume.pos(x, y, z);
                printf("Voxel: % 3.2f % 3.2f % 3.2f\n",
                       position[0],
                       position[1],
                       position[2]);
            }
        }
    }
}
