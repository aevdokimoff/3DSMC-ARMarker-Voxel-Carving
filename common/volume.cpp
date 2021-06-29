#include "volume.h"
#include <cmath>

Volume::Volume(const Vec3d& min_, const Vec3d& max_, uint resolution)
{
    v_min = min_;
    v_max = max_;
	diag = v_max - v_min;
	dx = resolution;
	dy = resolution;
	dz = resolution;
	vol = new double[dx * dy * dz];

	compute_ddx_dddx();
}

Volume::~Volume()
{
	delete[] vol;
};


//! Computes spacing in x,y,z-directions.
void Volume::compute_ddx_dddx()
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

//! Returns the Data.
double* Volume::getData() const
{
	return vol;
};

//! Sets all entries in the volume to '0'
void Volume::clean()
{
	for (uint i1 = 0; i1 < dx * dy * dz; i1++) vol[i1] = double(0.0);
}

//! Sets minimum extension
void Volume::SetMin(const Vec3d& min_)
{
    v_min = min_;
	diag = v_max - v_min;
}

//! Sets maximum extension
void Volume::SetMax(const Vec3d& max_)
{
    v_max = max_;
	diag = v_max - v_min;
}

void print(const Volume& volume) {
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
