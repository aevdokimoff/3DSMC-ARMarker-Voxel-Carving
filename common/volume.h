#ifndef VOLUME_H
#define VOLUME_H

#include <limits>
#include <opencv2/core/mat.hpp>
#include "common.h"
#include <vector>
#include <mutex>

typedef unsigned int uint;

using namespace cv;

/**
 * A class that stores the info about the 3D square point grid.
 * Constructor takes 2 points: one with the minimal possible coordinates, second one with the maximal,
 *  and number of spaces between points.
 */
class Volume
{
public:
    Volume() = default;
    Volume(const Vec3d &min_, const Vec3d &max_, uint resolution);

	//! Computes spacing in x,y,z-directions.
	void compute_ddx_dddx();

	//! Set the value at (x_, y_, z_).
	inline void set(uint x_, uint y_, uint z_, int val)
	{
        vol[getPosFromTuple(x_, y_, z_)] = val;
	};

	//! Get the value at (x_, y_, z_).
	inline int get(uint i) const
	{
		return vol[i];
	};

	//! Get number of voxels
	inline uint getVoxelCnt() const
	{
        return dx * dy * dz;
	}

	//! Get the value at (x_, y_, z_).
	inline int get(uint x_, uint y_, uint z_) const
	{
		return vol[getPosFromTuple(x_, y_, z_)];
	};

	//! Get the value at (pos.x, pos.y, pos.z).
	inline int get(const Vec3i& pos_) const
	{
		return(get(pos_[0], pos_[1], pos_[2]));
	}

	//! Returns the cartesian x-coordinates of node (i,..).
	inline double posX(int i) const
	{
		return v_min[0] + diag[0] * (double(i) * ddx);
	}

	//! Returns the cartesian y-coordinates of node (..,i,..).
	inline double posY(int i) const
	{
		return v_min[1] + diag[1] * (double(i) * ddy);
	}

	//! Returns the cartesian z-coordinates of node (..,i).
	inline double posZ(int i) const
	{
		return v_min[2] + diag[2] * (double(i) * ddz);
	}

	//! Returns the cartesian coordinates of node (i,j,k).
	inline Vec3d pos(int i, int j, int k) const
	{
		return Vec3d(posX(i), posY(j), posZ(k));
	}

	//! Returns number of cells in x-dir.
	inline uint getDimX() const { return dx; }

	//! Returns number of cells in y-dir.
	inline uint getDimY() const { return dy; }

	//! Returns number of cells in z-dir.
	inline uint getDimZ() const { return dz; }

	inline Vec3d getMin() { return v_min; }
	inline Vec3d getMax() { return v_max; }

	inline uint getInd(const Vec3d &point)
    {
	    return getPosFromTuple((point[0] - v_min[0]) * dx / (v_max[0] - v_min[0]),
                            (point[1] - v_min[1]) * dy / (v_max[1] - v_min[1]),
                            (point[2] - v_min[2]) * dz / (v_max[2] - v_min[2]));
    }

	//! Sets minimum extension
	void SetMin(const Vec3d& min_);

	//! Sets maximum extension
	void SetMax(const Vec3d& max_);

	inline uint getPosFromTuple(int x, int y, int z) const
	{
		return x * dy * dz + y * dz + z;
	}

    inline uint getPosFromTuple(const Vec3i &index) const
    {
        return index[0] * dy * dz + index[1] * dz + index[2];
    }

	//! Lower left and Upper right corner.
	Vec3d v_min, v_max;

	//! max-min
	Vec3d diag;

	double ddx, ddy, ddz;
	double dddx, dddy, dddz;

	//! Number of cells in x, y and z-direction.
	uint dx, dy, dz;
	std::vector<int> vol;
	std::vector<Vec3i> surface_indices;
	double sideLength;

    bool correctVoxel(int x, int y, int z);

    bool writeToFile(const std::string &filename);

    bool readFromFile(const std::string &filename);

    bool writePointCloudToFile(const std::string &filename);
private:

	//! x,y,z access to vol
	inline int vol_access(int x, int y, int z) const
	{
		return vol[getPosFromTuple(x, y, z)];
	}

};

void print(const Volume &volume);

#endif
