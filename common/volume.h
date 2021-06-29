#ifndef VOLUME_H
#define VOLUME_H

#include <limits>
#include <opencv2/core/mat.hpp>
#include "common.h"

typedef unsigned int uint;

using namespace cv;

//! A regular volume dataset
class Volume
{
public:

	//! Initializes an empty volume dataset.
	Volume(const Vec3d& min_, const Vec3d& max_, uint resolution = 10);

	~Volume();

	inline void computeMinMaxValues(double& minVal, double& maxVal) const
	{
		minVal = std::numeric_limits<double>::max();
		maxVal = -minVal;
		for (uint i1 = 0; i1 < dx * dy * dz; i1++)
		{
			if (minVal > vol[i1]) minVal = vol[i1];
			if (maxVal < vol[i1]) maxVal = vol[i1];
		}
	}

	//! Computes spacing in x,y,z-directions.
	void compute_ddx_dddx();

	//! Set the value at i.
	inline void set(uint i, double val)
	{
		if (val > maxValue)
			maxValue = val;

		if (val < minValue)
			minValue = val;

		vol[i] = val;
	}

	//! Set the value at (x_, y_, z_).
	inline void set(uint x_, uint y_, uint z_, double val)
	{
        if (val > maxValue)
            maxValue = val;

        if (val < minValue)
            minValue = val;

        vol[getPosFromTuple(x_, y_, z_)] = val;
	};

	//! Get the value at (x_, y_, z_).
	inline double get(uint i) const
	{
		return vol[i];
	};

	//! Get the value at (x_, y_, z_).
	inline double get(uint x_, uint y_, uint z_) const
	{
		return vol[getPosFromTuple(x_, y_, z_)];
	};

	//! Get the value at (pos.x, pos.y, pos.z).
	inline double get(const Vec3i& pos_) const
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

	//! Returns the Data.
	double* getData() const;

	//! Sets all entries in the volume to '0'
	void clean();

	//! Returns number of cells in x-dir.
	inline uint getDimX() const { return dx; }

	//! Returns number of cells in y-dir.
	inline uint getDimY() const { return dy; }

	//! Returns number of cells in z-dir.
	inline uint getDimZ() const { return dz; }

	inline Vec3d getMin() { return v_min; }
	inline Vec3d getMax() { return v_max; }

	//! Sets minimum extension
	void SetMin(const Vec3d& min_);

	//! Sets maximum extension
	void SetMax(const Vec3d& max_);

	inline uint getPosFromTuple(int x, int y, int z) const
	{
		return x * dy * dz + y * dz + z;
	}


	//! Lower left and Upper right corner.
	Vec3d v_min, v_max;

	//! max-min
	Vec3d diag;

	double ddx, ddy, ddz;
	double dddx, dddy, dddz;

	//! Number of cells in x, y and z-direction.
	uint dx, dy, dz;

	double* vol;

	double maxValue, minValue;
private:

	//! x,y,z access to vol*
	inline double vol_access(int x, int y, int z) const
	{
		return vol[getPosFromTuple(x, y, z)];
	}
};

void print(const Volume& volume);

#endif
