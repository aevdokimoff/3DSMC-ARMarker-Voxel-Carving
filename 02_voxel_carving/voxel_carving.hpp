#pragma once
#include "common.h"
#include "volume.h"

Volume voxel_carve(u32 res, f32 side_length, const char* path_to_runs, bool carve_in_parallel = false);
