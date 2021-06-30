#include "common.h"
#include <cstring>

Matx33d generate_z_rot_mat(f32 angle) {
    Matx33d mat;
    mat(0, 0) =  cos(angle);
    mat(0, 1) = -sin(angle);
    mat(1, 0) =  sin(angle);
    mat(1, 1) =  cos(angle);
    mat(2, 2) = 1;

    return mat;
}