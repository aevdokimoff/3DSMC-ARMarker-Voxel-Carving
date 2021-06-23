
clang++ main.cpp -fopenmp -O3 -o voxel || exit 1
time ./voxel || exit 1
