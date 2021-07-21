#include <omp.h>
#include "simple_marching_cubes.h"

ThresholdMarchingCubes::ThresholdMarchingCubes(Volume *_volume, int threshold, bool _in_parallel) :
        SimpleMarchingCubes(_volume, _in_parallel), threshold(threshold)
{}

Vec3d ThresholdMarchingCubes::interpret(const Vec3d &p1, const Vec3d &p2, int value1, int value2)
{
    return p1 + (p2 - p1) * value1 / (value1 + value2);
}

bool ThresholdMarchingCubes::isPointOutside(int value)
{
    return value > threshold;
}

SimpleMarchingCubes::SimpleMarchingCubes(Volume *_volume, bool _in_parallel) : MarchingCubes(_volume, _in_parallel) {}

void SimpleMarchingCubes::processVolume(SimpleMesh *mesh)
{
    cout << "Started the algorithm.\n";
    u32 thread_count = (in_parallel) ? omp_get_max_threads() : 1;

    #pragma omp parallel for num_threads(thread_count)
    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                processVolumeCell(x, y, z);
            }
        }
    }
    cout << "Finished the algorithm.\n";
    fillMesh(mesh);
}

void SimpleMarchingCubes::processVolumeCell(int x, int y, int z)
{
    uint ind = volume->getPosFromTuple(x, y, z);
    fillCell(grid[ind], x, y, z);

    polygonise(grid[ind]);

    if (edgeTable[grid[ind].cubeIndex] != 0)
    {
        unique_lock<std::mutex> lock(marching_cubes_mutex);
        volume->surface_indices.emplace_back(x, y, z);
    }
}

Vec3d SimpleMarchingCubes::interpret(const Vec3d &p1, const Vec3d &p2, int value1, int value2)
{
    return p1 + (p2 - p1) / 2;
}

bool SimpleMarchingCubes::isPointOutside(int value)
{
    return value > 0;
}

void SimpleMarchingCubes::polygonise(TriangulatedCell &cell)
{
    int degree = 1;
    for (int value : cell.values) {
        if (isPointOutside(value)) cell.cubeIndex |= degree;
        degree <<= 1;
    }

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cell.cubeIndex] == 0) return;
    degree = 1;
    for (int i = 0; i < 12; i++) {
        if (edgeTable[cell.cubeIndex] & degree) {
            cell.hasIntersection[i] = true;
            cell.intersections[i] = interpret(cell.corners[edges[i][0]], cell.corners[edges[i][1]], cell.values[edges[i][0]], cell.values[edges[i][1]]);
        }
        degree <<= 1;
    }
}

void MarchingCubes::fillCell(GridCell &cell, int x, int y, int z) {
    // cell corners
    cell.corners[0] = volume->pos(x + 1, y, z);
    cell.corners[1] = volume->pos(x, y, z);
    cell.corners[2] = volume->pos(x, y + 1, z);
    cell.corners[3] = volume->pos(x + 1, y + 1, z);
    cell.corners[4] = volume->pos(x + 1, y, z + 1);
    cell.corners[5] = volume->pos(x, y, z + 1);
    cell.corners[6] = volume->pos(x, y + 1, z + 1);
    cell.corners[7] = volume->pos(x + 1, y + 1, z + 1);

    // cell corner values
    cell.values[0] = volume->get(x + 1, y, z);
    cell.values[1] = volume->get(x, y, z);
    cell.values[2] = volume->get(x, y + 1, z);
    cell.values[3] = volume->get(x + 1, y + 1, z);
    cell.values[4] = volume->get(x + 1, y, z + 1);
    cell.values[5] = volume->get(x, y, z + 1);
    cell.values[6] = volume->get(x, y + 1, z + 1);
    cell.values[7] = volume->get(x + 1, y + 1, z + 1);
}

void MarchingCubes::fillMesh(SimpleMesh *mesh)
{
    map<pair<uint, int>, uint> vertex_indices;

    u32 thread_count = (in_parallel) ? omp_get_max_threads() : 1;
    int considered_edges[] = {7, 6, 11};

    int surface_cells_cnt = volume->surface_indices.size();
    #pragma omp parallel for num_threads(thread_count)
    for (int surface_cell_ind = 0; surface_cell_ind < surface_cells_cnt; surface_cell_ind++) {
        Vec3i index = volume->surface_indices[surface_cell_ind];
        uint ind = volume->getPosFromTuple(index);
        for (auto i: considered_edges)
        {
            if (grid[ind].hasIntersection[i])
            {
                uint vertex_index;
                {
                    unique_lock<std::mutex> lock(marching_cubes_mutex);
                    vertex_index = mesh->addVertex(grid[ind].intersections[i]);
                    vertex_indices[pair<uint, int>(ind, i)] = vertex_index;
                }

                for (int j = 0; j < 3; j++)
                {
                    int neighbourX = edgeNeighbours[i][j][0] + index[0];
                    int neighbourY = edgeNeighbours[i][j][1] + index[1];
                    int neighbourZ = edgeNeighbours[i][j][2] + index[2];
                    if (!volume->correctVoxel(neighbourX, neighbourY, neighbourZ)) continue;
                    uint next_ind = volume->getPosFromTuple(neighbourX, neighbourY, neighbourZ);
                    unique_lock<std::mutex> lock(marching_cubes_mutex);
                    vertex_indices[pair<uint, int>(next_ind, edgeNeighbours[i][j][3])] = vertex_index;
                }
            }
        }
    }

    cout << "Added all vertices to the mesh.\n";

    #pragma omp parallel for num_threads(thread_count)
    for (int surface_cell_ind = 0; surface_cell_ind < surface_cells_cnt; surface_cell_ind++) {
        uint ind = volume->getPosFromTuple(volume->surface_indices[surface_cell_ind]);

        for (int i = 0; triTable[grid[ind].cubeIndex][i] != -1; i += 3) {
            uint ind3 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i])];
            uint ind2 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i + 1])];
            uint ind1 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i + 2])];

            unique_lock<std::mutex> lock(marching_cubes_mutex);
            mesh->addFace(ind1, ind2, ind3);
        }
    }
}
