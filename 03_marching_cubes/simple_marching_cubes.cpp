#include <omp.h>
#include "simple_marching_cubes.h"

SimpleMarchingCubes::SimpleMarchingCubes(Volume<bool> *_volume) : MarchingCubes(_volume) {}

void SimpleMarchingCubes::processVolume(SimpleMesh* mesh)
{
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
    fillMesh(mesh);
}

void SimpleMarchingCubes::processVolumeCell(int x, int y, int z)
{
    uint ind = volume->getPosFromTuple(x, y, z);
    fillCell(grid[ind], x, y, z);

    polygonise(grid[ind]);
}

Vec3d SimpleMarchingCubes::interpret(const Vec3d &p1, const Vec3d &p2)
{
    return p1 + (p2 - p1) / 2;
}

void SimpleMarchingCubes::polygonise(TriangulatedCell &cell)
{
    int degree = 1;
    for (bool value : cell.values) {
        if (!value) cell.cubeIndex |= degree;
        degree <<= 1;
    }

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cell.cubeIndex] == 0) return;
    degree = 1;
    for (int i = 0; i < 12; i++) {
        if (edgeTable[cell.cubeIndex] & degree) {
            cell.hasIntersection[i] = true;
            cell.intersections[i] = interpret(cell.corners[edges[i][0]], cell.corners[edges[i][1]]);
        }
        degree <<= 1;
    }
}

void MarchingCubes::fillCell(GridCell<bool> &cell, int x, int y, int z) {
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

    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                uint ind = volume->getPosFromTuple(x, y, z);
                int considered_edges[] = {7, 6, 11};
                for (auto i: considered_edges)
                {
                    if (grid[ind].hasIntersection[i])
                    {
                        uint vertex_index = mesh->addVertex(grid[ind].intersections[i]);
                        vertex_indices[pair<uint, int>(ind, i)] = vertex_index;
                        for (int j = 0; j < 3; j++)
                        {
                            int neighbourX = edgeNeighbours[i][j][0] + x;
                            int neighbourY = edgeNeighbours[i][j][1] + y;
                            int neighbourZ = edgeNeighbours[i][j][2] + z;
                            if (!volume->correctVoxel(neighbourX, neighbourY, neighbourZ)) continue;
                            uint next_ind = volume->getPosFromTuple(neighbourX, neighbourY, neighbourZ);
                            vertex_indices[pair<uint, int>(next_ind, edgeNeighbours[i][j][3])] = vertex_index;
                        }
                    }
                }
            }
        }
    }

    cout << "Added all vertices to the mesh.\n";

    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                uint ind = volume->getPosFromTuple(x, y, z);
                for (int i = 0; triTable[grid[ind].cubeIndex][i] != -1; i += 3) {
                    uint ind3 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i])];
                    uint ind2 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i + 1])];
                    uint ind1 = vertex_indices[pair<uint, int>(ind, triTable[grid[ind].cubeIndex][i + 2])];
                    mesh->addFace(ind1, ind2, ind3);
                }
            }
        }
    }
}
