#include "simple_marching_cubes.h"

void SimpleMarchingCubes::processVolume(SimpleMesh* mesh)
{
    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                processVolumeCell(x, y, z, mesh);
            }
        }
    }
}

SimpleMarchingCubes::SimpleMarchingCubes(Volume<bool> *_volume) : MarchingCubes(_volume) {}

void SimpleMarchingCubes::processVolumeCell(int x, int y, int z, SimpleMesh* mesh)
{
    GridCell<bool> cell;
    fillCell(cell, x, y, z);

    Triangle tris[6];
    int numTris = polygonise(cell, tris);

    if (numTris == 0) return;

    for (int i1 = 0; i1 < numTris; i1++)
    {
        unsigned int vhandle[3];
        vhandle[0] = mesh->addVertex(tris[i1].points[0]);
        vhandle[1] = mesh->addVertex(tris[i1].points[1]);
        vhandle[2] = mesh->addVertex(tris[i1].points[2]);
        mesh->addFace(vhandle[0], vhandle[2], vhandle[1]);
    }
}

Vec3d SimpleMarchingCubes::interpret(const Vec3d &p1, const Vec3d &p2)
{
    return p1 + (p2 - p1) / 2;
}

int SimpleMarchingCubes::polygonise(GridCell<bool> &cell, Triangle* triangles)
{
    int ntriang;
    Vec3d vertlist[12];

    int degree = 1;
    for (bool value : cell.values) {
        if (!value) cell.cubeIndex |= degree;
        degree <<= 1;
    }

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cell.cubeIndex] == 0) return 0;
    degree = 1;
    for (int i = 0; i < 12; i++) {
        if (edgeTable[cell.cubeIndex] & degree)
            vertlist[i] = interpret(cell.corners[edges[i][0]], cell.corners[edges[i][1]]);
        degree <<= 1;
    }

    /* Create the triangle */
    ntriang = 0;
    for (int i = 0; triTable[cell.cubeIndex][i] != -1; i += 3) {
        triangles[ntriang].points[0] = vertlist[triTable[cell.cubeIndex][i]];
        triangles[ntriang].points[1] = vertlist[triTable[cell.cubeIndex][i + 1]];
        triangles[ntriang].points[2] = vertlist[triTable[cell.cubeIndex][i + 2]];
        ntriang++;
    }

    return ntriang;
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
