#include "marching_cubes.h"

bool MarchingCubes::processVolume(double iso, SimpleMesh* mesh)
{
    for (int x = 0; x < vol->getDimX() - 1; x++)
    {
        for (int y = 0; y < vol->getDimY() - 1; y++)
        {
            for (int z = 0; z < vol->getDimZ() - 1; z++)
            {
                processVolumeCell(x, y, z, iso, mesh);
            }
        }
    }

    return true;
}

SimpleMarchingCubes::SimpleMarchingCubes(Volume *_vol) : MarchingCubes(_vol) {}

bool SimpleMarchingCubes::processVolumeCell(int x, int y, int z, double iso, SimpleMesh* mesh)
{
    MC_Gridcell cell;
    Vec3d tmp;

    // cell corners
    tmp = vol->pos(x + 1, y, z);
    cell.p[0] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x, y, z);
    cell.p[1] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x, y + 1, z);
    cell.p[2] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x + 1, y + 1, z);
    cell.p[3] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x + 1, y, z + 1);
    cell.p[4] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x, y, z + 1);
    cell.p[5] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x, y + 1, z + 1);
    cell.p[6] = Vec3d(tmp[0], tmp[1], tmp[2]);
    tmp = vol->pos(x + 1, y + 1, z + 1);
    cell.p[7] = Vec3d(tmp[0], tmp[1], tmp[2]);

    // cell corner values
    cell.val[0] = (double)vol->get(x + 1, y, z);
    cell.val[1] = (double)vol->get(x, y, z);
    cell.val[2] = (double)vol->get(x, y + 1, z);
    cell.val[3] = (double)vol->get(x + 1, y + 1, z);
    cell.val[4] = (double)vol->get(x + 1, y, z + 1);
    cell.val[5] = (double)vol->get(x, y, z + 1);
    cell.val[6] = (double)vol->get(x, y + 1, z + 1);
    cell.val[7] = (double)vol->get(x + 1, y + 1, z + 1);

    MC_Triangle tris[6];
    int numTris = polygonise(cell, iso, tris);

    if (numTris == 0)
        return false;

    for (int i1 = 0; i1 < numTris; i1++)
    {
        Vertex v0((float)tris[i1].p[0][0], (float)tris[i1].p[0][1], (float)tris[i1].p[0][2]);
        Vertex v1((float)tris[i1].p[1][0], (float)tris[i1].p[1][1], (float)tris[i1].p[1][2]);
        Vertex v2((float)tris[i1].p[2][0], (float)tris[i1].p[2][1], (float)tris[i1].p[2][2]);

        unsigned int vhandle[3];
        vhandle[0] = mesh->addVertex(v0);
        vhandle[1] = mesh->addVertex(v1);
        vhandle[2] = mesh->addVertex(v2);

        mesh->addFace(vhandle[0], vhandle[1], vhandle[2]);
    }

    return true;
}

Vec3d SimpleMarchingCubes::interpret(double isolevel, const Vec3d& p1, const Vec3d& p2, double valp1, double valp2)
{
    return p1 + (p2 - p1) / 2;
}

int SimpleMarchingCubes::polygonise(MC_Gridcell grid, double isolevel, MC_Triangle* triangles)
{
    int ntriang;
    int cubeindex;
    Vec3d vertlist[12];

    cubeindex = 0;
    if (grid.val[0] < isolevel) cubeindex |= 1;
    if (grid.val[1] < isolevel) cubeindex |= 2;
    if (grid.val[2] < isolevel) cubeindex |= 4;
    if (grid.val[3] < isolevel) cubeindex |= 8;
    if (grid.val[4] < isolevel) cubeindex |= 16;
    if (grid.val[5] < isolevel) cubeindex |= 32;
    if (grid.val[6] < isolevel) cubeindex |= 64;
    if (grid.val[7] < isolevel) cubeindex |= 128;

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0)
        return 0;

    /* Find the vertices where the surface intersects the cube */
    if (edgeTable[cubeindex] & 1)
        vertlist[0] = interpret(isolevel, grid.p[0], grid.p[1], grid.val[0], grid.val[1]);
    if (edgeTable[cubeindex] & 2)
        vertlist[1] = interpret(isolevel, grid.p[1], grid.p[2], grid.val[1], grid.val[2]);
    if (edgeTable[cubeindex] & 4)
        vertlist[2] = interpret(isolevel, grid.p[2], grid.p[3], grid.val[2], grid.val[3]);
    if (edgeTable[cubeindex] & 8)
        vertlist[3] = interpret(isolevel, grid.p[3], grid.p[0], grid.val[3], grid.val[0]);
    if (edgeTable[cubeindex] & 16)
        vertlist[4] = interpret(isolevel, grid.p[4], grid.p[5], grid.val[4], grid.val[5]);
    if (edgeTable[cubeindex] & 32)
        vertlist[5] = interpret(isolevel, grid.p[5], grid.p[6], grid.val[5], grid.val[6]);
    if (edgeTable[cubeindex] & 64)
        vertlist[6] = interpret(isolevel, grid.p[6], grid.p[7], grid.val[6], grid.val[7]);
    if (edgeTable[cubeindex] & 128)
        vertlist[7] = interpret(isolevel, grid.p[7], grid.p[4], grid.val[7], grid.val[4]);
    if (edgeTable[cubeindex] & 256)
        vertlist[8] = interpret(isolevel, grid.p[0], grid.p[4], grid.val[0], grid.val[4]);
    if (edgeTable[cubeindex] & 512)
        vertlist[9] = interpret(isolevel, grid.p[1], grid.p[5], grid.val[1], grid.val[5]);
    if (edgeTable[cubeindex] & 1024)
        vertlist[10] = interpret(isolevel, grid.p[2], grid.p[6], grid.val[2], grid.val[6]);
    if (edgeTable[cubeindex] & 2048)
        vertlist[11] = interpret(isolevel, grid.p[3], grid.p[7], grid.val[3], grid.val[7]);

    /* Create the triangle */
    ntriang = 0;
    for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
        triangles[ntriang].p[0] = vertlist[triTable[cubeindex][i]];
        triangles[ntriang].p[1] = vertlist[triTable[cubeindex][i + 1]];
        triangles[ntriang].p[2] = vertlist[triTable[cubeindex][i + 2]];
        ntriang++;
    }

    return ntriang;
}