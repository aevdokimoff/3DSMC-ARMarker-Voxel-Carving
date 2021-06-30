#include "marching_cubes.h"

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
        Vertex v0((float)tris[i1].points[0][0], (float)tris[i1].points[0][1], (float)tris[i1].points[0][2]);
        Vertex v1((float)tris[i1].points[1][0], (float)tris[i1].points[1][1], (float)tris[i1].points[1][2]);
        Vertex v2((float)tris[i1].points[2][0], (float)tris[i1].points[2][1], (float)tris[i1].points[2][2]);

        unsigned int vhandle[3];
        vhandle[0] = mesh->addVertex(v0);
        vhandle[1] = mesh->addVertex(v1);
        vhandle[2] = mesh->addVertex(v2);

        mesh->addFace(vhandle[0], vhandle[1], vhandle[2]);
    }
}

Vec3d SimpleMarchingCubes::interpret(const Vec3d &p1, const Vec3d &p2)
{
    return p1 + (p2 - p1) / 2;
}

int SimpleMarchingCubes::polygonise(const GridCell<bool> &cell, Triangle* triangles)
{
    int ntriang;
    int cubeindex;
    Vec3d vertlist[12];

    cubeindex = 0;
    int degree = 1;
    for (bool value : cell.values) {
        if (!value) cubeindex |= degree;
        degree << 1;
    }

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0) return 0;

    degree = 1;
    for (int i = 0; i < 12; i++) {
        if (edgeTable[cubeindex] & degree)
            vertlist[i] = interpret(cell.corners[edges[i][0]], cell.corners[edges[i][1]]);
        degree << 1;
    }

    /* Create the triangle */
    ntriang = 0;
    for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
        triangles[ntriang].points[0] = vertlist[triTable[cubeindex][i]];
        triangles[ntriang].points[1] = vertlist[triTable[cubeindex][i + 1]];
        triangles[ntriang].points[2] = vertlist[triTable[cubeindex][i + 2]];
        ntriang++;
    }

    return ntriang;
}

ProjectedMarchingCubes::ProjectedMarchingCubes(Volume<bool> *_volume, string _dataPath) : MarchingCubes(_volume), dataPath(std::move(_dataPath))
{
    grid.resize(volume->getVoxelCnt());
}

void ProjectedMarchingCubes::processVolume(SimpleMesh* mesh)
{
    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                processVolumeCell(x, y, z, nullptr);
            }
        }
    }
    postProcessMesh(mesh);
}

void ProjectedMarchingCubes::processVolumeCell(int x, int y, int z, SimpleMesh* mesh)
{
    uint ind = volume->getPosFromTuple(x, y, z);
    fillCell(grid[ind], x, y, z);
    polygonise(grid[ind]);
}

void ProjectedMarchingCubes::polygonise(TriangulatedCell &cell)
{
    //iterate through images where p1 and p2 is not on the same side of an object
    //for each image:
    //  project voxel onto image, iterate through inner pixels
    //  for each cube side:
    //      for each inner pixel:
    //          intersect voxel with preimage of the pixel
    //      linear regression, compute segment
    //      intersect segment with border (for one vertex select the further point)

    int cubeindex = 0;
    int degree = 1;
    for (bool value : cell.values) {
        if (!value) cubeindex |= degree;
        degree <<= 1;
    }
    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0) return;

    degree = 1;
    for (auto & intersection : cell.intersections) {
        if (edgeTable[cubeindex] & degree) intersection = true;
        degree <<= 1;
    }

    processImages(cell, dataPath + "/run_1");
    processImages(cell, dataPath + "/run_2");

    //todo
}

void ProjectedMarchingCubes::processImages(TriangulatedCell &cell, const string& path)
{
    auto processImage = [&](const char* filePath, Matx44d viewMatrix, Matx44d projectionMatrix) {
        projectPixels(cell, filePath, viewMatrix, projectionMatrix);
    };

    Matx44d proj_mat = getProjectionMatrix();
    process_using_single_run(path.data(), proj_mat, false, processImage);
}

void ProjectedMarchingCubes::projectPixels(TriangulatedCell &cell, const char *filePath, Matx44d viewMatrix, Matx44d projectionMatrix)
{
    int width, height;
    int n;
    auto* pixels = (Pixel*)stbi_load(filePath, &width, &height, &n, 0);
    if (!pixels) {
        fprintf(stderr, "Error opening image: %s\n", filePath);
        return;
    }

    //Compute Bounding Box
    int leftBottomCornerX = width;
    int leftBottomCornerY = height;
    int rightUpperCornerX = 0;
    int rightUpperCornerY = 0;

    for (const auto &corner : cell.corners) {
        Vec3d projectedCorner = project_point_to_screen_space(corner, viewMatrix, projectionMatrix);

        int cornerX = (projectedCorner[0] + 1) / 2 * width; //todo (maybe) can be saved and not computed multiple times
        int cornerY = (projectedCorner[1] + 1) / 2 * height;
        leftBottomCornerX = min(leftBottomCornerX, cornerX);
        leftBottomCornerY = min(leftBottomCornerY, cornerY);
        rightUpperCornerX = max(rightUpperCornerX, cornerX);
        rightUpperCornerY = max(rightUpperCornerY, cornerY);
    }

    //
    for (int i  = 0; i < 6; i++) {
        for (int x = leftBottomCornerX; x <= rightUpperCornerX; x++) {
            for (int y = leftBottomCornerY; y <= rightUpperCornerY; y++) {
                Vec3d cameraPos(viewMatrix.get_minor<3, 1>(0, 3).val);

                //todo intersect ith side with preimage
            }
        }
    }
}

void ProjectedMarchingCubes::postProcessMesh(SimpleMesh *pMesh)
{
    //todo write mesh
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
