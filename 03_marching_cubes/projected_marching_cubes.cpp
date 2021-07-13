#include "simple_marching_cubes.h"

#include <chrono> // debug
#include <omp.h>

using namespace chrono; // debug

ProjectedMarchingCubes::ProjectedMarchingCubes(Volume<bool> *_volume, string _dataPath, bool _in_parallel) : MarchingCubes(_volume, _in_parallel), dataPath(std::move(_dataPath)) {}

void ProjectedMarchingCubes::processVolume(SimpleMesh *pMesh)
{
    //iterate through images where p1 and p2 is not on the same side of an object
    //for each image:
    //  project voxel onto image, iterate through inner pixels
    //  for each cube side:
    //      for each inner pixel:
    //          intersect voxel with preimage of the pixel
    //      linear regression, compute segment
    //      intersect segment with border (for one vertex select the further point)

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

    processImages(dataPath + "/run_1");
    processImages(dataPath + "/run_2");

    #pragma omp parallel for num_threads(thread_count)
    for (auto & ind : grid) {
        defineSurfaceLines(ind);
    }

    #pragma omp parallel for num_threads(thread_count)
    for (int x = 0; x < volume->getDimX() - 1; x++)
    {
        for (int y = 0; y < volume->getDimY() - 1; y++)
        {
            for (int z = 0; z < volume->getDimZ() - 1; z++)
            {
                postProcessVolumeCell(x, y, z);
            }
        }
    }

    fillMesh(pMesh);
}

void ProjectedMarchingCubes::processVolumeCell(int x, int y, int z)
{
    uint ind = volume->getPosFromTuple(x, y, z);
    fillCell(grid[ind], x, y, z);
    markVertices(grid[ind]);
}

void ProjectedMarchingCubes::markVertices(TriangulatedCell &cell)
{
    int degree = 1;
    for (bool value : cell.values) {
        if (!value) cell.cubeIndex |= degree;
        degree <<= 1;
    }
    /* Cube is entirely in/out of the surface */
    if (edgeTable[cell.cubeIndex] == 0) return;

    degree = 1;
    for (auto & hasIntersection : cell.hasIntersection) {
        if (edgeTable[cell.cubeIndex] & degree) hasIntersection = true;
        degree <<= 1;
    }
}

Vec3d randomBetween(Vec3d &point1, const Vec3d &point2) {
    return point1 + (point2 - point1) * ((double) rand() / (RAND_MAX));
}

Vec3d furtherFrom(Vec3d &origin, Vec3d &point1, const Vec3d &point2) {
    double dist1 = norm(point1, origin);
    double dist2 = norm(point2, origin);
    assert(dist1 <= 1 || dist2 <= 1);
    if (dist1 > 1) return point2;
    if (dist2 > 1) return point1;
    return dist1 > dist2 ? point1 : point2;
}

void ProjectedMarchingCubes::defineSurfaceLines(TriangulatedCell &cell) {
    //  iterate through sides
    //  for each side
    //    if 1 or 3 :
    //      compute one line
    //    else if 2:
    //      split in two halves
    //      compute two lines

    for (int i = 0; i < 6; i++) {
        int numberOfInVertices = 0;
        for (int j = 0; j < 4; j++) numberOfInVertices += cell.values[faces[i][j]];
        switch (numberOfInVertices) {
            case 1:
            case 3:
            {
                int edge1 = cell.hasIntersection[facesByEdges[i][0]] ? 0 : 2;
                int edge2 = cell.hasIntersection[facesByEdges[i][1]] ? 1 : 3;
                computeSplitLine(cell, i, edge1, edge2);
                break;
            }
            case 2:
            {
                bool oppositeVerticesInside = true;
                for (int j = 0; j < 4; j++) oppositeVerticesInside &= cell.hasIntersection[facesByEdges[i][j]];
                if (oppositeVerticesInside) {
                    //todo  split in halves, compute two lines
                    for (int j = 0; j < 4; j++)
                    {
                        int edge = facesByEdges[i][j];
                        Vec3d insideVertex = cell.values[edges[edge][0]] ? cell.corners[edges[edge][0]] : cell.corners[edges[edge][1]];
                        cell.intersections[edge] = furtherFrom(insideVertex, cell.intersections[edge], randomBetween(cell.corners[edges[edge][0]], cell.corners[edges[edge][1]]));
                    }
                } else {
                    int edge = cell.hasIntersection[facesByEdges[i][0]] ? 0 : 2;
                    computeSplitLine(cell, i, edge, edge + 2);
                }
                break;
            }
            default:
                continue;
        }
    }
}

void ProjectedMarchingCubes::computeSplitLine(TriangulatedCell &cell, int face, int edge1, int edge2)
{
    //todo redo
    Vec3d insideVertex1 = cell.values[edges[edge1][0]] ? cell.corners[edges[edge1][0]] : cell.corners[edges[edge1][1]];
    cell.intersections[edge1] = furtherFrom(insideVertex1, cell.intersections[edge1], randomBetween(cell.corners[edges[edge1][0]], cell.corners[edges[edge1][1]]));
    Vec3d insideVertex2 = cell.values[edges[edge2][0]] ? cell.corners[edges[edge2][0]] : cell.corners[edges[edge2][1]];
    cell.intersections[edge2] = furtherFrom(insideVertex1, cell.intersections[edge2], randomBetween(cell.corners[edges[edge2][0]], cell.corners[edges[edge2][1]]));
}

void ProjectedMarchingCubes::processImages(const string& path)
{
    Matx44d proj_mat = getProjectionMatrix();
    auto start = chrono::steady_clock::now(); // debug

    auto processImage = [&](const char* filePath, uint ind, Matx44d viewMatrix, Matx44d projectionMatrix) {
        for (int i = 0; i < grid.size(); i++)
        {
            projectPixels(grid[i], filePath, ind, viewMatrix, projectionMatrix);
            if (i % 100 == 0) { // debug
                auto end = chrono::steady_clock::now();
                cout << "pixel " << i << " out of " << grid.size()
                     << " computed in " << duration_cast<chrono::seconds>(end - start).count() << " sec" << endl;
            }
        }
    };

    process_using_single_run(path.data(), proj_mat, false, processImage);
}

void ProjectedMarchingCubes::projectPixels(TriangulatedCell &cell, const char *filePath, uint ind, Matx44d viewMatrix, Matx44d projectionMatrix)
{
    Image image = load_image(filePath);

    //Compute Bounding Box
    int leftBottomCornerX = image.width;
    int leftBottomCornerY = image.height;
    int rightUpperCornerX = 0;
    int rightUpperCornerY = 0;

    for (const auto &corner : cell.corners) {
        Vec2i projection = volume->projections[volume->getInd(corner)][ind];

        leftBottomCornerX = min(leftBottomCornerX, projection[0]);
        leftBottomCornerY = min(leftBottomCornerY, projection[1]);
        rightUpperCornerX = max(rightUpperCornerX, projection[0]);
        rightUpperCornerY = max(rightUpperCornerY, projection[1]);
    }

    Vec3d cameraPos(viewMatrix.get_minor<3, 1>(0, 3).val);

    for (int i = 1; i < 6; i += 2) // iterate through voxel faces only once
    {
        for (int x = leftBottomCornerX; x <= rightUpperCornerX; x += 10)
        {
            for (int y = leftBottomCornerY; y <= rightUpperCornerY; y += 10)
            {
                if (image.at(x, y).r >= 150) continue;

                Vec3d screenPos(2.0f * (float) x / (float) image.width - 1, 2.0f * (float) y / (float) image.height - 1, 1);
                Vec3d rayVector = project_screen_point_to_3d(screenPos, viewMatrix, projectionMatrix);
                Vec3d pos3d = cameraPos + rayVector;

                Vec3d intersection;
                switch (i) {
                    case 1:
                        if (rayVector[2] != 0)
                        {
                            intersection[2] = cell.corners[faces[i][7]][2];
                            double tmp = (pos3d[2] - intersection[2]) / rayVector[2];
                            intersection[1] = pos3d[1] - rayVector[1] * tmp;
                            intersection[0] = pos3d[0] - rayVector[0] * tmp;
                        }
                        break;
                    case 3:
                        if (rayVector[1] != 0)
                        {
                            intersection[1] = cell.corners[faces[i][7]][1];
                            double tmp = (pos3d[1] - intersection[1]) / rayVector[1];
                            intersection[2] = pos3d[2] - rayVector[2] * tmp;
                            intersection[0] = pos3d[0] - rayVector[0] * tmp;
                        }
                        break;
                    default:
                        if (rayVector[0] != 0)
                        {
                            intersection[0] = cell.corners[faces[i][7]][0];
                            double tmp = (pos3d[0] - intersection[0]) / rayVector[0];
                            intersection[2] = pos3d[2] - rayVector[2] * tmp;
                            intersection[1] = pos3d[1] - rayVector[1] * tmp;
                        }
                }
                if (intersection[2] >= cell.corners[faces[i][1]][2] && intersection[2] <= cell.corners[faces[i][7]][2] &&
                        intersection[1] >= cell.corners[faces[i][1]][1] && intersection[1] <= cell.corners[faces[i][7]][1] &&
                        intersection[0] >= cell.corners[faces[i][1]][0] && intersection[0] <= cell.corners[faces[i][7]][0]) {
                    unique_lock<std::mutex> lock(marching_cubes_mutex);
                    cell.sideIntersections->push_back(intersection);
                }
            }
        }
    }
}

void ProjectedMarchingCubes::postProcessVolumeCell(int x, int y, int z)
{
    uint ind = volume->getPosFromTuple(x, y, z);

    // for each edge
    // for each neighbour
    // choose the furthest point
    // write it

    for (int i = 0; triTable[grid[ind].cubeIndex][i] != -1; i += 3)
    {
        int edge = triTable[grid[ind].cubeIndex][i];

        for (int j = 0; j < 3; j++)
        {
            int neighbourX = edgeNeighbours[i][j][0] + x;
            int neighbourY = edgeNeighbours[i][j][1] + y;
            int neighbourZ = edgeNeighbours[i][j][2] + z;
            if (!volume->correctVoxel(neighbourX, neighbourY, neighbourZ)) continue;
            int neighbourInd = volume -> getPosFromTuple(neighbourX, neighbourY, neighbourZ);
            int neighbourEdge = edgeNeighbours[i][j][3];

            Vec3d insideVertex = grid[ind].values[edges[edge][0]] ? grid[ind].corners[edges[edge][0]] : grid[ind].corners[edges[edge][1]];
            unique_lock<std::mutex> lock(marching_cubes_mutex);
            grid[ind].intersections[edge] = furtherFrom(insideVertex, grid[ind].intersections[edge], grid[neighbourInd].intersections[neighbourEdge]);
            grid[neighbourInd].intersections[neighbourEdge] = grid[ind].intersections[edge];
        }
    }
}