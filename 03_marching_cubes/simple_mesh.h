#ifndef SIMPLE_MESH_H
#define SIMPLE_MESH_H

#include <iostream>
#include <fstream>

#include <opencv2/core/mat.hpp>

using namespace cv;

/**
 * Vertex that is used in the Mesh class.
 */
typedef Vec3d Vertex;

/**
 * A class that defines one triangle of a mesh. Contains 3 values --
 *      indices of the corresponding vertices in the clockwise order.
 */
struct MeshTriangle
{
    unsigned int idx0;
    unsigned int idx1;
    unsigned int idx2;
    MeshTriangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) :
            idx0(_idx0), idx1(_idx1), idx2(_idx2)
    {}
};

/**
 * Class for storing and accessing a mesh structure and writing it to the off file.
 */
class SimpleMesh
{
public:

    void clear();

    unsigned int addVertex(Vertex& vertex);

    unsigned int addFace(unsigned int idx0, unsigned int idx1, unsigned int idx2);

    std::vector<Vertex>& getVertices()
    {
        return m_vertices;
    }

    std::vector<MeshTriangle>& getTriangles()
    {
        return m_triangles;
    }

    bool writeMesh(const std::string& filename);

private:
    std::vector<Vertex> m_vertices;
    std::vector<MeshTriangle> m_triangles;
};

#endif