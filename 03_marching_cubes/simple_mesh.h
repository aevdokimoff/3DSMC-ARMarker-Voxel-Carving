#ifndef SIMPLE_MESH_H
#define SIMPLE_MESH_H

#include <iostream>
#include <fstream>

#include <opencv2/core/mat.hpp>

using namespace cv;

typedef Vec3f Vertex;

struct Triangle
{
    unsigned int idx0;
    unsigned int idx1;
    unsigned int idx2;
    Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) :
            idx0(_idx0), idx1(_idx1), idx2(_idx2)
    {}
};

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

    std::vector<Triangle>& getTriangles()
    {
        return m_triangles;
    }

    bool writeMesh(const std::string& filename);

private:
    std::vector<Vertex> m_vertices;
    std::vector<Triangle> m_triangles;
};

#endif