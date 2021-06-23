#include "simple_mesh.h"

void SimpleMesh::clear()
{
    m_vertices.clear();
    m_triangles.clear();
}

unsigned int SimpleMesh::addVertex(Vertex& vertex)
{
    auto vId = (unsigned int)m_vertices.size();
    m_vertices.push_back(vertex);
    return vId;
}

unsigned int SimpleMesh::addFace(unsigned int idx0, unsigned int idx1, unsigned int idx2)
{
    auto fId = (unsigned int)m_triangles.size();
    Triangle triangle(idx0, idx1, idx2);
    m_triangles.push_back(triangle);
    return fId;
}

bool SimpleMesh::writeMesh(const std::string& filename)
{
    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;

    // write header
    outFile << "OFF" << std::endl;
    outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

    // save vertices
    for (auto & m_vertice : m_vertices)
    {
        outFile << m_vertice(0) << " " << m_vertice(1) << " " << m_vertice(2) << std::endl;
    }

    // save faces
    for (auto & m_triangle : m_triangles)
    {
        outFile << "3 " << m_triangle.idx0 << " " << m_triangle.idx1 << " " << m_triangle.idx2 << std::endl;
    }

    // close file
    outFile.close();

    return true;
}
