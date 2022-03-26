//
// Created by arthur on 24/03/2022.
//

#include "Object.hpp"
#include "bounding-volumes/Sphere.hpp"
#include "bounding-volumes/AABB.hpp"


Object::Object(const geometrycentral::surface::VertexData<geometrycentral::Vector3> &vertexData) noexcept
{
    std::vector<Point> vertices{};
    for(unsigned int i = 0; i<vertexData.size(); i++)
    {
        Point np{};
        for(short j=0; j<3; j++)
            np[j] = (float)vertexData[i][j];

        vertices.push_back(np);
    }

    m_vertices = vertices;

    m_boundingVolume = bestFittingBV(vertices);
}

Object::Object(const geometrycentral::surface::VertexData<geometrycentral::Vector3> &vertexData, bvID bvId) noexcept
{
    std::vector<Point> vertices{};
    for(unsigned int i = 0; i<vertexData.size(); i++)
    {
        Point np{};
        for(short j=0; j<3; j++)
            np[j] = (float)vertexData[i][j];

        vertices.push_back(np);
    }

    m_vertices = vertices;

    switch (bvId)
    {
        case SPHERE_ID:
            m_boundingVolume = new Sphere(m_vertices);
            break;

        case AABB_ID:
            m_boundingVolume = new AABB(m_vertices);
            break;
    }

}

Object::Object(const std::vector<std::array<float, 3>> &vertices) noexcept
{
    std::vector<Point> verts{};
    for(auto vertice : vertices)
    {
        Point np{};
        for(short j=0; j<3; j++)
            np[j] = (float)vertice[j];

        verts.push_back(np);
    }

    m_vertices = verts;

    m_boundingVolume = bestFittingBV(verts);

}

Object::Object(const std::vector<std::array<float, 3>> &vertices, bvID bvId) noexcept
{
    std::vector<Point> verts{};
    for(auto vertice : vertices)
    {
        Point np{};
        for(short j=0; j<3; j++)
            np[j] = (float)vertice[j];

        verts.push_back(np);
    }

    m_vertices = verts;


    switch (bvId)
    {
        case SPHERE_ID:
            m_boundingVolume = new Sphere(m_vertices);
            break;

        case AABB_ID:
            m_boundingVolume = new AABB(m_vertices);
            break;
    }
}


BoundingVolume *Object::bestFittingBV(const std::vector<Point> &points) noexcept
{
    assert(!points.empty());

    // Compute every BoundingVolume possible and then compare volume/computations needed for an intersection check
    auto aabb = new AABB(points);
    auto aabb_vol = aabb->getVolume();
    auto sp = new Sphere(points);
    auto sp_vol = sp->getVolume();

    if(sp_vol>aabb_vol)
        return aabb;
    else
        return sp;
}

BoundingVolume *Object::getBoundingVolume() const noexcept {
    return m_boundingVolume;
}

Object::Object(BoundingVolume *bv) noexcept
{
    m_boundingVolume = bv;
    std::vector<std::array<float,3>> vertices = bv->getMeshVertices();
    for(auto v : vertices)
    {
        Point p{v[0], v[1], v[2]};
        m_vertices.push_back(p);
    }
}


