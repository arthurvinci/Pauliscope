//
// Created by arthur on 24/03/2022.
//

#include "Object.hpp"

#include <utility>
#include "bounding-volumes/Sphere.hpp"
#include "bounding-volumes/AABB.hpp"
#include "geometrycentral/surface/meshio.h"
#include "primitives/Triangle.hpp"

#define BOUND 5

void Object::update(const Eigen::Matrix3f &r, const Vector &t)
{
    // Don't know yet
}

void Object::update(float dt)
{

    //std::vector<std::array<float,3>> vertices;
    for(unsigned int i=0; i<m_mesh->nVertices(); i++)
    {
        m_mesh->vertices[i].x +=m_speed.x*dt;
        m_mesh->vertices[i].y +=m_speed.y*dt;
        m_mesh->vertices[i].z +=m_speed.z*dt;
    }
    updateMesh();

   m_insideTree.update(m_speed*dt);

   // Check if bounds have been reached
   auto root = m_insideTree.getNodeAt(0)->getBoundingVolume();
   auto center = root->getCenter();
   if(abs(center.x) >= -BOUND)
       m_speed.x = -m_speed.x;
    if(abs(center.z) >= -BOUND)
        m_speed.z = -m_speed.z;
    if(center.y < 0 || center.y>=BOUND)
        m_speed.y = -m_speed.y;

}

void Object::constructMesh()
{
    // Mesh is already constructed at the instantiation
    assert(m_isMeshConstructed);
}

void Object::setColor(int r, int g, int b) const noexcept
{
    m_mesh->setSurfaceColor({r,g,b});
}

void Object::setVisible(bool isVisible) noexcept
{
    m_isVisible = isVisible;
    if(isVisible)
        m_mesh->setTransparency(1);
    else
        m_mesh->setTransparency(0);
}

void Object::updateMesh()
{
    m_mesh->refresh();
}

Object::Object(const std::string& path, Vector speed, std::string name):
Displayable(true,true, true),
m_insideTree(),
m_speed(speed)
{
    std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = geometrycentral::surface::readManifoldSurfaceMesh(path);

    // Making sure every face is a triangle
    for(auto face : mesh->faces())
    {
        mesh->triangulate(face);
    }

    m_mesh = polyscope::registerSurfaceMesh(std::move(name), geometry->inputVertexPositions, mesh->getFaceVertexList(), geometrycentral::surface::polyscopePermutations(*mesh));
    m_insideTree = BVOctree(getObjectBarycenter(m_mesh->vertices), m_mesh->vertices, m_mesh->faces);
}

Point Object::getObjectBarycenter(const std::vector<glm::vec3> &points) {
    Point retPoint{};
    for( auto pt : points)
    {
        retPoint.x += pt.x;
        retPoint.y += pt.y;
        retPoint.z += pt.z;
    }
    retPoint/=(points.size());
    return retPoint;
}

void Object::showOctree(bool show, int depthToShow) {
    m_insideTree.setVisible(show,depthToShow);
}

bool Object::intersects(Object &other) const {
    return m_insideTree.intersects(other.m_insideTree);
}

bool Object::rawIntersects(Object &other) const {
    auto vert1 = m_mesh->vertices;
    auto vert2 = other.m_mesh->vertices;
    for(auto face1 : m_mesh->faces)
    {
        for(auto face2 : other.m_mesh->faces)
        {
            Point p1 = Point{vert1[face1[0]].x, vert1[face1[0]].y, vert1[face1[0]].z};
            Point p2 = Point{vert1[face1[1]].x, vert1[face1[1]].y, vert1[face1[1]].z};
            Point p3 = Point{vert1[face1[2]].x, vert1[face1[2]].y, vert1[face1[2]].z};
            Triangle tri1 = Triangle( p1, p2, p3);

            Point p4 = Point{vert1[face2[0]].x, vert1[face2[0]].y, vert1[face2[0]].z};
            Point p5 = Point{vert1[face2[1]].x, vert1[face2[1]].y, vert1[face2[1]].z};
            Point p6 = Point{vert1[face2[2]].x, vert1[face2[2]].y, vert1[face2[2]].z};
            Triangle tri2 = Triangle( p4, p5, p6);

            if(tri1.intersects(tri2))
                return true;

        }
    }
    return false;
}


