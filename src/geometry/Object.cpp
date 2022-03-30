//
// Created by arthur on 24/03/2022.
//

#include "Object.hpp"

#include <utility>
#include "bounding-volumes/Sphere.hpp"
#include "bounding-volumes/AABB.hpp"
#include "geometrycentral/surface/meshio.h"

BoundingVolume *Object::bestFittingBV(const std::vector<glm::vec3> &points) noexcept
{
    assert(!points.empty());

    std::vector<Point> pts{};
    for(auto pt_arr : points)
    {
        Point new_pt{pt_arr[0], pt_arr[1], pt_arr[2]};
        pts.push_back(new_pt);
    }
    // Compute every BoundingVolume possible and then compare volume/computations needed for an intersection check
    auto aabb = new AABB(pts);
    auto aabb_vol = aabb->getVolume();
    auto sp = new Sphere(pts);
    auto sp_vol = sp->getVolume();

    if(sp_vol>aabb_vol)
        return aabb;
    else
        return sp;
}

BoundingVolume *Object::getBoundingVolume() const noexcept {
    return m_boundingVolume;
}

void Object::update(const Eigen::Matrix3f &r, const Vector &t)
{
    // Don't know yet
}

void Object::update(const Vector &t)
{

    //std::vector<std::array<float,3>> vertices;
    for(unsigned int i=0; i<m_mesh->nVertices(); i++)
    {
        m_mesh->vertices[i].x +=t.x;
        m_mesh->vertices[i].y +=t.y;
        m_mesh->vertices[i].z +=t.z;
    }
    updateMesh();
    m_boundingVolume->update(t);
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

Object::Object(const std::string& path, std::string name): Displayable(true,true, true)
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
    m_boundingVolume = bestFittingBV(m_mesh->vertices);
}

Object::Object(std::string path, bvID boundingVolume, std::string name) : Displayable(true, true, true)
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

    std::vector<Point> pts{};
    for(auto pt_arr : m_mesh->vertices)
    {
        Point new_pt{pt_arr[0], pt_arr[1], pt_arr[2]};
        pts.push_back(new_pt);
    }

    switch (boundingVolume)
    {

        case SPHERE_ID:
            m_boundingVolume = new Sphere(pts);
            break;
        case AABB_ID:
            m_boundingVolume = new AABB(pts);
            break;
    }
}

Object::Object(polyscope::SurfaceMesh *surfaceMesh) noexcept : Displayable(true,true,true)
{
    m_mesh = surfaceMesh;
    m_boundingVolume = bestFittingBV(m_mesh->vertices);
}


