//
// Created by arthur on 16/03/2022.
//

#include "AABB.hpp"
#include "Sphere.hpp"
#include "polyscope/surface_mesh.h"
#include "../primitives/Triangle.hpp"

const Point &AABB::getCenter() const noexcept
{
    return m_center;
}


AABB::AABB( const Point &point, float r_x, float r_y,float r_z) noexcept : BoundingVolume(), m_center(point)
{
    m_radius[0] = r_x;
    m_radius[1] = r_y;
    m_radius[2] = r_z;

}

AABB::AABB(float x, float y, float z, float r_x, float r_y,float r_z) noexcept : BoundingVolume(), m_center(Point{x,y,z}), m_radius({r_x,r_y,r_z})
{}

AABB::AABB(const Sphere &sp) noexcept : BoundingVolume(), m_center(sp.getCenter()),
                                        m_radius({sp.getRadius(), sp.getRadius(), sp.getRadius()})
{}


AABB::AABB(const std::vector<Point> &pts) noexcept : BoundingVolume()
{
    assert(!pts.empty());
    m_center = Point::zero();
    m_radius = {0,0,0};

    for(int i = 0; i<3; i++)
    {
        Vector u = Vector::zero();
        u[i] = 1;
        unsigned int indexMin = 0;
        unsigned int indexMax = 0;
        extremePointsAlongDirection(u,pts, &indexMin, &indexMax);

        m_center[i] = (pts[indexMax][i] + pts[indexMin][i])/2;
        float radius_f = (pts[indexMax][i] - pts[indexMin][i])/2;
        m_radius[i] = radius_f;
    }
}

bool AABB::intersects(const Intersectable &intersectable) const noexcept
{
    return intersectable.intersects(*this);
}

bool AABB::intersects(const AABB &aabb) const noexcept
{
    float r;
    r = m_radius[0] + aabb.m_radius[0];
    if(std::abs(m_center[0] - aabb.m_center[0] ) > r  ) return false;

    r = m_radius[1] + aabb.m_radius[1];
    if(std::abs(m_center[1] - aabb.m_center[1] ) > r ) return false;

    r = m_radius[2] + aabb.m_radius[2];
    if(std::abs(m_center[2] - aabb.m_center[2] ) > r ) return false;

    return true;
}

bool AABB::intersects(const Sphere &sp) const noexcept {
    return false;
}

void AABB::update(Eigen::Matrix3f const& r, Vector const& t)
{
    float radius[3];
    std::copy(std::begin(m_radius), std::end(m_radius), std::begin(radius)) ;
    auto center = m_center;

    for(int i = 0; i<3; i++)
    {
        m_center[i] = t[i];
        m_radius[i] = 0;
        for(int j =0; j<3; j++)
        {
            m_center[i] += r(i,j)*center[j];
            m_radius[i] += std::abs( r(i,j) )*radius[j];
        }
    }

    m_needsMeshUpdate = true;
}

 std::array<float, 3> AABB::getRadius() const noexcept {
    return m_radius;
}

void AABB::extremePointsAlongDirection(Vector direction, std::vector<Point> const& pts, unsigned int *indexMin,
                                       unsigned int *indexMax)
{
    float min = MAXFLOAT, max = -MAXFLOAT;

    for(unsigned int i = 0; i < pts.size(); i++)
    {
        float proj = dot(pts[i], direction);
        if(proj < min)
        {
            min = proj;
            *indexMin = i;
        }

        if(proj > max)
        {
            max = proj;
            *indexMax = i;
        }
    }

}

bool AABB::intersects(const Plane &plane) const noexcept {
    assert(false);
    return false;
}

bool AABB::intersects(const Triangle &tri) const noexcept {
    return tri.intersects(*this);
}

float AABB::getVolume() {
    assert(isDefined(m_center));
    return (float) (8*m_radius[0]*m_radius[1]*m_radius[2]);
}

void AABB::constructMesh()
{
    m_meshVertices.clear();
    m_meshFaces.clear();
        /*
     * Order of the vertices of the cube:
     *         7+--------+8
     *         /        /|
     *        /        / |
     *      4+--------+3 |
     *       |        |  |
     *       |   6    |  +5
     *       |        | /
     *       |        |/
     *     1 +--------+ 2
     */
        Point v1 = {m_center[0]-m_radius[0], m_center[1]-m_radius[1], m_center[2] + m_radius[2]};
        Point v2 = {m_center[0]+m_radius[0], m_center[1]-m_radius[1], m_center[2] + m_radius[2]};
        Point v3 = {m_center[0]+m_radius[0], m_center[1]+m_radius[1], m_center[2] + m_radius[2]};
        Point v4 = {m_center[0]-m_radius[0], m_center[1]+m_radius[1], m_center[2] + m_radius[2]};
        Point v5 = {m_center[0]+m_radius[0], m_center[1]-m_radius[1], m_center[2] - m_radius[2]};
        Point v6 = {m_center[0]-m_radius[0], m_center[1]-m_radius[1], m_center[2] - m_radius[2]};
        Point v7 = {m_center[0]-m_radius[0], m_center[1]+m_radius[1], m_center[2] - m_radius[2]};
        Point v8 = {m_center[0]+m_radius[0], m_center[1]+m_radius[1], m_center[2] - m_radius[2]};
        m_meshVertices.push_back(v1.data());
        m_meshVertices.push_back(v2.data());
        m_meshVertices.push_back(v3.data());
        m_meshVertices.push_back(v4.data());
        m_meshVertices.push_back(v5.data());
        m_meshVertices.push_back(v6.data());
        m_meshVertices.push_back(v7.data());
        m_meshVertices.push_back(v8.data());

        std::vector<unsigned int> f1 = {0,1,2,3};
        std::vector<unsigned int> f2 = {1,4,7,2};
        std::vector<unsigned int> f3 = {4,5,6,7};
        std::vector<unsigned int> f4 = {5,0,3,6};
        std::vector<unsigned int> f5 = {3,2,7,6};
        std::vector<unsigned int> f6 = {5,4,1,0};
        m_meshFaces.push_back(f1);
        m_meshFaces.push_back(f2);
        m_meshFaces.push_back(f3);
        m_meshFaces.push_back(f4);
        m_meshFaces.push_back(f5);
        m_meshFaces.push_back(f6);

}

Point AABB::closestPointToPoint(const Point &p) const noexcept
{
    assert(p.isDefined());

    Point q{};
    // For each coordinate, if the coordinate is inside the box do nothing. Otherwise, push the point to the box.
    for(int i =0; i<3; i++)
    {
        float coord = p[i];
        float min_coord = m_center[i] - m_radius[i];
        float max_coord = m_center[i] + m_radius[i];
        if( coord < min_coord ) coord = min_coord;
        if( coord > max_coord ) coord = max_coord;

        q[i] = coord;
    }

    return q;
}

float AABB::distanceToPoint(const Point &p) const noexcept
{
    assert(p.isDefined());

    float dist2 = 0;
    // For each axis compute excess distance from the box

    for(int i = 0; i<3; i++)
    {
        float coord = p[i];
        float min_coord = m_center[i] - m_radius[i];
        float max_coord = m_center[i] + m_radius[i];
        if( coord < min_coord ) dist2 += (min_coord-coord)*(min_coord-coord);
        if( coord > max_coord) dist2  += (max_coord-coord)*(max_coord-coord);
    }

    return sqrtf(dist2);

}








