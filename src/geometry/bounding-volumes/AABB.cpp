//
// Created by arthur on 16/03/2022.
//

#include "AABB.hpp"
#include "Sphere.hpp"
#include "../primitives/Triangle.hpp"
#include "../defs.hpp"
#include "../primitives/Plane.hpp"

const Point &AABB::getCenter() const noexcept
{
    return m_center;
}


AABB::AABB( const Point &point, float r_x, float r_y,float r_z) noexcept :
BoundingVolume(), m_center(point), m_radius(), m_mesh()
{
    m_radius[0] = r_x;
    m_radius[1] = r_y;
    m_radius[2] = r_z;

}

AABB::AABB(float x, float y, float z, float r_x, float r_y,float r_z) noexcept :
BoundingVolume(), m_center(Point{x,y,z}), m_radius({r_x,r_y,r_z}), m_mesh()
{}

AABB::AABB(const Sphere &sp) noexcept :
BoundingVolume(), m_center(sp.getCenter()),m_radius({sp.getRadius(), sp.getRadius(), sp.getRadius()}), m_mesh()
{}


AABB::AABB(const std::vector<Point> &pts) noexcept :
BoundingVolume(), m_center(), m_radius(), m_mesh()
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

bool AABB::intersectsBV(const AABB &aabb) const noexcept
{
    float r;
    float test;
    r = m_radius[0] + aabb.m_radius[0];
    test = abs(m_center[0] - aabb.m_center[0] );
    if( test > r  )
        return false;

    r = m_radius[1] + aabb.m_radius[1];
    test = abs(m_center[1] - aabb.m_center[1]);
    if(test > r )
        return false;

    r = m_radius[2] + aabb.m_radius[2];
    test = abs(m_center[2] - aabb.m_center[2]);
    if(test > r )
        return false;

    return true;
}

bool AABB::intersectsBV(const Sphere &sp) const noexcept {
    Point pt = closestPointToPoint(sp.getCenter());
    Vector toCenter = sp.getCenter() - pt;
    return norm2(toCenter) <= sp.getRadius()*sp.getRadius();
}

void AABB::update(Eigen::Matrix3f const& r, Vector const& t)
{
    if(!m_isMeshConstructed) return;

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
            m_radius[i] += abs( r(i,j) )*radius[j];
        }
    }
    constructMesh(true); // Reconstruct the mesh
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
    Point a = tri.getPoint(0);
    Point b = tri.getPoint(1);
    Point c = tri.getPoint(2);

    Point center = m_center;
    float r_x = m_radius[0];
    float r_y = m_radius[1];
    float r_z = m_radius[2];

    // Move AABB's center to the origin:
    a -= center;
    b -= center;
    c -= center;

    Vector ab = b - a;
    Vector bc = c - b;
    Vector ca = a - c;
    Vector n{};

    // Try finding a separating axes -> If we manage then they can't collide
    float p0,p1,p2,r;
    Vector u_x = {1, 0, 0};
    Vector u_y = {0, 1, 0};
    Vector u_z = {0, 0, 1};

    //
    // Axis 1 : (1,0,0)
    //

    // Test 1
    n = cross(u_x, ab);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    // If [-r,r]  and [min(p0,p1,p2),max(p0,p1,p2)] are disjoint then the Triangle and the AABB can't overlap
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Test 2
    n = cross(u_x, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Test 3
    n = cross(u_x, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;


    // Axis 2 : (0,1,0)

    n = cross(u_y, ab);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Test 2
    n = cross(u_y, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Test 3
    n = cross(u_y, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;


    // Axis 3 : (0,0,1)

    n = cross(u_z, ab);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0, p1,p2) < r) return false;

    // Test 2
    n = cross(u_z, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Test 3
    n = cross(u_z, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max3(p0,p1,p2) < -r || min3(p0,p1,p2) < r) return false;

    // Now if the 9 above tests could not prove that there were no intersection,
    // test the three axes corresponding to the face normals of the AABB

    // [-r_x, r_x] and [min(a.x, b.x, center.x), max(a.x,b.x, center.x)] don't overlap
    if(max3(a.x, b.x, c.x) < -r_x || min3(a.x, b.x, c.x) > r_x) return false;

    if(max3(a.y, b.y, c.y) < -r_y || min3(a.y, b.y, c.y) > r_y) return false;

    if(max3(a.z,b.z, c.z) < -r_z || min3(a.z, b.z, c.z) > r_z) return false;


    // Finally, test the axis corresponding to the triangle face normal
    Point normal = cross(ab, bc);
    float d = dot(normal, a);
    Plane p(normal,d);
    return intersects(p);
}

float AABB::getVolume() {
    assert(isDefined(m_center));
    return (float) (8*m_radius[0]*m_radius[1]*m_radius[2]);
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

void AABB::constructMesh()

{
    if(m_isMeshConstructed)
        return;

    constructMesh(false);
}


void AABB::setColor(int r, int g, int b) const noexcept
{
    if(m_isMeshConstructed)
        m_mesh->setSurfaceColor({r,g,b});
}

void AABB::setVisible(bool isVisible) noexcept
{
    m_isVisible = isVisible;

    if(isVisible)
    {
        if(!m_isMeshConstructed)
            constructMesh();

        m_mesh->setTransparency(1);
    }
    else if(m_isMeshConstructed)
        m_mesh->setTransparency(0);
}

void AABB::updateMesh() noexcept
{
    if(!m_isMeshConstructed)
        constructMesh();

    m_mesh->refresh();
}

void AABB::update(const Vector &t)
{
    m_center += t;

    if(m_isMeshConstructed)
    {
        for(unsigned int i=0; i<m_mesh->nVertices(); i++)
        {
            m_mesh->vertices[i].x +=t.x;
            m_mesh->vertices[i].y +=t.y;
            m_mesh->vertices[i].z +=t.z;
        }
        m_mesh->refresh();
    }
    else
    {
        updateMesh();
    }


}

void AABB::constructMesh(bool update)
{
    std::vector<std::array<float,3>> vertices{};
    std::vector<std::vector<size_t>> faces{};
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
    vertices.push_back(v1.data());
    vertices.push_back(v2.data());
    vertices.push_back(v3.data());
    vertices.push_back(v4.data());
    vertices.push_back(v5.data());
    vertices.push_back(v6.data());
    vertices.push_back(v7.data());
    vertices.push_back(v8.data());

    std::vector<size_t> f1 = {0,1,2,3};
    std::vector<size_t> f2 = {1,4,7,2};
    std::vector<size_t> f3 = {4,5,6,7};
    std::vector<size_t> f4 = {5,0,3,6};
    std::vector<size_t> f5 = {3,2,7,6};
    std::vector<size_t> f6 = {5,4,1,0};
    faces.push_back(f1);
    faces.push_back(f2);
    faces.push_back(f3);
    faces.push_back(f4);
    faces.push_back(f5);
    faces.push_back(f6);


    if(update)
    {
        m_mesh->updateVertexPositions(vertices);
    }
    else
    {
        std::string name;
        name = genName("aabb",m_id);

        m_mesh =  polyscope::registerSurfaceMesh(name, vertices, faces);
        m_isMeshConstructed = true;
        setVisible(false);
    }

}

polyscope::SurfaceMesh *AABB::getMesh() {
    if(!m_isMeshConstructed)
        constructMesh();
    return m_mesh;
}

Vector AABB::getHalfWidth() {
    return Vector{m_radius[0],m_radius[1],m_radius[2]};
}

bool AABB::intersects(const BoundingVolume &boundingVolume) const noexcept {
    return boundingVolume.intersectsBV(*this);
}








