//
// Created by arthur on 26/03/2022.
//

#include "Triangle.hpp"
#include "../bounding-volumes/Sphere.hpp"
#include "../bounding-volumes/AABB.hpp"
#include "Plane.hpp"
#include "tritri_macros.hpp"

Triangle::Triangle(Point &p1,Point &p2,Point &p3) : Displayable(false,false, false),
m_pts()
{
    m_pts[0] = p1;
    m_pts[1] = p2;
    m_pts[2] = p3;
}


bool Triangle::intersects(const Triangle &tri) const noexcept
{
    float V0[3] = {m_pts[0].x, m_pts[0].y, m_pts[0].z};
    float V1[3] = {m_pts[1].x, m_pts[1].y, m_pts[1].z};
    float V2[3] = {m_pts[2].x, m_pts[2].y, m_pts[2].z};

    float U0[3] = {tri.m_pts[0].x, tri.m_pts[0].y, tri.m_pts[0].z};
    float U1[3] = {tri.m_pts[1].x, tri.m_pts[1].y, tri.m_pts[1].z};
    float U2[3] = {tri.m_pts[2].x, tri.m_pts[2].y, tri.m_pts[2].z};


    return NoDivTriTriIsect(V0,V1,V2, U0,U1,U2);
}


bool Triangle::intersects(const Plane &plane) const noexcept
{
    // Not yet implemented
    assert(false);
    return false;
}

Point Triangle::closestPointToPoint(const Point &p) const noexcept {

    /*      A /\
     *       /  \
     *      /    \
     *    B/______\C
     */
    Point a = m_pts[0];
    Point b = m_pts[1];
    Point c = m_pts[2];

    // If P is in Voronoi region for vertex A -> return A
    Vector ab = b-a;
    Vector ac = c-a;
    Vector ap = p-a;
    float abap = dot(ab, ap);
    float acap = dot(ac, ap);
    if(abap <= 0 && acap <= 0) return a;


    // If P is in Voronoi region for vertex B -> return B
    Vector bp = p-b;
    float abbp = dot(ab, bp);
    float acbp = dot(ac, bp);
    if(abbp >= 0 && acbp <= abbp) return b;

    // If P is none of above but still between vertices A and B -> Project on segment AB
    float vc = abap * acbp - abbp * acap;
    if (vc <= 0 && abap >= 0 && abbp <= 0)
    {
        float v = abap / (abap - abbp);
        return a + v*ab;
    }

    // If P is in Voronoi region for vertex C -> return C
    Vector cp = p - c;
    float abcp = dot(ab, cp);
    float accp = dot(ac, cp);
    if(accp >= 0 && abcp <= accp) return c;

    // If P is none above but still between vertices A and C -> Project on segment AC
    float vb = abcp * acap - abap * accp;
    if(vb <= 0 && acap >= 0 && accp <= 0)
    {
        float w = acap / (acap - accp);
        return a + w*ac;
    }

    // If P is none above but still between vertices B and C -> Project on segment BC
    float va = abbp*accp - abcp*acbp;
    if( va <=0 && (acbp-abbp) >= 0 && (abcp-accp) >=0)
    {
        float w = (acbp-abbp)/( acbp-abbp + abcp-accp );
        return b + w*(c-b);
    }

    // Otherwise, the point is inside the triangle and return the closest point on the border;
    float denom = 1/(va+vb+vc);
    float v = vb*denom;
    float w = vc*denom;
    return a+ab*v +ac*w;
}

void Triangle::constructMesh()
{
    std::vector<std::array<float,3>> vertices;
    std::vector<std::vector<size_t>> faces;
    float value = 0;
    for(int i=0; i<3; i++)
    {
        Point p = m_pts[i];
        vertices.push_back({p.x, p.y, p.z});
        value+= p.x + p.y + p.z;
    }
    faces.push_back({0,1,2});
    std::string name = std::to_string(value);
    m_mesh = polyscope::registerSurfaceMesh(name, vertices, faces);
    m_needsMeshUpdate = false;

}

bool Triangle::intersects(const Intersectable &intersectable) const noexcept {
    return intersectable.intersects(*this);
}

Vector Triangle::getPoint(unsigned int i) const {
    assert(i<3);
    return m_pts[i];
}

bool Triangle::intersects(const BoundingVolume &boundingVolume) const noexcept {
    return boundingVolume.intersects(*this);
}

void Triangle::setColor(int r, int g, int b) const noexcept
{
    m_mesh->setSurfaceColor({r,g,b});
}

void Triangle::setVisible(bool isVisible) noexcept {
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

void Triangle::updateMesh()
{
    m_mesh->refresh();
}
