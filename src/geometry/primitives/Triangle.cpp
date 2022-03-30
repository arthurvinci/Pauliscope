//
// Created by arthur on 26/03/2022.
//

#include "Triangle.hpp"
#include "../bounding-volumes/Sphere.hpp"
#include "../bounding-volumes/AABB.hpp"
#include "../../deps/eigen/eigen-git-mirror/blas/f2c/datatypes.h"
#include "Plane.hpp"
#include <memory>
#include <utility>


Triangle::Triangle(Point *p1,Point *p2,Point *p3, const std::string& name)
{
    m_pts[0] = p1;
    m_pts[1] = p2;
    m_pts[2] = p3;
}


bool Triangle::intersects(const Triangle &tri) const noexcept
{
    Point a = *m_pts[0];
    Point b = *m_pts[1];
    Point c = *m_pts[2];

    Point d = *tri.m_pts[0];
    Point e = *tri.m_pts[1];
    Point f = *tri.m_pts[2];

    // Check if the other triangle points are all on the same side of the plane where the first one is
    Vector normal1 = cross(e-d,f-d);
    float d1 = - dot(normal1,d);
    float t1 = dot(normal1,a) + d1;
    float t2 = dot(normal1,b) + d1;
    float t3 = dot(normal1,c) + d1;
    if( (t1>0 && t2 >0 && t3 >0) || (t1<0 && t2<0 && t3<0) ) return false;

    // Make the same test but reverse
    Vector normal2 = cross(b-a, c-a);
    float d2 = - dot(normal2, a);
    float t4 = dot(normal2,d) + d2;
    float t5 = dot(normal2,e) + d2;
    float t6 = dot(normal2,f) + d2;
    if( (t4>0 && t5>0 && t6>0) || (t4<0 && t5<0 && t6<0) ) return false;

    if(t1 == 0 && t2 == 0 && t3 == 0)
    {
        // Then the triangles are coplanar
        // Case not yet implemented
        assert(false);
    }

    // Else determine the line of intersection between the two planes
    Point o{};
    Vector dir{};
    Plane pl1(normal1,d1);
    Plane pl2(normal2,d2);
    std::tie(o,dir) = pl1.getIntersectionLine(pl2);

    // At this point, we know that both triangles intersect L due to previous computations
    // We compute intersection intervals for both triangle and see if they intersect

    // Intersection interval for t1

    // determine which point is on the other side of tri2_plane and switch everything to make it be a
    if( ( t2 >= 0 && t1>=0) || (t2<=0 && t1<=0) )
    {
        // Switch a and c
        Point tmp = c;
        float tmp_f = t3;

        c = a;
        t3 = t1;

        a = tmp;
        t1 = tmp_f;
    }
    else if( (t3>=0 && t1>=0) || (t3<=0 && t1<=0) )
    {
        // Switch a and b
        Point tmp = b;
        float tmp_f = t2;

        b = a;
        t2 = t1;

        a = tmp;
        t1= tmp_f;
    }

    float proja = dot(dir, a-o);
    float projb = dot(dir, b-o);
    float projc = dot(dir, c-o);

    float s1 = projb + (projb-proja)*t2/(t2-t1);
    float s2 = projc + (projc-proja)*t3/(t3-t1);


    // Do the same for the other triangle and make d the edge on the other side
    if( (t4 >=0 && t5>=0) || (t4<=0 && t5<=0) )
    {
        Point tmp = f;
        float tmp_f = t6;

        f = d;
        t6 = t4;

        d=tmp;
        t4 = tmp_f;
    }
    else if( (t4>=0 && t6>=0) || (t4<=0 && t6<=0) )
    {
        Point tmp = e;
        float tmp_f = t5;

        e = d;
        t5 = t4;

        d = tmp;
        t4 = tmp_f;
    }

    float projd = dot(dir,d);
    float proje = dot(dir,e);
    float projf = dot(dir,f);

    float s3 = proje + (proje-projd)*t5/(t5-t4);
    float s4 = projf + (projf-projd)*t6/(t6-t4);

    // Now compare [s1,s2] to [s3,s4]

    if( min(s1,s2) > max(s3,s4)) return false;
    if( min(s3,s4) > max(s1,s2)) return false;


    return true;
}

bool Triangle::intersects(const Sphere &sp) const noexcept {

    // Compute the closest point on triangle to the sphere center
    Point p = closestPointToPoint(sp.getCenter());

    // Compute square distance to sphere center and compare to square radius
    float dist2 = norm2(p-sp.getCenter());

    return dist2 <= sp.getRadius()*sp.getRadius();
}

bool Triangle::intersects(const AABB &aabb) const noexcept
{
    Point a = *m_pts[0];
    Point b = *m_pts[1];
    Point c = *m_pts[2];

    Point center = aabb.getCenter();
    float r_x = aabb.getRadius()[0];
    float r_y = aabb.getRadius()[1];
    float r_z = aabb.getRadius()[2];

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
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 2
    n = cross(u_x, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 3
    n = cross(u_x, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;


    // Axis 2 : (0,1,0)

    n = cross(u_y, ab);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 2
    n = cross(u_y, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 3
    n = cross(u_y, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;


    // Axis 3 : (0,0,1)

    n = cross(u_z, ab);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 2
    n = cross(u_z, bc);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Test 3
    n = cross(u_z, ca);
    p0 = dot(a, n);
    p1 = dot(b, n);
    p2 = dot(c, n);
    r = r_x * abs(dot(u_x, n)) + r_y * abs(dot(u_y, n)) + r_z * abs(dot(u_z, n));
    if( max(p0,max(p1,p2)) < -r || min(p0, min(p1,p2)) < r) return false;

    // Now if the 9 above tests could not prove that there were no intersection,
    // test the three axes corresponding to the face normals of the AABB

    // [-r_x, r_x] and [min(a.x, b.x, center.x), max(a.x,b.x, center.x)] don't overlap
    if(max(a.x, max(b.x, c.x)) < -r_x || min(a.x, min(b.x, c.x)) > r_x) return false;

    if(max(a.y, max(b.y, c.y)) < -r_x || min(a.y, min(b.y, c.y)) > r_x) return false;

    if(max(a.z, max(b.z, c.z)) < -r_x || min(a.z, min(b.z, c.z)) > r_x) return false;


    // Finally, test the axis corresponding to the triangle face normal
    Point normal = cross(ab, bc);
    float d = dot(normal, a);
    Plane p(normal,d);
    return aabb.intersects(p);
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
    Point a = *m_pts[0];
    Point b = *m_pts[1];
    Point c = *m_pts[2];

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
    /*
    for(int i=0; i<3; i++)
    {
        Point p = *m_pts[i];
        m_meshVertices.push_back({p.x, p.y, p.z});
    }
    m_meshFaces.push_back({0,1,2});

    m_needsMeshUpdate = false;
     */
}

bool Triangle::intersects(const Intersectable &intersectable) const noexcept {
    return intersectable.intersects(*this);
}

