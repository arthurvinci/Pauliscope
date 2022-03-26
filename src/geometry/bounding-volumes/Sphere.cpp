//
// Created by arthur on 16/03/2022.
//

#include <random>
#include <Eigen/Dense>
#include "Sphere.hpp"
#include "../defs.hpp"
#include "../primitives/Plane.hpp"
#include "../primitives/Triangle.hpp"


Sphere::Sphere(const Point &p, float radius, bool realInstance) : BoundingVolume(realInstance), m_center(p), m_radius(radius)
{
    assert(m_radius>=0);
}

const Point &Sphere::getCenter() const{
    return m_center;
}

float Sphere::getRadius() const{
    return m_radius;
}

bool Sphere::intersects(const Sphere &sp) const noexcept{
    Vector v = m_center - sp.m_center;
    float dist = norm2(v);

    float radius_sum = m_radius + sp.m_radius;
    return dist <= radius_sum*radius_sum;
}

Sphere::Sphere(const Point &p, bool realInstance) : BoundingVolume(realInstance), m_center(p), m_radius(0)
{}

Sphere::Sphere(const Point &p1, const Point &p2, bool realInstance) : BoundingVolume(realInstance)
{
    if(p1!=p2)
    {
        m_center = (p1+p2)/2;
        m_radius = distance(p1, p2) / 2;
    }
    else
    {
        Sphere s = Sphere(p1);
        m_center = s.m_center;
        m_radius = s.m_radius;
    }
}

Sphere::Sphere(const Point &p1, const Point &p2, const Point &p3, bool realInstance) : BoundingVolume(realInstance)
{


    if(areCollinear(p1, p2, p3))
    {
        Vector vec1 = p3 - p1;
        Vector vec2 = p2 - p1;
        float dot_prod = dot(vec1,vec2);

        if( dot_prod < 0 ) // It means that p1 is the middle point of the three points
        {
            Sphere(p2,p3);
        }
        else
        {
            Vector vec3 = p3-p2;

            if(dot(vec1,vec3)>0) // Then p2 is in the middle
            {
                Sphere(p1,p3);
            }
            else
            {
                Sphere(p1,p2);
            }
        }
    }
    else
    {
        if(norm2(p1) <= ACCEPTABLE_ERROR*ACCEPTABLE_ERROR)
        {
            // p1 is 0,0,0
            m_radius = (float)(norm(p3 - p2) / (2 * sin(angle(p2, p3))));
        }
        else if(norm2(p2) <= ACCEPTABLE_ERROR*ACCEPTABLE_ERROR)
        {
            // p2 is 0,0,0
            m_radius = (float)(norm(p3 - p1) / (2 * sin(angle(p1, p3))));
        }
        else
        {
            // place p3 as origin
            Vector a = p1 - p3;
            Vector b = p2 - p3;
            m_radius = (float)(norm(a - b) / (2 * sin(angle(a, b))));
        }


        float denoms = 1/(2*norm2(cross(p1-p2, p2-p3)));
        float alpha = norm2(p2-p3)*dot(p1-p2, p1-p3)*denoms;
        float beta = norm2(p1-p3)*dot(p2-p1,p2-p3)*denoms;
        float gamma = norm2(p1-p2)*dot(p3-p1, p3-p2)*denoms;

        m_center = alpha*p1 + beta*p2 + gamma*p3;
    }


}

Sphere::Sphere(const Point &p1, const Point &p2, const Point &p3, const Point &p4, bool realInstance) : BoundingVolume(realInstance)
{
    Eigen::Matrix<float, 4,4> a_mat= {};
    Eigen::Matrix<float, 4,4>  c_mat;
    Eigen::Matrix<float, 4,4>  d_x_mat;
    Eigen::Matrix<float, 4,4>  d_y_mat;
    Eigen::Matrix<float, 4,4>  d_z_mat;


    float p1_dot = dot(p1,p1);
    float p2_dot = dot(p2,p2);
    float p3_dot = dot(p3,p3);
    float p4_dot = dot(p4,p4);

    a_mat << p1.x, p1.y, p1.z, 1,
             p2.x, p2.y, p2.z, 1,
             p3.x, p3.y, p3.z, 1,
             p4.x, p4.y, p4.z, 1;

    c_mat << p1_dot, p1.x, p1.y, p1.z,
             p2_dot, p2.x, p2.y, p2.z,
             p3_dot, p3.x, p3.y, p3.z,
             p4_dot, p4.x, p4.y, p4.z;

    d_x_mat << p1_dot, p1.y, p1.z, 1,
               p2_dot, p2.y, p2.z, 1,
               p3_dot, p3.y, p3.z, 1,
               p4_dot, p4.y, p4.z, 1;

    d_y_mat <<  p1_dot, p1.x, p1.z, 1,
                p2_dot, p2.x, p2.z, 1,
                p3_dot, p3.x, p3.z, 1,
                p4_dot, p4.x, p4.z, 1;

    d_z_mat <<  p1_dot, p1.x, p1.y, 1,
                p2_dot, p2.x, p2.y, 1,
                p3_dot, p3.x, p3.y, 1,
                p4_dot, p4.x, p4.y, 1;


    float a = a_mat.determinant();
    float c = c_mat.determinant();
    float dx = d_x_mat.determinant();
    float dy = - d_y_mat.determinant();
    float dz = d_z_mat.determinant();

    // a=0 => the points are coplanar

    m_center = {dx/(2*a), dy/(2*a), dz/(2*a) };
    m_radius = sqrtf(dx*dx + dy*dy + dz*dz - 4*a*c)/(2* std::abs(a));
}

Sphere::Sphere(bool realInstance): BoundingVolume(realInstance),m_center(Point::undefined()), m_radius(0)
{}

Sphere::Sphere(const std::vector<Point> &points, bool realInstance) : BoundingVolume(realInstance)
{
    unsigned int nbPoints = points.size();
    std::vector<Point> pts(points);

    Sphere minSp = minimalSphere(pts,nbPoints,0,0);

    m_radius = minSp.m_radius;
    m_center = minSp.m_center;

    for(auto pt : points)
    {
        // For debugging purpose only. This should disappear when using release compilation option
        auto dist2 = norm2(m_center-pt);
        auto rad2 = m_radius*m_radius;
        assert( dist2 <= (rad2 + ACCEPTABLE_ERROR));
    }


}

bool Sphere::isInside(const Point &p)
{
    if( !m_center.isDefined() || !p.isDefined())
        return false;

    return norm2(p-m_center) <= m_radius*m_radius;
}


Sphere Sphere::minimalSphere(std::vector<Point> &pts, unsigned int nbPts, unsigned int start, unsigned int nbFound)
{
    Sphere minSphere;

    switch(nbFound)
    {
        case 0:
            minSphere =  Sphere(false);
            break;
        case 1:
            minSphere = Sphere(pts[start-1],false);
            break;
        case 2:
            minSphere =  Sphere(pts[start-1], pts[start-2], false);
            break;
        case 3:
            minSphere =  Sphere(pts[start-1], pts[start-2], pts[start-3], false);
            break;
        case 4:
            minSphere =  Sphere(pts[start-1], pts[start-2], pts[start-3], pts[start-4], false);
            return minSphere;
        default:
            minSphere =  Sphere(false); // This should not happen
    }

    for(unsigned int i=0; i<nbPts; i++)
    {
        Point current = pts[i+start];
        // If the chosen point is not in the current minimal sphere, add it to it
        if(!minSphere.isInside(current))
        {
            // Puts the new point at the sort of the vector
            for(unsigned int j = i+start; j > 0; j--)
            {
                Point temp = pts[j];
                pts[j] = pts[j - 1];
                pts[j - 1] = temp;
            }

            minSphere = minimalSphere(pts, i, start+1, nbFound+1);
        }
    }

}

bool Sphere::intersects(const AABB &aabb) const noexcept {
    return false;
}

bool Sphere::intersects(const Intersectable &intersectable) const noexcept {
    return intersectable.intersects(*this);
}

float Sphere::getVolume()
{
    assert(isDefined(m_center));

    return 4*M_PI*m_radius*m_radius*m_radius/3;
}


void Sphere::constructMesh()
{
    // This function should only be called once
    assert(m_meshFaces.empty());
    assert(m_meshFaces.empty());

    // Geometry of the dodecahedron based on Paul Bourke's code:
    // http://paulbourke.net/geometry/platonic/sphere.cpp

    const float sqrt5 = sqrtf (5.0f);
    const float phi = (1.0f + sqrt5) * 0.5f;

    // ratio of edge length to radius
    const float ratio = sqrtf (10.0f + (2.0f * sqrt5)) / (4.0f * phi);
    const float a = (m_radius / ratio) * 0.5f;
    const float b = (m_radius / ratio) / (2.0f * phi);

    // define the icosahedron's 12 vertices:
    const Point v1  = m_center + Point { 0,  b, -a};
    const Point v2  = m_center + Point { b,  a,  0};
    const Point v3  = m_center + Point {-b,  a,  0};
    const Point v4  = m_center + Point { 0,  b,  a};
    const Point v5  = m_center + Point { 0, -b,  a};
    const Point v6  = m_center + Point {-a,  0,  b};
    const Point v7  = m_center + Point { 0, -b, -a};
    const Point v8  = m_center + Point { a,  0, -b};
    const Point v9  = m_center + Point { a,  0,  b};
    const Point v10 = m_center + Point {-a,  0, -b};
    const Point v11 = m_center + Point { b, -a,  0};
    const Point v12 = m_center + Point {-b, -a,  0};


    m_meshVertices.push_back(v1.data());
    m_meshVertices.push_back(v2.data());
    m_meshVertices.push_back(v3.data());
    m_meshVertices.push_back(v4.data());
    m_meshVertices.push_back(v5.data());
    m_meshVertices.push_back(v6.data());
    m_meshVertices.push_back(v7.data());
    m_meshVertices.push_back(v8.data());
    m_meshVertices.push_back(v9.data());
    m_meshVertices.push_back(v10.data());
    m_meshVertices.push_back(v11.data());
    m_meshVertices.push_back(v12.data());

    std::vector<unsigned int> f1 = {0,1,2};
    std::vector<unsigned int> f2 = {3,2,1};
    std::vector<unsigned int> f3 = {3,4,5};
    std::vector<unsigned int> f4 = {3,8,4};
    std::vector<unsigned int> f5 = {0,6,7};
    std::vector<unsigned int> f6 = {0,9,6};
    std::vector<unsigned int> f7 = {4,10,11};
    std::vector<unsigned int> f8 = {6,11,10};
    std::vector<unsigned int> f9 = {2,5,9};
    std::vector<unsigned int> f10 = {11,9,5};
    std::vector<unsigned int> f11 = {1,7,8};
    std::vector<unsigned int> f12 = {10,8,7};
    std::vector<unsigned int> f13 = {3,5,2};
    std::vector<unsigned int> f14 = {3,1,8};
    std::vector<unsigned int> f15 = {0,2,9};
    std::vector<unsigned int> f16 = {0,7,1};
    std::vector<unsigned int> f17 = {6,9,11};
    std::vector<unsigned int> f18 = {6,10,7};
    std::vector<unsigned int> f19 = {4,11,5};
    std::vector<unsigned int> f20 = {4,8,10};

    m_meshFaces.push_back(f1);
    m_meshFaces.push_back(f2);
    m_meshFaces.push_back(f3);
    m_meshFaces.push_back(f4);
    m_meshFaces.push_back(f5);
    m_meshFaces.push_back(f6);
    m_meshFaces.push_back(f7);
    m_meshFaces.push_back(f8);
    m_meshFaces.push_back(f9);
    m_meshFaces.push_back(f10);
    m_meshFaces.push_back(f11);
    m_meshFaces.push_back(f12);
    m_meshFaces.push_back(f13);
    m_meshFaces.push_back(f14);
    m_meshFaces.push_back(f15);
    m_meshFaces.push_back(f16);
    m_meshFaces.push_back(f17);
    m_meshFaces.push_back(f18);
    m_meshFaces.push_back(f19);
    m_meshFaces.push_back(f20);
}


bool Sphere::intersects(const Plane &plane) const noexcept {
    // Feature not implemented yet
    assert(false);
    return false;
}

bool Sphere::intersects(const Triangle &tri) const noexcept {
    return tri.intersects(*this);
}
