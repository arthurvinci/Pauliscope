//
// Created by arthur on 16/03/2022.
//

#include "Plane.hpp"
#include "../bounding-volumes/Sphere.hpp"
#include "../bounding-volumes/AABB.hpp"
#include "../defs.hpp"

Plane::Plane(const Point &point, const Vector &normal) noexcept : m_normal(normalize(normal))
{
    m_d = dot(m_normal, point);
}

Plane::Plane(Point const &a, Point const &b, Point const &c) noexcept
{
    m_normal = normalize(cross(b-a, c-a));
    m_d = dot(m_normal, a);
}

std::ostream &operator<<(std::ostream &stream, const Plane &plane) noexcept
{
    stream << "( d : " << plane.m_d << " ; normal : " << plane.m_normal << " )";
    return stream;
}

const Vector &Plane::getNormal() const {
    return m_normal;
}

void Plane::setNormal(const Vector &normal) {
    m_normal = normal;
}

float Plane::getD() const noexcept {
    return m_d;
}

void Plane::setD(float d) {
    m_d = d;
}

Point Plane::closestPointToPoint(const Point &p) const noexcept
{
    float temp  = dot(m_normal, p) - m_d;
    return p -temp*m_normal;
}

float Plane::distanceTo(const Point &p) const noexcept {
    return dot(m_normal, p);
}


bool operator==(const Plane &  plane1, const Plane & plane2) {

    return plane1.getNormal() == plane2.getNormal() && plane1.getD() == plane2.getD();
}

bool operator!=(const Plane & plane1, const Plane & plane2)
{
    return !(plane1 == plane2);
}

Plane::Plane(const Vector &normal, float d) noexcept : m_d(d), m_normal(normalize(normal)){}

bool Plane::intersects(const Intersectable &intersectable) const noexcept {
    return intersectable.intersects(*this);
}

bool Plane::intersects(const Sphere &sp) const noexcept {
    return sp.intersects(*this);
}

bool Plane::intersects(const AABB &aabb) const noexcept {
    return aabb.intersects(*this);
}

bool Plane::intersects(const Plane &plane) const noexcept
{
    return (plane.m_normal == m_normal || plane.m_normal == -m_normal);
}

bool Plane::intersects(const Triangle &tri) const noexcept
{
    // Not implemented yet
    assert(false);
    return false;
}

std::tuple<Point, Vector> Plane::getIntersectionLine(const Plane &otherPlane)
{
    Vector line_dir = cross(m_normal, otherPlane.m_normal);

    if(norm2(line_dir) < ACCEPTABLE_ERROR*ACCEPTABLE_ERROR)
    {
        // Planes are parallel..
        auto tuple = std::make_tuple(Point::undefined(), Vector::undefined());
        return tuple;
    }
    Point p = cross( m_d*otherPlane.m_normal - otherPlane.m_d*m_normal, line_dir);
    auto tuple = std::make_tuple(p,line_dir);
    return tuple;
}
