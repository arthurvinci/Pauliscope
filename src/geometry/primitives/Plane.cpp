//
// Created by arthur on 16/03/2022.
//

#include "Plane.hpp"

Plane::Plane(const Point &point, const Vector &normal) noexcept : m_normal(normal)
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



bool operator==(const Plane &  plane1, const Plane & plane2) {

    return plane1.getNormal() == plane2.getNormal() && plane1.getD() == plane2.getD();
}

bool operator!=(const Plane & plane1, const Plane & plane2)
{
    return !(plane1 == plane2);
}