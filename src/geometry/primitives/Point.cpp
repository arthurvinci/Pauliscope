//
// Created by arthur on 23/03/2022.
//

#include "Point.hpp"

float distance(Point p1, Point p2)
{
    float dist2 = (p1.x - p2.x)*(p1.x - p2.x) +  (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z);
    return sqrtf(dist2);
}

bool areCollinear(Point p1, Point p2, Point p3)
{
    Vector v1 = p3 -p2;
    Vector v2 = p3-p1;

    return dot(v1,v2)*dot(v1,v2) == norm2(v1)*norm2(v2);
}