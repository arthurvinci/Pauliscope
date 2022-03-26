//
// Created by arthur on 16/03/2022.
//

#ifndef POINT_HPP
#define POINT_HPP
#include "Vector.hpp"

using Point = Vector;

float distance(Point p1, Point p2);

bool areCollinear(Point p1, Point p2, Point p3);

#endif //POINT_HPP
