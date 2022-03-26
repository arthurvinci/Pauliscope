//
// Created by arthur on 16/03/2022.
//

#ifndef PLANE_HPP
#define PLANE_HPP

#include "Point.hpp"
#include "Vector.hpp"

class Plane{

public:

    /**
     * @brief An empty plane cannot be instantiated
     */
    Plane() = delete;

    /**
     * @brief Default copy constructor
     * @param plane plane to be copied
     */
    Plane(Plane const& plane) = default;

    /**
     * @brief Creates a new instance of Plane from a point and a vector
     * @param point point in the plane
     * @param normal normal vector to the plane
     */
    Plane(const Point &point, const Vector &normal) noexcept;

    /**
     * @brief Creates a new instance of Plane from three points
     * @param a first point
     * @param b second point
     * @param c third point
     */
    Plane(Point const &a, Point const &b, Point const &c) noexcept;


    /**
     * @brief Default assignation by copy operator
     * @param plane plane to be copied
     * @return A copy of the given plane
     */
    Plane& operator=(Plane const & plane) = default;

    /**
     * @brief Displays a plane on a given output stream
     * @param stream where the plane should be displayed
     * @param plane the plane to be displayed
     * @return The used output stream
     */
    friend std::ostream& operator<< (std::ostream & stream, Plane const & plane) noexcept;

    float getD() const noexcept;

    void setD(float d);

    const Vector &getNormal() const;

    void setNormal(const Vector &normal);

private:
    // Describes a plane in constant-normal form
    float m_d; // d = m_normal * P for a certain point P
    Vector m_normal{}; // Plane normal. Points contained in the plane satisfy : m_normal * p = m_d

};


/**
 * @brief Determines if two planes are equale
 * @details Two planes are equal if all their coordinates values are equal
 * @param plane other plane to compare to
 * @return A boolean stating if the planes are equal
 */
bool operator==(const Plane & plane1, const Plane & plane2);

/**
 * @brief Determines if two planes are different
 * @details Two planes are different if one of their coordinates are different
 * @param plane other plane to compare to
 * @return A boolean stating if the planes are different
 */
bool operator!=(const Plane & plane1, const Plane & plane2);

#endif //PLANE_HPP
