//
// Created by arthur on 16/03/2022.
//

#ifndef PLANE_HPP
#define PLANE_HPP

#include "Point.hpp"
#include "../traits/Intersectable.hpp"

class Plane : public Intersectable{

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
     * @brief Creates a new instance of `Plane` from a normal and a number
     * @param point
     * @param d
     */
    Plane(const Vector &normal, float d) noexcept;

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

    /**
     * @brief Returns the closest `Point` in the plane to the given `Point`
     * @param p `Point` to make the omputation for
     * @return closest `Point` in plane to p
     */
    Point closestPointToPoint(Point const& p) const noexcept;

    /**
     * @brief Returns the distance between a `Point` and the `Plane`
     * @param p `Point` for which to compute the distance
     * @return the distance from the `Plane` to the `Point`
     */
    float distanceTo(Point const& p) const noexcept;

    float getD() const noexcept;

    void setD(float d);

    const Vector &getNormal() const;

    void setNormal(const Vector &normal);

    bool intersects(const Intersectable &intersectable) const noexcept override;

    bool intersects(const Sphere &sp) const noexcept override;

    bool intersects(const AABB &aabb) const noexcept override;

    bool intersects(const Plane &plane) const noexcept override;

    bool intersects(const Triangle &tri) const noexcept override;

    std::tuple<Point, Vector> getIntersectionLine(Plane const& otherPlane);

private:
    // Describes a plane in constant-normal form
    float m_d; // d = m_normal * P for a certain point P in the plane
    Vector m_normal{}; // Plane unitary normal. Points contained in the plane satisfy : m_normal * p = m_d

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
