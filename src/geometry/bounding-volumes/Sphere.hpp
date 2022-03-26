//
// Created by arthur on 16/03/2022.
//

#ifndef SPHERE_HPP
#define SPHERE_HPP


#include "../primitives/Point.hpp"
#include "BoundingVolume.hpp"
#include <vector>

/**
 * Implementation of Spheres
 */
class Sphere: public BoundingVolume{

public:

    /**
     * @brief An empty `Sphere` is a sphere with undefined center and radius 0
     */
    explicit Sphere(bool realInstance=true);

    /**
     * @brief Default copy constructor
     * @param sp `Sphere` to be copied
     */
    Sphere(Sphere const& sp) = default;

    /**
     * @brief Creates a `Sphere` form a given point and a radius
     * @param p center of the `Sphere`
     * @param radius radius of the `Sphere`
     */
    Sphere(Point const& p, float radius, bool realInstance=true);

    /**
     * @brief Creates a sphere with the given point as center and 0 value as radius
     * @param p center of the sphere
     */
    explicit Sphere(Point const& p, bool realInstance=true);

    /**
     * @brief Creates a minimum `Sphere` encompassing two given points
     * @param p1 first point
     * @param p2 second point
     */
    Sphere(Point const& p1, Point const& p2, bool realInstance=true);

    /**
     * @brief Creates a minimum `Sphere` encompassing three given points
     * @param p1 first point
     * @param p2 second point
     * @param p3 third point
     *
     * @bug Does not work if the three given points are collinear
     */
    Sphere(Point const& p1, Point const& p2, Point const& p3, bool realInstance=true);


    /**
     * Creates a `Sphere` from four non co-planar points
     * @param p1 first point
     * @param p2 second point
     * @param p3 third point
     * @param p4 fourth point
     *
     * @more uses the computations given in https://mathworld.wolfram.com/Circumsphere.html
     */
    Sphere(Point const& p1, Point const& p2, Point const& p3, Point const& p4, bool realInstance=true);

    /**
     * Creates a `Sphere` that encompasses a given set of points.
     * @param points set of points to create the sphere for
     *
     * @more the returned `Sphere` will be minimal
     */
    explicit Sphere(std::vector<Point> const& points, bool realInstance=true);

    /**
    * @brief Returns whether a point is inside the `Sphere`
    * @param p point to check
    * @return whether the point is inside the `Sphere`
    */
    bool isInside(Point const& p);

    const Point &getCenter() const;

    float getRadius() const;

    bool intersects(Sphere const& sp) const noexcept override;

    bool intersects(class AABB const& sp) const noexcept override;

    bool intersects(Intersectable const& intersectable) const noexcept override;

    bool intersects(const Plane &plane) const noexcept override;

    bool intersects(const Triangle &tri) const noexcept override;

    float getVolume() override;


private:
    Point m_center;
    float m_radius;

    void constructMesh() override;

    /**
     * @brief Computes the minimal `Sphere` encompassing a set of `Point`s
     * @param pts `std::vector` of `Point` from which to construct the `Sphere`
     * @param nbPts number of points
     * @param start start index of the sub vector in the pts vector
     * @param nbFound number of points found to construct the Sphere (A Sphere needs at most 4 points to be defined)
     * @return The minimal `Sphere` encompassing all the points
     *
     * @more This is an implementation of Welzl algorithm and more precisely the move-to-front heuristic.
     *       The algorithm is inspired by Bernd Gärtner paper https://people.inf.ethz.ch/gaertner/subdir/texts/own_work/esa99_final.pdf
     *       and Nicolas Capens' blog post https://www.flipcode.com/archives/Smallest_Enclosing_Spheres.shtml
     */
    static Sphere minimalSphere(std::vector<Point> &pts, unsigned int nbPts, unsigned int start, unsigned int nbFound);

};


#endif //SPHERE_HPP
