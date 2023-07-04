//
// Created by arthur on 26/03/2022.
//

#ifndef INTERSECTABLE_HPP
#define INTERSECTABLE_HPP

class Sphere;
class AABB;
class Plane;
class Triangle;
class BoundingVolume;

class Intersectable
{

public:

    /**
     * @brief Computes whether the `Intersectable` intersects with another `Intersectable`
     * @param intersectable other `Intersectable` to check intersection with
     * @return whether there is an intersection with the given `Intersectable`
     *
     * @details Basic function to implement the visitor pattern to compute intersections
     */
    virtual bool intersects(Intersectable const &intersectable) const noexcept = 0;

    virtual bool intersects(BoundingVolume const& boundingVolume) const noexcept = 0;

    /**
     * @brief Computes whether the `Intersectable` intersects with a `Plane`
     * @param plane other `Plane` to check the intersection with
     * @return whether they intersect or not
     */
    virtual bool intersects(Plane const &plane) const noexcept = 0;

    /**
     * @brief Computes whether the `Intersectable` intersects with a `Triangle`
     * @param tri other `Triangle` to check the intersection with
     * @return whether they intersect or not
     */
    virtual bool intersects(Triangle const &tri) const noexcept = 0;



};

#endif //INTERSECTABLE_HPP