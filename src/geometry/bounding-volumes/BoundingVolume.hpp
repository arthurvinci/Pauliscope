//
// Created by arthur on 16/03/2022.
//

#ifndef BOUNDINGVOLUME_HPP
#define BOUNDINGVOLUME_HPP

#include <vector>
#include "Eigen/Core"
#include "../primitives/Point.hpp"
#include "../traits/Displayable.hpp"
#include "../traits/Intersectable.hpp"

class BoundingVolume : public Displayable, public Intersectable {

public:

    explicit BoundingVolume(bool realInstance = true) : Displayable(false, realInstance, false)
    {}


    /**
     * @brief Computes whether the `Intersectable` intersects with a `Sphere`
     * @param sp other `Sphere` to check the intersection with
     * @return whether they intersect or not
    */
    virtual bool intersectsBV(Sphere const &sp) const noexcept = 0;


    /**
     * @brief Computes whether the `Intersectable` intersects with an `AABB`
     * @param aabb other `AABB` to check the intersection with
     * @return whether they intersect or not
    */
    virtual bool intersectsBV(AABB const &aabb) const noexcept = 0;


    bool intersects(const Intersectable &intersectable) const noexcept override;

    /**
     * @brief Returns the volume of the `BoundingVolume`
     * @return volume of the `BoundingVolume`
     */
    virtual float getVolume() = 0;

    /**
    * @brief Updates a `BoundingVolume` from a rotation r and a translation t and updates its mesh
    * @param r rotation matrix of the transformation
    * @param t translation vector
 */
    virtual void update(Eigen::Matrix3f const& r, Vector const& t) = 0;

    /**
    * @brief Updates a `BoundingVolume` from a translation t and updates its mesh
    * @param t translation vector
    */
    virtual void update(Vector const& t) = 0;

    /**
    * @brief Computes the best fitting `BoundingVolume` for a given set of `Point`s
     * @return  best fitting `BoundingVolume` for the `Object`
     *
     * @note This function is very costly and should only be used at initialization
     *
     * @todo Take possibility of rotation into account
     */
    static BoundingVolume *bestFittingBV(std::vector<glm::vec3>const& points) noexcept;

    static BoundingVolume *bestFittingBV(std::vector<Point>const& points) noexcept;

    virtual Vector getHalfWidth() = 0;

    virtual const Point &getCenter() const noexcept = 0;
};



enum bvID
{
    SPHERE_ID = 0,
    AABB_ID   = 1
};


#endif //BOUNDINGVOLUME_HPP
