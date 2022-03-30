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

};

enum bvID
{
    SPHERE_ID = 0,
    AABB_ID   = 1
};


#endif //BOUNDINGVOLUME_HPP
