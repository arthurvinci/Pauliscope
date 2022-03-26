//
// Created by arthur on 16/03/2022.
//

#ifndef BOUNDINGVOLUME_HPP
#define BOUNDINGVOLUME_HPP

#include <vector>
#include "../primitives/Point.hpp"
#include "../Displayable.hpp"

class Sphere;
class AABB;

#define NOT_REAL_INSTANCE (4294967295U)

class BoundingVolume : public Displayable {

public:

    explicit BoundingVolume(bool realInstance = true) : Displayable(genName(bv_instances), true) {
        if (realInstance) {
            m_id = bv_instances;
            bv_instances++;
            //std::cout << "New Sphere instance "<<m_id<<std::endl;

        } else
        {
            m_id = NOT_REAL_INSTANCE;
        }

    }



/**
     * @brief Computes whether the `BoundingVolume` intersects with another `BoundingVolume`
     * @param bv other `BoundingVolume` to check intersection with
     * @return whether there is an intersection with the given `BoundingVolume`
     *
     * @more Basic function to implement the visitor pattern to compute intersections
     */
    virtual bool intersects(BoundingVolume const &bv) const noexcept = 0;

    /**
     * @brief Computes whether `BoundingVolume` intersects with a `Sphere`
     * @param sp other `Sphere` to check the intersection with
     * @return whether they intersect or not
    */
    virtual bool intersects(Sphere const &sp) const noexcept = 0;


    /**
     * @brief Computes whether `BoundingVolume` intersects with an `AABB`
     * @param aabb other `AABB` to check the intersection with
     * @return whether they intersect or not
    */
    virtual bool intersects(AABB const &aabb) const noexcept = 0;

    /**
     * @brief Returns the volume of the `BoundingVolume`
     * @return volume of the `BoundingVolume`
     */
    virtual float getVolume() = 0;

    static unsigned int bv_instances;

protected:
    unsigned int m_id;

private:
    static std::string genName(unsigned int id) {
        std::ostringstream os;
        os << "BoundingVolume" << id;
        std::string n = os.str();
        return n;
    };

};

enum bvID
{
    SPHERE_ID = 0,
    AABB_ID   = 1
};


#endif //BOUNDINGVOLUME_HPP
