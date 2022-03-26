//
// Created by arthur on 16/03/2022.
//

#ifndef AABB_HPP
#define AABB_HPP

#include "../primitives/Point.hpp"
#include "Eigen/Core"
#include "BoundingVolume.hpp"
#include "../Displayable.hpp"


/**
 * Implementation of Axis-Aligned Bounding Boxes with center-radius representation
 */
class AABB : public BoundingVolume{

public:

    /**
     * @brief An empty `AABB` cannot be instantiated
     */
    AABB() = delete;

    /**
     * @brief Default copy constructor
     * @param aabb `AABB` to be copied
     */
    AABB(AABB const& aabb) = default;

    /**
     * @brief Creates an `AABB` from a point and 3 values for the radius
     * @param point center point of the `AABB`
     * @param r_x radius in the x direction
     * @param r_y radius in the y direction
     * @param r_z radius in the z direction
     */
    AABB(const Point &point, float r_x, float r_y,float r_z) noexcept;

    /**
     * @brief Creates an `AABB` from three points coordinates and 3 values for the radius
     * @param x x coordinate for the center point
     * @param y y coordinate for the center point
     * @param z z coordinate for the center point
     * @param r_x radius in the x direction
     * @param r_y radius in the y direction
     * @param r_z radius in the z direction
     */
    AABB(float x, float y, float z, float r_x, float r_y,float r_z) noexcept;

    /**
     * @brief Creates an `AABB` from a set of points
     * @param pts set of points to construct the `AABB` from
     */
    explicit AABB(std::vector<Point> const& pts) noexcept;

    /**
     * @brief Creates an `AABB` from a given `Sphere`
     * @param sp `Sphere` from which to construct an `AABB`
     */
    explicit AABB(Sphere const& sp) noexcept;

    const Point &getCenter() const noexcept;

    std::array<float,3> getRadius() const noexcept;

    bool intersects(BoundingVolume const& bv) const noexcept override;

    bool intersects(AABB const& aabb) const noexcept override;

    bool intersects(Sphere const& sp) const noexcept override;

    /**
     * @brief Updates an `AABB` from a rotation m and a translation t
     * @param r rotation matrix of the transformation
     * @param t translation vector
     */
    void update(Eigen::Matrix3f const& r, Vector const& t);


    float getVolume() override;


private:
    Point m_center; // Center point of the box
    std::array<float,3> m_radius; // radius in each direction (x,y,z)

    static void extremePointsAlongDirection(Vector direction, std::vector<Point> const& pts, unsigned int *indexMin, unsigned int *indexMax);

    void constructMesh() override;
};


#endif //AABB_HPP
