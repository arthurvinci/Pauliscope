//
// Created by arthur on 16/03/2022.
//

#ifndef AABB_HPP
#define AABB_HPP

#include "../primitives/Point.hpp"
#include "polyscope/surface_mesh.h"
#include "BoundingVolume.hpp"
#include "../traits/Displayable.hpp"
#include "polyscope/surface_mesh.h"


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

    bool intersects(Intersectable const& intersectable) const noexcept override;

    bool intersectsBV(AABB const& aabb) const noexcept override;

    bool intersectsBV(Sphere const& sp) const noexcept override;

    bool intersects(const BoundingVolume &boundingVolume) const noexcept override;

    bool intersects(const Plane &plane) const noexcept override;

    /**
     *
     * @param aabb
     * @return
     *
     * @more using Akenine-Möller approach https://dl.acm.org/doi/10.1080/10867651.2001.10487535
     */
    bool intersects(const Triangle &tri) const noexcept override;

    void update(Eigen::Matrix3f const& r, Vector const& t) override;

    void update(const Vector &t) override;

    Point closestPointToPoint(Point const& p) const noexcept;

    float distanceToPoint(Point const& p) const noexcept;

    float getVolume() override;

    void constructMesh() override;

    void updateMesh() noexcept override;

    void setColor(int r, int g, int b) const noexcept override;

    void setVisible(bool isVisible) noexcept override;

    polyscope::SurfaceMesh* getMesh();

    Vector getHalfWidth() override;



private:
    Point m_center; // Center point of the box
    std::array<float,3> m_radius; // radius in each direction (x,y,z)
    polyscope::SurfaceMesh* m_mesh; // mesh of the AABB

    static void extremePointsAlongDirection(Vector direction, std::vector<Point> const& pts, unsigned int *indexMin, unsigned int *indexMax);

    void constructMesh(bool update);
};


#endif //AABB_HPP
