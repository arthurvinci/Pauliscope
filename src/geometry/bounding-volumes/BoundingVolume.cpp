//
// Created by arthur on 31/03/2022.
//

#include "BoundingVolume.hpp"
#include "AABB.hpp"
#include "Sphere.hpp"

BoundingVolume *BoundingVolume::bestFittingBV(const std::vector<glm::vec3> &points) noexcept
{
    assert(!points.empty());

    std::vector<Point> pts{};
    for(auto pt_arr : points)
    {
        Point new_pt{pt_arr[0], pt_arr[1], pt_arr[2]};
        pts.push_back(new_pt);
    }
    return bestFittingBV(pts);
}

bool BoundingVolume::intersects(const Intersectable &intersectable) const noexcept {
    return intersectable.intersects(*this);
}

BoundingVolume *BoundingVolume::bestFittingBV(const std::vector<Point> &points) noexcept {
    assert(!points.empty());

    // Compute every BoundingVolume possible and then compare volume/computations needed for an intersection check
    auto aabb = new AABB(points);
    auto aabb_vol = aabb->getVolume();
    auto sp = new Sphere(points);
    auto sp_vol = sp->getVolume();

    if(sp_vol>aabb_vol)
        return aabb;
    else
        return sp;
}
