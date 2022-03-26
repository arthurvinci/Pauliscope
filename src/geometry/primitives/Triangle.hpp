//
// Created by arthur on 26/03/2022.
//

#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP


#include <memory>
#include "Point.hpp"
#include "../traits/Intersectable.hpp"
#include "../traits/Displayable.hpp"

class Triangle : public Intersectable, public Displayable
{
public:
    /**
     * @brief An empty `Triangle` cannot be instantiated
     */
    Triangle() = delete;

    /**
     * @brief Default copy constructor is deleted because we should not be able to copy a triangle
     */
    Triangle(Triangle const& Triangle) = delete;

    Triangle(Point * p1, Point * p2, Point * p3, std::string name);

    Point closestPointToPoint(Point const& p) const noexcept;

    bool intersects(const Sphere &sp) const noexcept override;

    /**
     *
     * @param aabb
     * @return
     *
     * @more using Akenine-Möller approach https://dl.acm.org/doi/10.1080/10867651.2001.10487535
     */
    bool intersects(const AABB &aabb) const noexcept override;

    bool intersects(const Plane &plane) const noexcept override;

    /**
     *
     * @param tri
     * @return
     *
     * @more using Möller method https://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
     */
    bool intersects(const Triangle &tri) const noexcept override;

    bool intersects(const Intersectable &intersectable) const noexcept override;

protected:
    void constructMesh() override;

protected:





private:
    // Using array of pointers to improve space complexity when construct triangles from an object
    std::array<Point*,3> m_pts;

};


#endif //TRIANGLE_HPP
