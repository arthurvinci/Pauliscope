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
    polyscope::SurfaceMesh *m_mesh;
    /**
     * @brief An empty `Triangle` cannot be instantiated
     */
    Triangle() = delete;

    /**
     * @brief Default copy constructor is deleted because we should not be able to copy a triangle
     */
    Triangle(Triangle const& Triangle) = default;

    Triangle(Point &p1, Point &p2, Point &p3);

    Point closestPointToPoint(Point const& p) const noexcept;

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

    bool intersects(const BoundingVolume &boundingVolume) const noexcept override;

    Vector getPoint(unsigned int i) const;

    void setColor(int r, int g, int b) const noexcept override;

    void setVisible(bool isVisible) noexcept override;

    void updateMesh() override;


protected:
    void constructMesh();


private:
    // Using array of pointers to improve space complexity when construct triangles from an object
    std::array<Point,3> m_pts;
    bool m_needsMeshUpdate;


};


#endif //TRIANGLE_HPP
