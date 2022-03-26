//
// Created by arthur on 24/03/2022.
//

#ifndef OBJECT_HPP
#define OBJECT_HPP


#include <vector>
#include "primitives/Point.hpp"
#include "bounding-volumes/BoundingVolume.hpp"
#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/surface/embedded_geometry_interface.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/geometry.h"


class Object{

public:
    /**
    * @brief An empty `Object` cannot be instantiated
    */
    Object() = delete;

    /**
     * @brief Default copy constructor
     * @param obj `Object` to be copied
     */
    Object(Object const& obj) = default;

    /**
     * @brief Creates an `Object` from a given set of vertex data
     * @param vertexData set of vertex data
     */
    explicit Object(geometrycentral::surface::VertexData<geometrycentral::Vector3> const& vertexData) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertex data and a `BoundingVolume` hint
     * @param vertexData set of vertex data
     * @param bvId `bvID` of the `BoundingVolume` to construct for the object
     */
    Object(geometrycentral::surface::VertexData<geometrycentral::Vector3> const& vertexData, bvID bvId) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertices
     * @param vertices set of vertices
     */
    explicit Object(std::vector<std::array<float,3>> const& vertices) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertices and a `BoundingVolume`
     * @param vertices set of vertices
     * @param bv `BoundingVolume`
     */
    Object(std::vector<std::array<float,3>> const& vertices, bvID bvId)noexcept;

    /**
     * @brief Creates an `Object` from a BoundingVolume
     * @param bv
     */
    explicit Object(BoundingVolume *bv) noexcept;

    BoundingVolume *getBoundingVolume() const noexcept;

    /**
     * @brief Computes the best fitting `BoundingVolume` for a given set of `Point`s
     * @return  best fitting `BoundingVolume` for the `Object`
     *
     * @note This function is very costly and should only be used at initialization
     *
     * @todo Take possibility of rotation into account
     */
    static BoundingVolume *bestFittingBV(std::vector<Point> const& points) noexcept;


private:
    std::vector<Point> m_vertices;
    BoundingVolume*    m_boundingVolume;
};



#endif //OBJECT_HPP
