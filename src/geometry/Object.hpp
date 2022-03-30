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


class Object: public Displayable{

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

    explicit Object(const std::string& path, std::string name=genName("object"));

    Object(std::string path, bvID boundingVolume,std::string name= genName("object"));


    /**
     * @brief Creates an `Object` from a given set of vertex data
     * @param vertexData set of vertex data
     */
    explicit Object(geometrycentral::surface::VertexData<geometrycentral::Vector3> const& vertexData, std::vector<std::vector<size_t>> const& faceData, std::string name) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertex data and a `BoundingVolume` hint
     * @param vertexData set of vertex data
     * @param bvId `bvID` of the `BoundingVolume` to construct for the object
     */
    Object(geometrycentral::surface::VertexData<geometrycentral::Vector3> const& vertexData, std::vector<std::vector<size_t>> const& faceData, std::string name, bvID bvId) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertices
     * @param vertices set of vertices
     */
    explicit Object(std::vector<std::array<float,3>> const& vertices, std::vector<std::vector<size_t>> const& faceData, std::string name) noexcept;

    /**
     * @brief Creates an `Object` from a given set of vertices and a `BoundingVolume`
     * @param vertices set of vertices
     * @param bv `BoundingVolume`
     */
    Object(std::vector<std::array<float,3>> const& vertices, std::vector<std::vector<size_t>> const& faceData, std::string name, bvID bvId)noexcept;

    /**
     * @brief Creates an `Object` from a BoundingVolume
     * @param bv
     */
    explicit Object(BoundingVolume *bv) noexcept;

    explicit Object(polyscope::SurfaceMesh* surfaceMesh) noexcept;

    BoundingVolume *getBoundingVolume() const noexcept;

    /**
     * @brief Computes the best fitting `BoundingVolume` for a given set of `Point`s
     * @return  best fitting `BoundingVolume` for the `Object`
     *
     * @note This function is very costly and should only be used at initialization
     *
     * @todo Take possibility of rotation into account
     */
    static BoundingVolume *bestFittingBV(std::vector<glm::vec3>const& points) noexcept;

    void update(Eigen::Matrix3f const& r, Vector const& t);

    void update(Vector const& t);

    void constructMesh() override;

    void setColor(int r, int g, int b) const noexcept override;

    void setVisible(bool isVisible) noexcept override;

    void updateMesh() override;

private:
    BoundingVolume*    m_boundingVolume;
    polyscope::SurfaceMesh* m_mesh;

};



#endif //OBJECT_HPP
