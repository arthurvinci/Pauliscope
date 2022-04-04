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
#include "trees/BVOctree.hpp"


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

    explicit Object(const std::string& path, Vector speed, std::string name=genName("object"));

    /*
    Object(std::string path, bvID boundingVolume,std::string name= genName("object"));

    explicit Object(polyscope::SurfaceMesh* surfaceMesh) noexcept;
     */

    void update(Eigen::Matrix3f const& r, Vector const& t);

    void update(float dt);

    void constructMesh() override;

    void setColor(int r, int g, int b) const noexcept override;

    void setVisible(bool isVisible) noexcept override;

    void updateMesh() override;

    void showOctree(bool show, int depthToShow);

    bool intersects(Object &other) const;

    bool rawIntersects(Object &other) const;

private:
    BVOctree m_insideTree;
    polyscope::SurfaceMesh* m_mesh;
    Vector m_speed;

    static Point getObjectBarycenter(std::vector<glm::vec3>const& points);

};



#endif //OBJECT_HPP
