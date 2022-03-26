//
// Created by arthur on 24/03/2022.
//

#ifndef DISPLAYABLE_HPP
#define DISPLAYABLE_HPP

#include "polyscope/surface_mesh.h"
#include <utility>
#include <vector>
#include <array>

class Displayable {

public:
    std::vector<std::array<float,3>> m_meshVertices;
    std::vector<std::vector<unsigned int>> m_meshFaces;

    Displayable(std::string &name, std::vector<std::array<float,3>> &vertices, std::vector<std::vector<unsigned int>> &faces, bool needsUpdate)
    {
        m_name = name;
        m_meshVertices = vertices;
        m_meshFaces = faces;
        m_needsMeshUpdate = needsUpdate;
    }

    Displayable(std::string name, bool needsUpdate)
    {
        m_name = std::move(name);
        m_needsMeshUpdate = needsUpdate;
    }

    std::vector<std::array<float,3>> &getMeshVertices()
    {
        if(m_needsMeshUpdate)
            constructMesh();
        m_needsMeshUpdate  = false;

        return m_meshVertices;
    }

    virtual std::vector<std::vector<unsigned int>> &getMeshFaces()
    {
        if(m_needsMeshUpdate)
            constructMesh();
        m_needsMeshUpdate  = false;

        return m_meshFaces;
    }

    /**
     * Computes a Mesh out of the `Displayable` and returns a tuple of vertices and faces
     * @return a tuple containing the vertices and faces of the mesh
     */
    void registerMesh()
    {
        if(m_needsMeshUpdate)
            constructMesh();
        m_needsMeshUpdate  = false;

        polyscope::registerSurfaceMesh(m_name, m_meshVertices, m_meshFaces);
    }

protected:
    std::string m_name;
    bool m_needsMeshUpdate;

    virtual void constructMesh() = 0;

};


#endif //DISPLAYABLE_HPP
