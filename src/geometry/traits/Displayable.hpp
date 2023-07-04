//
// Created by arthur on 24/03/2022.
//

#ifndef DISPLAYABLE_HPP
#define DISPLAYABLE_HPP

#include "polyscope/surface_mesh.h"
#include "polyscope/structure.h"
#include "Instantiable.hpp"
#include <utility>
#include <vector>
#include <array>
class Displayable : public Instantiable {

public:

    explicit Displayable(bool isVisible, bool realInstance, bool isMeshConstructed) : Instantiable(realInstance)
    {
        m_isVisible = isVisible;
        m_isMeshConstructed = isMeshConstructed;
    }

    virtual void constructMesh() = 0;

    virtual void setColor(int r, int g, int b) const noexcept = 0;

    virtual void setVisible(bool isVisible) noexcept = 0;

    virtual void updateMesh() = 0;

protected:
    bool m_isVisible;
    bool m_isMeshConstructed;
};



#endif //DISPLAYABLE_HPP
