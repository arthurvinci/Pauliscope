//
// Created by arthur on 31/03/2022.
//

#ifndef BVNODE_HPP
#define BVNODE_HPP

#include "../bounding-volumes/BoundingVolume.hpp"

class BVNode{

public:

    BVNode(Point const& center, Vector halfWidth, unsigned int depth, std::vector<glm::vec3> &vertices, const std::vector<std::vector<size_t>> &faces);

    static int nodeDepth(unsigned int key);

    static unsigned int mortonKey(unsigned int x, unsigned int y, unsigned int z);

    BoundingVolume* getBoundingVolume();

    unsigned int getDepth() const;

    bool intersects(BVNode &other);

private:
    Point m_center; // Theoretical center of the node, can differ from the bounding volume center
    BoundingVolume * m_boundingVolume; // Bounding volume for this node
    //std::vector<std::vector<unsigned int>*> m_faces; // faces of the original mesh in the octree
    std::vector<Triangle> m_faces;
    unsigned int m_depth;
    static unsigned int part1By2(unsigned int n);
};



#endif //BVNODE_HPP
