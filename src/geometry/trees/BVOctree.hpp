//
// Created by arthur on 01/04/2022.
//

#ifndef BVOCTREE_HPP
#define BVOCTREE_HPP


#include <unordered_map>
#include "BVNode.hpp"

#define NB_NODES 73 // height of 2
#define MAX_DEPTH 2

class BVOctree{

public:

    BVOctree() = default;

    BVOctree(Point const& center, std::vector<glm::vec3> &vertices, const std::vector<std::vector<size_t>> &faces);

    bool intersects(BVOctree const& other) const;

    void update(Vector const& t);

    void setVisible(bool visible, int startDepth=0);

    BVNode* getNodeAt(unsigned int i);

private:
    std::array<BVNode*,NB_NODES> m_nodes{};

    void buildTree(Point const& center, Vector halfWidth, std::vector<glm::vec3> &vertices, const std::vector<std::vector<size_t>> &faces, unsigned int parent_depth, unsigned int parent_index);

    bool intersectsAtDepth(BVOctree const& other, unsigned int index1, unsigned int index2) const;

    bool isLeaf(unsigned int index) const noexcept;

};


#endif //BVOCTREE_HPP
