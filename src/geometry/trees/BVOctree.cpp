//
// Created by arthur on 01/04/2022.
//

#include "BVOctree.hpp"

BVOctree::BVOctree(const Point &center, std::vector<glm::vec3> &vertices,
                   const std::vector<std::vector<size_t>> &faces):
m_nodes()
{

    // Construct root of tree
    Vector part_halfwidth{MAXFLOAT, MAXFLOAT, MAXFLOAT};
    m_nodes[0] = new BVNode(center, part_halfwidth, 0,vertices, faces);
    Vector halfWidth = m_nodes[0]->getBoundingVolume()->getHalfWidth()/2;
    auto new_cent = m_nodes[0]->getBoundingVolume()->getCenter();
    //builds tree recursively
    buildTree(new_cent, halfWidth, vertices, faces, 0, 0);


}

void BVOctree::buildTree(const Point &center, Vector halfWidth, std::vector<glm::vec3> &vertices,
                         const std::vector<std::vector<size_t>> &faces, unsigned int parent_depth,unsigned int parent_index){

    assert(parent_depth <= MAX_DEPTH);

    if(parent_depth == MAX_DEPTH) return;

    Point offset;
    for(int i = 0; i<8; i++)
    {
        offset.x = ((i & 1) ? halfWidth.x : -halfWidth.x);
        offset.y = ((i & 2) ? halfWidth.y : -halfWidth.y);
        offset.z = ((i & 4) ? halfWidth.z : -halfWidth.z);

        m_nodes[8 * parent_index + (i + 1)] = new BVNode(center + offset, halfWidth, parent_depth + 1, vertices, faces);

        auto bv = m_nodes[8 * parent_index + (i + 1)]->getBoundingVolume();
        if( bv == nullptr )
            m_nodes[8 * parent_index + (i + 1)] = nullptr;
        else
        {
            auto new_center = bv->getCenter();
            auto new_halfWidth = bv->getHalfWidth();
            buildTree(new_center, new_halfWidth/2, vertices, faces, parent_depth + 1, 8 * parent_index + (i + 1));
        }

    }
}

bool BVOctree::intersects(const BVOctree &other) const
{
    return intersectsAtDepth(other, 0,0);
}

bool BVOctree::intersectsAtDepth(const BVOctree &other, unsigned int index1, unsigned int index2) const
{
    auto node1 = m_nodes[index1];
    auto node2 = other.m_nodes[index2];

    if(node1 == nullptr || node2 == nullptr)
        return false;

    if(node1->getBoundingVolume()->intersects(*(node2->getBoundingVolume())))
    {
        // Disjunction on depths of both nodes
        if(isLeaf(index1) && other.isLeaf(index2))
        {
            // If both nodes are leaves compute real intersection
            return node1->intersects(*node2);
        }
        else if(!isLeaf(index1))
        {
            // If node1 is not a leaf node, go down the first tree
            bool intersects = false;
            for(int i = 1; i<9; i++)
                intersects = intersects || intersectsAtDepth(other, 8*index1+i, index2);
            return intersects;
        }
        else
        {
            // If node1 is a leaf and node2 is not a leaf, go down on the second tree
            bool intersects = false;
            for(int i = 1; i<9; i++)
                intersects = intersects || intersectsAtDepth(other, index1, 8*index2+i);
            return intersects;
        }
    }
    else
    {
        return false;
    }


}

void BVOctree::update(const Vector &t)
{
    for(int i=0; i<NB_NODES; i++)
    {
        if(m_nodes[i]!= nullptr)
        {
            m_nodes[i]->getBoundingVolume()->update(t);
        }
    }
}

void BVOctree::setVisible(bool visible, int startDepth) {

    if(visible)
    {
        for(int i=0; i<NB_NODES; i++)
        {
            if(m_nodes[i]!= nullptr)
            {
                if(m_nodes[i]->getDepth()>=startDepth)
                    m_nodes[i]->getBoundingVolume()->setVisible(true);
                else
                    m_nodes[i]->getBoundingVolume()->setVisible(false);
            }
        }
    }
    else
    {
        for(int i=0; i<NB_NODES; i++)
        {
            if(m_nodes[i]!= nullptr)
                m_nodes[i]->getBoundingVolume()->setVisible(false);
        }
    }


}

bool BVOctree::isLeaf(unsigned int index) const noexcept
{
    return m_nodes[index]->getDepth() == MAX_DEPTH;
    /*auto firstChild = 8*index+1;

    if(firstChild>=NB_NODES)
        return true;

    else
    {
        bool isLeaf = true;
        for(int i = 1; i<9; i++)
            isLeaf = isLeaf&& (m_nodes[8*index +i] == nullptr);

        return isLeaf;
    }*/
}

BVNode *BVOctree::getNodeAt(unsigned int i) {
    assert(i<NB_NODES);
    return m_nodes[i];
}




