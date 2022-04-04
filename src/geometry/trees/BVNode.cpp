//
// Created by arthur on 31/03/2022.
//

#include "BVNode.hpp"
#include "../primitives/Triangle.hpp"
#include "../bounding-volumes/AABB.hpp"

int BVNode::nodeDepth(unsigned int key)
{
    for(int d = 0; key; d++)
    {
        if(key==1)
            return d;

        key>>=3;
    }

    assert(0); // Make sure that no bad keys are created
}

unsigned int BVNode::part1By2(unsigned int n)
{
   // if n =    ----------------------9876543210
   // Return is ----9--8--7--6--5--4--3--2--1--0
   n = (n^(n<<16)) & 0xff0000ff;
   n = (n^(n << 8)) & 0x0300f00f;
   n = (n^(n << 4)) & 0x030c30c3;
   n = (n^(n << 2)) & 0x09249249;

   return n;
}

unsigned int BVNode::mortonKey(unsigned int x, unsigned int y, unsigned int z) {
    return (part1By2(z) << 2) + (part1By2(y )<< 1) + part1By2(x);
}

BVNode::BVNode(const Point &center, Vector halfWidth, unsigned int depth, std::vector<glm::vec3> &vertices,
               const std::vector<std::vector<size_t>> &faces):
m_center()
{

    std::set<Point> my_vertices{};
    for(const auto& face : faces)
    {
        assert(face.size() ==3);

        std::vector<unsigned int> good_vertices;
        for(auto v_ind : face)
        {
            auto vertex = vertices[v_ind];
            if( abs(vertex.x - center.x)  <= halfWidth.x  && abs(vertex.y - center.y)  <= halfWidth.y && abs(vertex.z - center.z)  <= halfWidth.z)
            {
                good_vertices.push_back(v_ind);
            }
        }

        if(!good_vertices.empty())
        {
            Point p1{ vertices[face[0]].x,vertices[face[0]].y, vertices[face[0]].z };
            Point p2{ vertices[face[1]].x,vertices[face[1]].y, vertices[face[1]].z };
            Point p3{ vertices[face[2]].x,vertices[face[2]].y, vertices[face[2]].z };

            m_faces.emplace_back(p1, p2, p3);

            if(good_vertices.size()>1)
            {
                my_vertices.insert(p1);
                my_vertices.insert(p2);
                my_vertices.insert(p3);
            }
            else
            {
                auto v_ind = good_vertices[0];
                my_vertices.insert(Point{vertices[v_ind].x, vertices[v_ind].y, vertices[v_ind].z});
            }


        }
    }

    if(m_faces.empty())
    {
        m_boundingVolume = nullptr;
    }
    else
    {
        std::vector<Point> verts_vec(my_vertices.begin(), my_vertices.end());
        m_boundingVolume = BoundingVolume::bestFittingBV(verts_vec);
        m_center = center;
        m_depth = depth;
    }
}

BoundingVolume *BVNode::getBoundingVolume() {
    return m_boundingVolume;
}

unsigned int BVNode::getDepth() const {
    return m_depth;
}

bool BVNode::intersects(BVNode &other)
{
    for(const auto& tri1 : m_faces)
    {

        for(const auto& tri2 : other.m_faces)
        {
            if(tri1.intersects(tri2))
                return true;
        }
    }
    return false;
}
