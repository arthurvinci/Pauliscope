#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "geometry/bounding-volumes/AABB.hpp"
#include "geometrycentral/surface/meshio.h"
#include "geometry/bounding-volumes/Sphere.hpp"
#include "geometry/Object.hpp"
#include "geometry/primitives/Triangle.hpp"
#include "geometry/primitives/Plane.hpp"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// == Initialize instances count
unsigned int BoundingVolume::bv_instances = 0;

int main(int argc, char **argv) {

    // Initialize polyscope
    polyscope::init();


    // Load mesh
    //std::tie(mesh, geometry) = readManifoldSurfaceMesh(argv[1]);


    // Register the mesh with polyscope
    //polyscope::registerSurfaceMesh("Input obj",geometry->inputVertexPositions,mesh->getFaceVertexList(),
    //                                         polyscopePermutations(*mesh));

    //Object input_obj{geometry->inputVertexPositions, SPHERE_ID};
    //input_obj.getBoundingVolume()->registerMesh();


    //Object aabb1_obj{new AABB(4, 5, 7, 5, 3, 11) };
    //aabb1_obj.getBoundingVolume()->registerMesh();

    //std::vector<std::array<float,3>> tmp = aabb1_obj.getBoundingVolume()->getMeshVertices();
    /*
    std::vector<Point> vertices;
    for(auto arr : tmp)
    {
        Point p{};
        for(int i =0; i<3; i++)
            p[i] = arr[i];

        vertices.push_back(p);
    }
     */


   /* Point p1 = {0,0,0};
    Point p2 = {1,0,0};
    Point p3 = {0,1,0};
    Point p4 = {0,0,1};
    Point p5 = {-6,7,3.45};


    std::vector<Point> pts;
    pts.push_back(p1);
    pts.push_back(p2);
    pts.push_back(p3);
    pts.push_back(p4);
    pts.push_back(p5);

    std::vector<Point> pt2;

    Sphere sphere_from_v(pts);
    sphere_from_v.registerMesh();

    polyscope::registerPointCloud("test", pts);
    */

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(-3, 3);

    Sphere sp1(Point{distr(eng), distr(eng), distr(eng)}, abs(distr(eng)));
    //Sphere sp2(Point{distr(eng), distr(eng), distr(eng)}, abs(distr(eng)));
    Point p1{distr(eng), distr(eng), distr(eng)};
    Point p2{distr(eng), distr(eng), distr(eng)};
    Point p3{distr(eng), distr(eng), distr(eng)};

    Triangle tri(&p1,&p2,&p3, "SEUM");
    sp1.registerMesh();
    tri.registerMesh();
    std::cout << sp1.intersects(tri) << std::endl;

   polyscope::show();

    return EXIT_SUCCESS;
}