#include "polyscope/polyscope.h"
#include "geometry/bounding-volumes/AABB.hpp"
#include "geometry/bounding-volumes/Sphere.hpp"
#include "geometry/Object.hpp"
#include "geometry/primitives/Triangle.hpp"
#include "geometry/primitives/Plane.hpp"
#include "geometry/traits/Instantiable.hpp"
#include <chrono>


// == Initialize instances count
unsigned int Instantiable::m_instances = 0;
std::vector<Object> objects;
std::vector<Vector> speeds;
std::chrono::high_resolution_clock::time_point last_refresh = std::chrono::high_resolution_clock::now();


bool show_bv = false;
bool run_sim = false;


void oneSimulationStep(float dt)
{

    for(auto i = 0; i < objects.size(); i++)
    {
        objects[i].update(speeds[i]*dt);
        objects[i].getBoundingVolume()->setVisible(true);
    }
}


void simulationCallback()
{

    if(ImGui::Checkbox("Show Bounding Volumes", &show_bv))
    {
        for(auto obj :objects)
            obj.getBoundingVolume()->setVisible(show_bv);
    }

    if (ImGui::Button("Run"))
    {
        run_sim= !run_sim;
        last_refresh = std::chrono::high_resolution_clock::now();
    }

    if(run_sim)
    {
        std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> time_span = t - last_refresh;
        if(time_span > std::chrono::duration<double,std::milli>(16))
        {
            std::cout << "time span : " << time_span.count()/1000 << " s"<< std::endl;
            oneSimulationStep((time_span.count()) / 1000.);
            last_refresh = t;
        }
    }


}

int main(int argc, char **argv) {

    // Initialize polyscope
    polyscope::init();

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> distr(-3, 3);

    // Load objects
    //for(int i = 1; i<argc; i++)
    //{
        //objects.emplace_back(argv[i]);
        /*auto t1 =new AABB(Point{3,3,3}, 2,2,2);
        objects.emplace_back(t1->getMesh());
        Vector v1{distr(eng), distr(eng), distr(eng)};
        speeds.push_back(v1);

        auto t2 =new AABB(Point{0,0,0}, 2,2,2);
        objects.emplace_back(t2->getMesh());
        Vector v2{distr(eng), distr(eng), distr(eng)};
        speeds.push_back(v2);*/
        //}
    for(int i=1; i<argc; i++)
    {
        objects.push_back(Object(argv[i], SPHERE_ID));
        Vector v1{distr(eng), distr(eng), distr(eng)};
        speeds.push_back(v1);
    }


    std::array<float,3> p1 = {0,0,0};
    std::array<float,3> p2 = {0,1,0};
    std::array<float,3> p3 = {0,0,1};

    std::vector<std::array<float,3>> vertices = {p1,p2,p3};
    std::vector<std::vector<unsigned int >> faces = {{0,1,2}};
   // mesh = polyscope::registerSurfaceMesh("test", vertices,faces);


    std::array<float,3> p4 = {0,0,0};
    std::array<float,3> p5 = {0,1,0};
    std::array<float,3> p6 = {0,0,1};

    std::vector<std::array<float,3>> vertices2 = {p4,p5,p6};
    std::vector<std::vector<unsigned int >> faces2 = {{0,1,2}};
    //polyscope::registerSurfaceMesh("t2", vertices2, faces2);

    // Adding the callback
    polyscope::state::userCallback = simulationCallback;

    polyscope::show();

    return EXIT_SUCCESS;
}