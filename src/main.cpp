#include "polyscope/polyscope.h"
#include "geometry/bounding-volumes/AABB.hpp"
#include "geometry/bounding-volumes/Sphere.hpp"
#include "geometry/Object.hpp"
#include "geometry/primitives/Triangle.hpp"
#include "geometry/primitives/Plane.hpp"
#include "geometry/traits/Instantiable.hpp"
#include "geometry/defs.hpp"
#include <chrono>


// == Initialize instances count
unsigned int Instantiable::m_instances = 0;
std::vector<Object> objects;
std::chrono::high_resolution_clock::time_point last_refresh = std::chrono::high_resolution_clock::now();


bool show_bv = false;
bool run_sim = false;
bool compute_intersections = false;
int depth_to_show = 0;

void oneSimulationStep(float dt)
{

    for(auto & object : objects)
    {
        object.update(dt);
        object.showOctree(show_bv, depth_to_show);
    }
}


void simulationCallback()
{

    if(ImGui::Checkbox("Show Bounding Volumes", &show_bv))
    {
        for(auto obj :objects)
            obj.showOctree(show_bv, depth_to_show);
    }

    ImGui::InputInt("Depth to show", &depth_to_show);

    if (ImGui::Checkbox("Run", &run_sim))
    {
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

        if(compute_intersections)
        {
            // First everything back to green
            for(const auto& obj : objects)
                obj.setColor(GREEN_COLOR);

            // Then Compute intersections
            for(unsigned int i = 0; i<objects.size(); i++)
            {
                for(unsigned int j = i+1; j< objects.size(); j++)
                {
                    if(objects[i].intersects(objects[j]))
                    {
                        objects[i].setColor(RED_COLOR);
                        objects[j].setColor(RED_COLOR);
                    }
                }

            }
        }
    }

    if(ImGui::Checkbox("Check intersections", &compute_intersections))
    {
        // Register the checkbox
        for(const auto& obj : objects)
            obj.setColor(GREEN_COLOR);

        // Compute intersections only if the button was checked or when simulation is running
        if(compute_intersections)
        {
            // Compute intersections
            for(unsigned int i = 0; i<objects.size(); i++)
            {
                for(unsigned int j = i+1; j< objects.size(); j++)
                {
                    if(objects[i].intersects(objects[j]))
                    {
                        objects[i].setColor(RED_COLOR);
                        objects[j].setColor(RED_COLOR);
                    }
                }

            }
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
    for(int i=1; i<argc; i++)
    {
        Vector speed{distr(eng), distr(eng), distr(eng)};
        objects.emplace_back(argv[i],speed);
    }



    /*
    std::random_device rd2;
    std::default_random_engine eng2(rd());
    std::uniform_int_distribution<unsigned int> distr2(10, 30);
    std::vector<Triangle> triangles;
    for(unsigned int i = 0; i<10; i++) {
        Point p1 = {distr(eng), distr(eng), distr(eng)};
        Point p2 = {distr(eng), distr(eng), distr(eng)};
        Point p3 = {distr(eng), distr(eng), distr(eng)};
        auto tri1 = Triangle(p1, p2, p3);
        tri1.setVisible(true);
        tri1.setColor(GREEN_COLOR);
        triangles.push_back(tri1);

        std::cout << "Triangle " << tri1.m_mesh->name << std::endl;
        std::cout << "Point p" << 3 * i << " = " << p1 << ";" << std::endl;
        std::cout << "Point p" << 3 * i + 1 << " = " << p2 << ";" << std::endl;
        std::cout << "Point p" << 3 * i + 2 << " = " << p3 << ";" << std::endl;

    }

    for(unsigned int i =0; i< triangles.size(); i++)
    {
        for(unsigned int j=i+1; j<triangles.size(); j++)
        {
            if(triangles[i].intersects(triangles[j]))
            {
                triangles[i].setColor(RED_COLOR);
                triangles[j].setColor(RED_COLOR);

                std::cout << "Triangle " << triangles[i].m_mesh->name << " intersects triangle " << triangles[j].m_mesh->name << std::endl;
                //triangles[i].updateMesh();
                //triangles[j].updateMesh();
            }
        }
    }*/

    /*Point p1 = {distr(eng),distr(eng),distr(eng)};
    Point p2 = {distr(eng),distr(eng),distr(eng)};
    Point p3 = {distr(eng),distr(eng),distr(eng)};
    Triangle t1 = Triangle(p1,p2,p3);
    t1.setVisible(true);
    t1.setColor(GREEN_COLOR);

    Point p4{1,0,0};
    Point p5{-1,-1,-2};
    Point p6{-1,2,3};
    Triangle t2 = Triangle(p4,p5,p6);
    t2.setVisible(true);
    t2.setColor(GREEN_COLOR);

    if(t1.intersects(t2))
    {
        t1.setColor(RED_COLOR);
        t2.setColor(RED_COLOR);
    }*/

    /*Point p18 = {-2.01937, 2.45639, -1.50424};
    Point p19 = {2.23096, 1.8202, -1.95515};
    Point p20 = {1.83384, -0.577413, -2.57916};
    Triangle t1(p18,p19,p20);
    t1.setVisible(true);
    t1.setColor(GREEN_COLOR);

    Point p6 = {1.53023, 2.51265, 2.16773};
    Point p7 = {1.00514, -2.66857, -0.705039};
    Point p8 = {0.412282, -0.778774, -2.84686};
    Triangle t2(p6,p7,p8);
    t2.setVisible(true);
    t2.setColor(GREEN_COLOR);

    if(t1.intersects(t2))
    {
        t1.setColor(RED_COLOR);
        t2.setColor(RED_COLOR);
    }*/

    // Adding the callback
    polyscope::state::userCallback = simulationCallback;

    polyscope::show();

    return EXIT_SUCCESS;
}