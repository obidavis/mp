#include <cmath>
#include <vector>
#include <chrono>
#include <mp/World.hpp>
#include <mp/common/vec_os.hpp>
#include "ClothRenderer.hpp"


using Vec3 = mp::Vec<3, double>;
using Particle3 = mp::Particle<3, double>;

std::vector<Particle3> makeGrid(Vec3 min, Vec3 max, Vec3 dim)
{
    Vec3 inc = (max - min) / (dim - 1);
    std::vector<Particle3> vec;
    for (int y = 0; y < static_cast<int>(dim.y()); ++y)
    {
        for (int x = 0; x < static_cast<int>(dim.x()); ++x)
        {
            Particle3 particle;
            particle.position.x() = min.x() + x * inc.x();
            particle.position.y() = min.y() + y * inc.y();
            if (y == 0 || x == 0 || x == static_cast<int>(dim.x()) - 1)
                particle.inverseMass = 0.0;
            vec.push_back(particle);
        }
    }
    return vec;
}


int main(int argc, char * argv[]){
    Vec3 min{-70.0, -20.0, -5.0};
    Vec3 max{70.0, 20.0, 5.0};
    Vec3 gridDim = {45.0, 15.0, 0.0};
    auto particles = makeGrid(min, max, gridDim);
    float aspectRatio = ((max.x() - min.x()) / (max.y() - min.y()));
    int winHeight = 200;
    int winWidth = 700;
    mp::NurbsRenderer renderer(winHeight, winWidth, {min.x(), max.y(), min.z()}, {max.x(), min.y(), max.z()});
    

    mp::World<3, double> world;
    std::vector<mp::DistanceConstraint<3, double>> joins;
    std::vector<mp::Triangle<3, double>> polygons;
    for (int y = 0; y < gridDim.y(); ++y)
    {
        for (int x = 0; x < gridDim.x(); ++x)
        {

            int indexTopLeft = x + y * gridDim.x();
            int indexTopRight = indexTopLeft + 1;
            int indexBottomLeft = indexTopLeft + gridDim.x();
            int indexBottomRight = indexBottomLeft + 1;
            
            Particle3 &p0 = particles[indexTopLeft];
            Particle3 &p1 = particles[indexTopRight];
            Particle3 &p2 = particles[indexBottomLeft];
            Particle3 &p3 = particles[indexBottomRight];
            
            bool firstRow = y == 0;
            bool lastRow = y == gridDim.y() - 1;
            bool lastCol = x == gridDim.x() - 1;

            if (!firstRow)
                joins.emplace_back(p0, p1);
            
            if (!lastRow)
                joins.emplace_back(p0, p2);

            if (!lastCol)
                joins.emplace_back(p0, p1);
            
            if (!lastRow && !lastCol)
            {
                polygons.emplace_back(p0.position, p1.position, p2.position);
                polygons.emplace_back(p1.position, p3.position, p2.position);
            }
        }
    }

    world.addParticles({particles}); 
    std::vector<std::reference_wrapper<mp::Constraint<3, double>>> join_refs(joins.begin(), joins.end());
    world.addConstraints({join_refs});
    
    world.setGravity({0, -13, 0}); 

    SDL_Event sdl_event;
    int i = 10;
    while (true)
    {
        
        // update graphics
        while (SDL_PollEvent(&sdl_event))
        {
            switch (sdl_event.type)
            {
                case (SDL_QUIT): 
                    exit(1);
                    break;
                case (SDL_MOUSEBUTTONDOWN):
                {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    auto map = [](auto input, auto input_start, auto input_end, auto output_start, auto output_end) { return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start); };
                    double physX = map(x, 0, winWidth, min.x(), max.x());
                    double physY = map(y, 0, winHeight, max.y(), min.y());
                    Vec3 impulsePos = {physX, physY, 0.0};
                    std::cout << "interaction: " << impulsePos << "\n";
                    double minDistance = std::numeric_limits<double>::max();
                    Particle3 *closestBody = nullptr;
                        for (Particle3 &particle : particles)
                        {
                            double distance = (particle.position - impulsePos).length();
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                closestBody = &particle;
                            }
                        }
                    if (closestBody != nullptr)
                        closestBody->applyImpulse({0.0, 0.0, 200.0});
                    break;
                }
                default:
                    break;
            }
        }            

        // update physics
        static auto start = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start);
        start = now;

        auto step = duration.count() / 1'000'000'000.0;
        world.step(step);   
        
        auto step_duration = std::chrono::high_resolution_clock::now() - start;
        // std::cout << "step_duration: " << std::chrono::duration_cast<std::chrono::microseconds>(step_duration).count() << "\n";
        
        renderer.clear();
       for (mp::Triangle<3, double> &polygon : polygons)
            renderer.drawPolygon(polygon);
//        for (DistanceConstraint<3, double> &join : joins)
//            renderer.drawConstraint(join);
//        for (auto &row : particleRows)
//            for (Particle3 &particle : row)
//                renderer.drawParticle(particle);
        renderer.show();
    }
        


    return 1;
}

