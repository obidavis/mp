#include <cmath>
#include <vector>
#include <chrono>
#include "../../include/mp/Vec_ios.hpp"
#include "../../include/mp/World.hpp"
#include "SDL_Renderer.hpp"


using Vec3 = Vec<3, double>;
using Particle3 = Particle<3, double>;

struct Damping
{
    double damping = 0.4;
    Vec3 operator()(Particle3 particle) { return particle.linearVelocity * -damping; }
};

Vec3 Gravity(const Particle3 &particle) { return Vec3{0, -13.0, 0} * particle.mass; }

std::vector<std::vector<Particle3>> makeGrid(Vec3 min, Vec3 max, Vec<3, double> dim)
{
    Vec<3, double> inc = (max - min) / (dim - 1);
    std::vector<std::vector<Particle3>> vec;
    for (int y = 0; y < static_cast<int>(dim.y()); ++y)
    {
        std::vector<Particle3> row;
        for (int x = 0; x < static_cast<int>(dim.x()); ++x)
        {
            Particle3 particle(1.0, 1.0);
            particle.position.x() = min.x() + x * inc.x();
            particle.position.y() = min.y() + y * inc.y();
            row.push_back(particle);
        }
        vec.push_back(row);
    }
    return vec;
}


int main(int argc, char * argv[]){
    Vec3 min{-70.0, -20.0, -5.0};
    Vec3 max{70.0, 20.0, 5.0};
    auto particleRows = makeGrid(min, max, {45.0, 15.0, 0.0});
    float aspectRatio = ((max.x() - min.x()) / (max.y() - min.y()));
    int winHeight = 200;
    int winWidth = 700;
    MP_SDL_Renderer renderer(winHeight, winWidth, {min.x(), max.y(), min.z()}, {max.x(), min.y(), max.z()});
    

    mp::World<3, double> world;
    Damping damping;
//    std::vector<std::vector<Particle3>> particleRows;
//    std::vector<Particle3> topRow;
//    for (int i = 0; i < 15; ++i)
//    {
//        Particle3 particle(0.4, 1.0);
//        particle.inverseMass = 0.0;
//        particle.position = {static_cast<double>(i), 5.0};
//        topRow.push_back(particle);
//    }
//    particleRows.push_back(topRow);
//    for (int r = 1; r <= 5; ++r)
//    {
//        std::vector<Particle3> row;
//        for (int i = 0; i < 15; i++)
//        {
//            Particle3 particle(0.4, 1.0);
//            particle.position = {static_cast<double>(i), 5.0 - static_cast<double>(r) * 0.5};
//            row.push_back(particle);
//        }
//        particleRows.push_back(row);
//    }
    for (auto &p : particleRows[particleRows.size() - 1])
        p.inverseMass = 0.0;
//    for (auto &p : particleRows[0])
//        p.inverseMass = 0.0;
    for (auto &row : particleRows)
    {
        row[0].inverseMass = 0.0;
        row[row.size() - 1].inverseMass = 0.0;
    }
    std::vector<DistanceConstraint<3, double>> joins;
    for (int r = 1; r < particleRows.size(); ++r)
    {
        for (int c = 0; c < particleRows[r].size(); ++c)
        {
            joins.emplace_back(particleRows[r][c], particleRows[r - 1][c]);
            if (c > 0)
                joins.emplace_back(particleRows[r][c], particleRows[r][c - 1]);
        }
    }


    world.addForce(Gravity);
    for (auto &row : particleRows)
        for (Particle3 &particle : row)
            world.addParticle(&particle);
    
    for (DistanceConstraint<3, double> &join : joins)
        world.addConstraint(&join);

    world.addForce(Gravity);
    world.addForce(damping);
    
    std::vector<Surface<double>> polygons;
    for (int i = 0; i < particleRows.size() - 1; ++i)
    {
        std::vector<Particle3> &row = particleRows[i];
        std::vector<Particle3> &rowBelow = particleRows[i + 1];
        for (int j = 0; j < row.size() - 1; ++j)
        {
            Particle3 &p1 = row[j];
            Particle3 &p2 = row[j + 1];
            Particle3 &p3 = rowBelow[j];
            Particle3 &p4 = rowBelow[j + 1];
            
            polygons.emplace_back(p1.position, p2.position, p3.position);
            polygons.emplace_back(p2.position, p4.position, p3.position);
        } 
    }

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
                    double minDistance = 999999999.9;
                    Particle3 *closestBody = nullptr;
                    for (auto &row : particleRows)
                        for (Particle3 &particle : row)
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
       for (Surface<double> &polygon : polygons)
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

