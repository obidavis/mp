#include <chrono>
#include <vector>
#include <iostream>
#include <mp/Vec_ios.hpp>
#include <mp/World.hpp>
#include <mp/utility/maths.hpp>
#include <mp/dynamics/spring_force.hpp>
#include <mp/rendering/shape.hpp>
#include "SDL_Renderer.hpp"


class _Renderer : public MP_SDL_Renderer<2>
{
public:
    _Renderer(int height, int width, Vec_t min, Vec_t max)
        : MP_SDL_Renderer<2>(height, width), mapPosition(min, max, {0, height}, {width, 0}) {}
    mp::map_linear<Vec_t> mapPosition;
    
    void drawConstraint(const Constraint_t &) override {}
    void drawShape(mp::Line<2, double> line)
    {
        static mp::map_linear<double> radiansToByte(-2.14, 2.14, 0, 255);
        Vec_t p1Pos = *line.vertices.x();   
        Vec_t p2Pos = *line.vertices.y();   

        Vec_t relpos = (p2Pos - p1Pos).normalised();
        if (relpos.x() < 0.0)
        {
            return;
        }
        double gradient = relpos.y() != 0.0 ? relpos.x() / relpos.y() : 0.0;
        uint8_t theta = radiansToByte(atan(gradient));
        RGBA<uint8_t> colour{static_cast<uint8_t>(theta / 2), theta, theta , 255};
        const Vec_t position1 = mapPosition(p1Pos);
        const Vec_t position2 = mapPosition(p2Pos);
        drawLine(position1.x(), position1.y(), position2.x(), position2.y(), colour);
    }

    void drawParticle(const Particle_t &particle) override
    {
        RGBA<uint8_t> colour = {127, 0, 0, 255};
        const Vec_t mappedPosition = mapPosition(particle.position);
        drawCircle(mappedPosition.x(), mappedPosition.y(), 3, colour);
    }
    

};
using Vec_t = Vec<2, double>;
using Particle_t = Particle<2, double>;
using Constraint_t = Constraint<2, double>;


class WrappedDistanceConstraint : public Constraint<2, double>
{
    using Particle_t = Particle<2, double>;
    using Vec_t = Vec<2, double>;
    Vec_t wrappedDistance(Vec_t &lhs, Vec_t &rhs, double mod)
    {
        Vec_t diff = lhs - rhs;
        diff.x() = wrap(diff.x());
        return diff;
    }
public:
    WrappedDistanceConstraint(Particle_t &p1, Particle_t &p2) 
        : Constraint<2, double>(p1, p2), wrap(1.0), length((wrappedDistance(this->p1.position, this->p2.position, 1.0)).length()) 
    {
    }
    void solve(double dt) override
    {
        double constraintMass = this->p1.inverseMass + this->p2.inverseMass;
        if (constraintMass <= 0)
            return;

        Vec<2, double> relativePosition = wrappedDistance(this->p1.position, this->p2.position, 1.0);
        double distance = relativePosition.length();
        double offset = length - distance;
        offset *= strength;
        Vec<2, double> offsetDir = relativePosition.normalised();
        Vec<2, double> relativeVelocity = this->p1.linearVelocity - this->p2.linearVelocity;
        double velocityDot = Vec<2, double>::dot(relativeVelocity, offsetDir);
        double bias = -(biasFactor / dt) * offset;
        double lambda = -(velocityDot + bias) / constraintMass;
        
        this->p1.applyImpulse(offsetDir * lambda);
        this->p2.applyImpulse(-offsetDir * lambda);
    }
    mp::wrapped_distance<double> wrap;
    bool log = false;
    double length;
    double strength = 0.05;
    double biasFactor = 0.3;
};

class _EdgeHandler : public mp::EdgeHandlerBase<2, double>
{
public:
    _EdgeHandler(Vec_t min, Vec_t max) : min(min), max(max) {}
   void handleEdge(Particle_t *particle) override
   {
        if (particle->position.x() > max.x())
            particle->position.x() = min.x();
        if (particle->position.x() < min.x())
            particle->position.x() = max.x();
        if (particle->position.y() < min.y())
        {
            particle->position.y() = min.y();
            particle->linearVelocity.y() *= 0.01;
        }
   }
private:
   Vec_t min;
   Vec_t max;
};
std::vector<Particle_t> particles;

int main()
{
    _EdgeHandler edge({0.0, -0.01}, {1.0, 100000.0});
    mp::World<2, double> world(&edge);
    
    int nParticles = 50;
    for (int i = 0; i < nParticles; ++i)
    {
        Particle_t p;
        p.position.x() = i / (static_cast<double>(nParticles) * 0.98);
        particles.push_back(p);
    }
    
    std::vector<WrappedDistanceConstraint> constraints;
    for (int i = 0; i < particles.size(); ++i)
    {
        WrappedDistanceConstraint d(particles[i], particles[(i + 1) % particles.size()]);
        constraints.push_back(d);
    }

    for (auto &p : particles)
        world.addParticle(&p);
    for (auto &c : constraints)
        world.addConstraint(&c);

    // world.setUserCB(applySprings);

    world.gravity = {0.0, -9.0};

    int width = 1000;
    int height = 200;

    Vec_t physRange = particles.back().position - particles.front().position;
    physRange.y() = 5.0;
    Vec_t physMin = {particles.front().position.x(), -0.1}  ;
    Vec_t physMax = {particles.back().position.x(), 0.5};
    _Renderer renderer(height, width, physMin, physMax);
    mp::map_linear<Vec<2, double>> winMap({0, height}, {width, 0}, physMin, physMax);
    
   SDL_Event sdl_event;
   while (true)
   {
       while (SDL_PollEvent(&sdl_event))
       {
           switch (sdl_event.type)
           {
                case (SDL_QUIT):
                   exit(1);
                case (SDL_MOUSEBUTTONDOWN):
                {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    Vec<2, double> pos = winMap({x, y});
                    double minDistance = std::numeric_limits<double>::max();
                    Particle_t *closest = nullptr;
                    for (Particle_t &p : particles)
                    {
                        double dist = (pos - p.position).length();
                        if (dist < minDistance)
                        {
                            minDistance = dist;
                            closest = &p;
                        }
                    }
                    if (closest != nullptr)
                        closest->applyImpulse({0.0, 7.5 + pos.y() * 30});
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
        
        renderer.clear();
        for (auto &spring : constraints)
        {
            mp::Line<2, double> line;
            line.vertices = {&spring.p1.position, &spring.p2.position};
            renderer.drawShape(line);
        }
       renderer.show();

   }

}
