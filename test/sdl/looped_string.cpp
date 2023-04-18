#include <chrono>
#include <vector>
#include <iostream>
#include <mp/common/vec_os.hpp>
#include <mp/World.hpp>
#include <mp/utility/maths.hpp>
#include <mp/dynamics/spring_force.hpp>
#include <mp/rendering/shape.hpp>
#include "SDL_Renderer.hpp"

#include "logistic_medium.hpp"

using namespace mp;

using Vec_t = Vec<2, double>;
using Particle_t = Particle<2, double>;
using Constraint_t = Constraint<2, double>;

inline Vec_t wrappedDistance(const Vec_t &lhs, const Vec_t &rhs)
{
    static mp::wrapped_distance<double> wrap(1.0);
    Vec_t diff = lhs - rhs;
    diff.x() = wrap(diff.x());
    return diff;
}

class _Renderer : public MP_SDL_Renderer<2>
{
public:
    _Renderer(int height, int width, Vec_t min, Vec_t max)
        : MP_SDL_Renderer<2>(height, width), mapPosition(min, max, {0, height}, {width, 0}) {}
    mp::map_linear<Vec_t> mapPosition;
    
    void drawConstraint(const Constraint_t &) {}
    void drawShape(const mp::Line<2, double> &line)
    {
        static mp::map_linear<double> radiansToByte(-6.14, 6.14, 0, 255);
        Vec_t p1Pos = line.vertices[0];   
        Vec_t p2Pos = line.vertices[1];   
        
        Vec_t relativePosition = wrappedDistance(p2Pos, p1Pos);

        Vec_t p1posWrapped = p2Pos - relativePosition;
        Vec_t p2PosWrapped = p1Pos + relativePosition;
        
        double gradient = relativePosition.y() != 0.0 ? relativePosition.x() / relativePosition.y() : 0.0;
        uint8_t theta = radiansToByte(atan(gradient));
        RGBA<uint8_t> colour{static_cast<uint8_t>(theta / 2), theta, theta , 255};
        std::cout << static_cast<int>(theta) << " ";

        double length = (p2Pos - p1Pos).lengthSquared();
        const Vec_t position1 = mapPosition(p1Pos);
        const Vec_t position2 = mapPosition(p2Pos);
        if (length < 0.5)
        {
            drawLine(position1.x(), position1.y(), position2.x(), position2.y(), colour);
        }
        else
        {
            const Vec_t position2Wrapped = mapPosition(p1Pos + relativePosition);
            const Vec_t position1Wrapped = mapPosition(p2Pos - relativePosition);
            drawLine(position1.x(), position1.y(), position2Wrapped.x(), position2Wrapped.y(), colour);
            drawLine(position1Wrapped.x(), position1Wrapped.y(), position2.x(), position2.y(), colour);
        }
        const Vec_t position1Wrapped = mapPosition(p1posWrapped);
        const Vec_t position2Wrapped = mapPosition(p2PosWrapped);
    }

    void drawParticle(const Particle_t &particle) 
    {
        RGBA<uint8_t> colour = {127, 0, 0, 255};
        const Vec_t mappedPosition = mapPosition(particle.position);
        drawCircle(mappedPosition.x(), mappedPosition.y(), 3, colour);
    }
    

};

class WrappedDistanceConstraint : public Constraint<2, double>
{
    using Particle_t = Particle<2, double>;
    using Vec_t = Vec<2, double>;
public:
    WrappedDistanceConstraint(Particle_t &p1, Particle_t &p2) 
        : Constraint<2, double>(p1, p2), wrap(1.0), length((wrappedDistance(this->p1.position, this->p2.position)).length()) 
    {
    }
    void solve(double dt) override
    {
        double constraintMass = this->p1.inverseMass + this->p2.inverseMass;
        if (constraintMass <= 0)
            return;

        Vec<2, double> relativePosition = wrappedDistance(this->p1.position, this->p2.position);
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
    double strength = 1.0;
    double biasFactor = 0.6;
};

void handleEdge(Particle_t &particle)
{
    const Vec_t min = {0.0, -0.01};
    const Vec_t max = {1.0, 1000000.0};

    if (particle.position.x() > max.x())
        particle.position.x() = min.x();
    if (particle.position.x() < min.x())
        particle.position.x() = max.x();
    return;
    if (particle.position.y() < min.y())
    {
        particle.position.y() = min.y();
        particle.linearVelocity.y() *= 0.01;
    }
}

std::vector<Particle_t> particles;

    mp::World<2, double> world;
LogisticMedium medium(-1.0, 1.0, 2.0, 0.0, 0.5, world.gravity);

Vec_t applyForces(Particle_t &particle)
{
    return medium.calculateForce(particle);
}

int main()
{
    int width = 1500;
    int height = 400;
    world.setForceCB(applyForces);
    
    Vec_t physMin = {0.0, -0.03};
    Vec_t physMax = {1.0, 0.03};
    _Renderer renderer(height, width, physMin, physMax);
    world.setPositionCB(handleEdge);
    int nParticles = 30;
    for (int i = 0; i < nParticles; ++i)
    {
        Particle_t p;
        p.position.x() = i / (static_cast<double>(nParticles));
        particles.push_back(p);
    }
    
    std::vector<WrappedDistanceConstraint> constraints;
    for (int i = 0; i < particles.size(); ++i)
    {
        WrappedDistanceConstraint d(particles[i], particles[(i + 1) % particles.size()]);
        constraints.push_back(d);
    }
    
    world.addParticles({particles});
    std::vector<std::reference_wrapper<Constraint_t>> constraint_refs(constraints.begin(), constraints.end());
    world.addConstraints({constraint_refs});
    world.timeStretch = 1.0; 
    world.iterationCount = 2;
    world.stepSize = 0.04;
    world.gravity = {0.0, -9.5};
    world.setDamping(0.3);

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
                        closest->applyImpulse({0.0, 0.5f});
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
            mp::Line<2, double> line(spring.p1.position, spring.p2.position);
            renderer.drawShape(line);

        }
       renderer.show();
       std::cout << "\n";
        
   }

}
