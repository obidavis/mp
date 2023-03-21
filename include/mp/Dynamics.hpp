#ifndef Particle_h_
#define Particle_h_

#include <functional>
#include "Vec.hpp"



template <int Dim, typename ScalarType>
struct Particle;

template <int Dim, typename T>
struct Particle
{
    using Vec_t = Vec<Dim, T>;
    T inverseMass;
    Vec<Dim, T> position, linearVelocity, forceAccumulator;
    Particle() : inverseMass{1.0}, position{}, linearVelocity{}, forceAccumulator{} {}

    void applyImpulse(const Vec_t &impulse) 
    { 
        linearVelocity += impulse * inverseMass; 
    }
    void applyForce(const Vec_t &force)
    {
        forceAccumulator += force;
    }
    void integrateVelocity(T dt)
    {
        const Vec_t acceleration = forceAccumulator * inverseMass;
        linearVelocity += acceleration * dt;
        forceAccumulator = {};
    }
    void integratePosition(T dt)
    {
        position += linearVelocity * dt;
    }

};

template <int Dim, typename ScalarType = double>
using Force = std::function<Vec<Dim, ScalarType>(const Particle<Dim, ScalarType> &)>;



template <int Dim, typename ScalarType = double>
struct Medium
{
    Medium(Force<Dim, ScalarType> gravity, std::function<ScalarType(Vec<Dim, ScalarType>)> density)   
        : gravity(gravity), getDensity(density) {}
    Medium(Force<Dim, ScalarType> gravity, ScalarType density)
        : gravity(gravity), getDensity([density](Vec<Dim, ScalarType> position){ return density; }) {}

    Vec<Dim, ScalarType> operator()(const Particle<Dim, ScalarType> &particle)
    {
        const ScalarType density = getDensity(particle.position);
        //const VecType drag = 

    }

    std::function<ScalarType(Vec<Dim, ScalarType>)> getDensity;

protected:

    Force<Dim, ScalarType> gravity;
};

#endif
