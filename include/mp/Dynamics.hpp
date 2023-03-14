#ifndef Particle_h_
#define Particle_h_

#include <functional>
#include "Vec.hpp"



template <int Dim, typename ScalarType>
struct Particle;

template <int Dim, typename ScalarType>
struct Particle
{
    ScalarType mass, inverseMass, drag;
    Vec<Dim, ScalarType> position, linearVelocity, acceleration;
    Particle(ScalarType mass, ScalarType drag) 
        : mass(mass), inverseMass(1/ mass), drag(drag),
          position{}, linearVelocity{}, acceleration{} {}
    void applyImpulse(Vec<Dim, ScalarType> impulse) { linearVelocity += impulse * inverseMass; }
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
