#pragma once

#include "../common/vec.hpp"

namespace mp {

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

}
