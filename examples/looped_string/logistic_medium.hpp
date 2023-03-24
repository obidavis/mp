#pragma once

#include <mp/dynamics/medium.hpp>


class LogisticMedium : public mp::Medium<2, float>
{
public:
    LogisticMedium(float minY, float maxY, float minDensity, float maxDensity, float slope, Vec_t &gravity) 
        : densityMap(minY, maxY, minDensity, maxDensity, slope), gravity(gravity) {}
    Vec_t calculateForce(Particle_t &particle) 
    {
        const float density = densityMap(particle.position.y());
        const Vec_t drag = calculateDrag(density, particle.linearVelocity, 1.0, 1.0);
        const Vec_t buoyancy = calculateBuoyancy(density, 1.0, gravity);
        return drag + buoyancy;
    }

private:
    mp::map_logistic<float> densityMap;
    Vec_t &gravity;

};
