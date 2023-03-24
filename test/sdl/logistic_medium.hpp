#pragma once

#include <mp/dynamics/medium.hpp>
#include <mp/utility/maths.hpp>

class LogisticMedium : public mp::Medium<2, double>
{
public:
    LogisticMedium(double minY, double maxY, double minDensity, double maxDensity, double slope, Vec_t &gravity) 
        : densityMap(minY, maxY, minDensity, maxDensity, slope), gravity(gravity) {}
    Vec_t calculateForce(Particle_t &particle) 
    {
        const double density = densityMap(particle.position.y());
        const Vec_t drag = calculateDrag(density, particle.linearVelocity, 1.0, 1.0);
        const Vec_t buoyancy = calculateBuoyancy(density, 1.0, gravity);
        return drag + buoyancy;
    }

private:
    mp::map_logistic<double> densityMap;
    Vec_t &gravity;

};
