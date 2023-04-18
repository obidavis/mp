#pragma once

#include "../common/vec.hpp"
#include "particle.hpp"
#include "../utility/meta.hpp"
#include "../utility/maths.hpp"

namespace mp {

template <int Dim, typename T>
class Medium
{
protected:
    using Vec_t = Vec<Dim, T>;
    using Particle_t = Particle<Dim, T>;

    static inline Vec_t calculateDrag(const T density, const Vec_t &velocity, const T area, const T dragCoefficient)
    {
        const Vec_t velDir = velocity.normalised();
        const T velMagnitude = velocity.length();
        const T dragMagnitude = 0.5f * density * velMagnitude * velMagnitude * mp_sqrt(area / 3.14159f) * dragCoefficient;
        const Vec_t drag = dragMagnitude * -velDir;
        return drag;
    }

    static inline Vec_t calculateBuoyancy(const T density, const T area, const Vec_t gravity)
    {
        return area * density * -gravity;
    }
};

}

