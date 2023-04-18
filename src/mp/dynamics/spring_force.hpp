#pragma once

#include "../common/vec.hpp"

namespace mp {

template <int Dim, typename T>
struct DampedSpring
{
protected:
    using Vec_t = Vec<Dim, T>;
    
    inline static Vec_t calculateDampingForce(Vec_t vel, T dampingCoefficient, T k, T mass) 
    {
        return vel * (dampingCoefficient * 2 * std::sqrt(k * mass));
    }

    inline static Vec_t calculateSpringForce(T distance, T relaxedDistance, const Vec_t &direction, T k) 
    {
        return ((distance - relaxedDistance) * k) * direction;
    }
};

}


