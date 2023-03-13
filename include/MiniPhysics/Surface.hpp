#pragma once

#include "Vec.hpp"

template <typename T>
struct Surface
{
    using Vec_t = Vec<3, T>;
    Vec_t &v1, &v2, &v3;
    Surface(Vec_t &v1, Vec_t &v2, Vec_t &v3) 
        : v1(v1), v2(v2), v3(v3) {}
    Vec_t centre(void) const 
    {
        return (v1 + v2 + v3) / 3.0;
    }
    Vec_t normal(void) const
    {
        Vec_t u = v2 - v1;
        Vec_t v = v3 - v1;

        Vec_t normal;
        
        normal.x() = (u.y() * v.z()) - (u.z() * v.y());
        normal.y() = (u.z() * v.x()) - (u.x() * v.z());
        normal.z() = (u.x() * v.y()) - (u.y() * v.x());

        return normal;
    }
};
