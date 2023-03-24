#pragma once

#include "../utility/meta.hpp"
#include "../Vec.hpp"
#include <functional>

namespace mp {

template <int Dim, typename T, int nVertices>
struct Shape
{
    using Vec_t = Vec<Dim, T>;
    template <typename ...Ts>
    Shape(Ts &...args) : vertices{std::ref(args)...} {}
    Vec<nVertices, std::reference_wrapper<Vec_t>> vertices;

    inline Vec_t centre(void) const
    {
        Vec_t c;
        for (int i = 0; i < nVertices; ++i)
            c += &(vertices[i]);
        c /= nVertices;
        return c;
    }

    
    template <int N = nVertices, typename meta::enable_if_t<N == 3>>
    inline Vec_t normal(void) const 
    {
        Vec_t u = vertices[1] - vertices[0];
        Vec_t v = vertices[2] - vertices[0];

        return {
            (u.y() * v.z()) - (u.z() * v.y()),
            (u.z() * v.x()) - (u.x() * v.z()),
            (u.x() * v.y()) - (u.y() * v.x())
        };

    }
    
    template <int N = nVertices, typename meta::enable_if_t<N == 2>>
    inline Vec_t normal(void) const 
    {
        Vec_t u = vertices[1] - vertices[0];
        return {-u.y(), u.x()};
    }
};

template <int Dim, typename T>
using Point = Shape<Dim, T, 1>;

template <int Dim, typename T>
using Line = Shape<Dim, T, 2>;

template <int Dim, typename T>
using Triangle = Shape<Dim, T, 3>;

}
