#pragma once

#include "../utility/meta.hpp"
#include "../Vec.hpp"
#include "../utility/maths.hpp"
#include "shape.hpp"

namespace mp {

template <int Dim, typename T, int FrameDim> 
class Frame
{
public:

    using Point_t = Point<Dim, T>;
    using Line_t = Line<Dim, T>;

    virtual void drawPoint(const Point_t &point) {}
    virtual void drawLine(const Line_t &line) {}

protected:
    Vec<FrameDim, int> frameDimensions;
    map_linear<T> phys2frame;
};

}
