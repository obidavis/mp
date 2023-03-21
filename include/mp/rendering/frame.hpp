#pragma once

#include "../utility/meta.hpp"
#include "../Vec.hpp"
#include "../utility/maths.hpp"
#include "shape.hpp"

namespace mp {

template <int Dim, typename T, int FrameDim> 
class Frame
{
     
protected:
    Vec<FrameDim, int> frameDimensions;
    map_linear<T> phys2frame;
};

}
