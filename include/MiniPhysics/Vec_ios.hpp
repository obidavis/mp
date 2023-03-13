#include "Vec.hpp"
#include <iostream>

template <int Dim, typename T>
std::ostream &operator<<(std::ostream &os, const Vec<Dim, T> &vec)
{
    os << "{";
    for (int i = 0; i < Dim - 1; ++i)
        os << vec[i] << ", ";
    os << vec[Dim - 1] << "}";
    return os;
}
