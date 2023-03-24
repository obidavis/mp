#pragma once
#include <cmath>
#include <type_traits>
#include "../Vec.hpp"
#ifdef ARDUINO
#include <arm_math.h>
#endif

namespace mp {
    
    float exp_fun(float f) { return expf(f); }
    double exp_fun(double d) { return exp(d); }
    template <typename T>
    typename std::enable_if<std::is_integral<T>::value, T>::type
    exp_fun(T i) { return expf(static_cast<float>(i)); }
    

    template <typename T>
    struct map_linear
    {
        map_linear(T inputMin, T inputMax, T outputMin, T outputMax)
            : inputMin(inputMin), outputMin(outputMin),
            slope((outputMax - outputMin) / (inputMax - inputMin)) {}
        T operator()(T val) { return outputMin + slope * (val - inputMin); }
    private:
        T inputMin, outputMin;
        T slope;
    };
    
    template <typename T>
    struct map_logistic
    {
        map_logistic(T inputMin, T inputMax, T outputMin, T outputMax, T slope = 1.0)
            : outputMin(outputMin), outputRange(outputMax - outputMin), slope(slope),
            midPoint(inputMin + ((inputMax - inputMin) / 2)), 
            exponent(-9.2 / (inputMax - inputMin))
       {}

        T operator()(T val) 
        {
            return outputMin + ((outputRange) / (1 + exp_fun(slope * exponent * (val - midPoint))));
        }
        T outputMin;
        T outputRange;
        T slope; 
        T midPoint;
        T exponent;

    };
    
    template <typename T>
    struct wrapped_distance
    {
        wrapped_distance(T range) : range(range) {}
        template <typename U = T, typename meta::enable_if_t<std::is_same<U, float>::value, int> = 0>
        T operator()(T val) { return fmodf(fmodf(val, range) + range * 1.5f, range) - range * 0.5f; }
        template <typename U = T, typename meta::enable_if_t<std::is_same<U, double>::value, int> = 0>
        T operator()(T val) { return fmod(fmod(val, range) + range * 1.5, range) - range * 0.5; }
    private:
        T range;
    };
 };
