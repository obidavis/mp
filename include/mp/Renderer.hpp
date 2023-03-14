#include "Vec.hpp"
#include "Dynamics.hpp"
#include "Constraint.hpp"

template <typename T = uint8_t>
struct RGBA
{
    T r;
    T g;
    T b;
    T a;
};

template <int Dim, typename T>
class Renderer
{
protected:
    using Vec_t = Vec<Dim, T>;
    using Particle_t = Particle<Dim, T>;
    using Constraint_t = Constraint<Dim, T>;

public:
    Renderer(Vec_t inputMin, Vec_t inputMax, Vec_t outputMin, Vec_t outputMax)
        : inputMin(inputMin)
        , outputMin(outputMin)
        , slope((outputMax - outputMin) / (inputMax - inputMin)) 
    {}
    virtual void clear(void) = 0;
    virtual void show(void) = 0;
    virtual void drawParticle(const Particle_t &particle) = 0;
    virtual void drawConstraint(const Constraint_t &constraint) = 0;
    // https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
    Vec_t mapPosition(const Vec_t &position) const 
    {
        return outputMin + slope * (position - inputMin);
    }
protected:
    Vec_t inputMin;
    Vec_t outputMin;
    Vec_t slope;
};
