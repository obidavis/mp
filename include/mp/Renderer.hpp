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
    virtual void clear(void) = 0;
    virtual void show(void) = 0;
    virtual void drawParticle(const Particle_t &particle) = 0;
    virtual void drawConstraint(const Constraint_t &constraint) = 0;
};
