#pragma once

#include "utility/debug.hpp"
#include "utility/range.hpp"
#include "dynamics/particle.hpp"
#include "constraints/constraint.hpp"
#include "common/vec.hpp"

namespace mp {

template<int Dim, typename T >
class World
{
    // typedefs for current template types
    using Vec_t = Vec<Dim, T>;
    using Particle_t = Particle<Dim, T>;
    using Constraint_t = Constraint<Dim, T>;
    using particle_cb_fn = void (*)(Particle<Dim, T> &);
    using force_cb_fn = Vec_t (*)(Particle<Dim, T> &);
    using user_cb_fn = void (*)(void);
public:
    void addParticles(contiguous_range<Particle_t> _particles) { particles = _particles; }
    void addConstraints(contiguous_range<std::reference_wrapper<Constraint_t>> _constraints) { constraints = _constraints; }
    void setForceCB(force_cb_fn cb) { force_cb = cb; } 
    void setUserCB(user_cb_fn cb) { user_cb = cb; } 
    void setPositionCB(particle_cb_fn cb) { position_handler = cb; }
    void setGravity(Vec_t g) { gravity = g; }
    void setDamping(T d) { damping = d; }
    void step(T dt)
    {
        static T prevDt = stepSize;
        dtAccumulator += dt;
        bool didUpdate = false;
        while (dtAccumulator >= stepSize)
        {
            didUpdate = true;
            // apply gravity, damping and user forces to all particles
            // then integtrate tentative velocity
            for (Particle_t &particle : particles)
            {           
                if (particle.inverseMass != T{})
                    particle.applyForce(gravity / particle.inverseMass);

                particle.applyForce(-damping * particle.linearVelocity);

                if (force_cb)
                    particle.applyForce(force_cb(particle));

                particle.integrateVelocity(stepSize * timeStretch);
            }
            
            // post-integration user callback
            if (user_cb != nullptr)
                user_cb();
            
            // solve constraints iteratively
            T iterationDt = (stepSize * timeStretch) / static_cast<T>(iterationCount);
            for (int i = 0; i < iterationCount; ++i)
            {
                for (Constraint_t &constraint : constraints)
                {
                   constraint.solve(iterationDt);
                }
            }

            // integrate positions and call user position fn
            for (Particle_t &particle : particles)
            {
                particle.integratePosition(stepSize * timeStretch);
                if (position_handler)
                    position_handler(particle);
            }

            dtAccumulator -= stepSize;
        }
        
        if (didUpdate)
        {
            T diff = dt - prevDt;
            isDeathSpiralling = diff > stepSize;
            prevDt = dt;
        }



    }     

    contiguous_range<Particle_t> particles;
    contiguous_range<std::reference_wrapper<Constraint_t>> constraints;
    force_cb_fn force_cb = nullptr;
    particle_cb_fn position_handler = nullptr;
    user_cb_fn user_cb = nullptr;
    Vec_t gravity{};
    int iterationCount = 5;
    T timeStretch = 1.0;
    T stepSize = 0.01;
    T damping = 0.3;
    T dtAccumulator{}; 
    bool isDeathSpiralling = false;
};

}
