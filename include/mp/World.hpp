#ifndef World_h_
#define World_h_

#include "utility/debug.hpp"
#include "Dynamics.hpp"
#include "Constraint.hpp"
#include "Container.hpp"
#include "EdgeHandlers.hpp"
#include "dynamics/spring_force.hpp"
#include "Vec.hpp"

namespace mp {

template<int Dim, typename ScalarType = double>
class World
{
    using VecType = Vec<Dim, ScalarType>;
    using Particle_t = Particle<Dim, ScalarType>;
    using Force_t = Force<Dim, ScalarType>;
    using Medium_t = Medium<Dim, ScalarType>;
    using unary_force_cb_t = void (*)(Particle<Dim, ScalarType> *);
    using user_cb_t = void (*)(void);
public:
    World() : edgeHandler(new EdgeHandlerBase<Dim, ScalarType>()) {}
    World(EdgeHandlerBase<Dim, ScalarType> *edgeHandler) : edgeHandler(edgeHandler) {}
    ~World() { delete edgeHandler; }
    void setRandomSeed(float s) { randomSeed = s; }
    void addParticle(Particle_t *particle) { particles.push_back(particle); }
    void addForce(Force_t force) { forces.push_back(force); }
    void addConstraint(Constraint<Dim, ScalarType> *constraint) { constraints.push_back(constraint); }
    void setForceCB(unary_force_cb_t cb) { force_cb = cb; } 
    void setUserCB(user_cb_t cb) { user_cb = cb; } 
    void setGravity(VecType g) { gravity = g; }
    void addMedium(Medium_t *medium)
    {       
        addForce(medium);
        //densityFunctions.push_back(std::ref(medium.densityFunction));
    }   
    //float densityAt(Vec2d pos);
    // template <typename T>
    // bool addForce(T &&force) { return addForce(std::ref(force)); }
    void step(ScalarType dt)
    {
        dtAccumulator += dt;
        int updateCounter = 0;
        while (dtAccumulator >= stepSize)
        {
            updateCounter++;
//            std::cout << "dt: " << dt << "\n";
//            std::cout << "dtAccumulator: " << dtAccumulator << "\n";
            for (Particle_t *particle : particles)
            {           

                if (particle->inverseMass != ScalarType{})
                    particle->applyForce(gravity / particle->inverseMass);
                particle->applyForce(-damping * particle->linearVelocity);
                if (force_cb != nullptr)
                    force_cb(particle);
                particle->integrateVelocity(stepSize);
            }
            if (user_cb != nullptr)
                user_cb();
            int iterationCount = 10;
            ScalarType iterationDt = stepSize / static_cast<ScalarType>(iterationCount);
            for (int i = 0; i < iterationCount; ++i)
            {
                for (Constraint<Dim, ScalarType> *constraint : constraints)
                {
                   constraint->solve(iterationDt);
                }
            }
            for (Particle_t *p : particles)
            {
                p->integratePosition(stepSize);
                edgeHandler->handleEdge(p);
            }

            dtAccumulator -= stepSize;
        }
        
        
        static ScalarType prevDt = 0.0;
        ScalarType diff = dt - prevDt;
        prevDt = dt;
        isDeathSpiralling = diff > stepSize;

    }     
    unary_force_cb_t force_cb = nullptr;
    user_cb_t user_cb = nullptr;
    VecType gravity{};
    vector<Particle_t *> particles;
    vector<Force_t> forces;
    vector<Constraint<Dim, ScalarType> *> constraints;
    EdgeHandlerBase<Dim, ScalarType> *edgeHandler;
    float randomSeed = 0.0f;
    ScalarType stepSize = 0.01;
    ScalarType damping = 0.3;
    ScalarType dtAccumulator{};
    bool isDeathSpiralling = false;
};
}
#endif
