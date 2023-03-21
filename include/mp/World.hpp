#ifndef World_h_
#define World_h_

#include "utility/debug.hpp"
#include "Dynamics.hpp"
#include "Constraint.hpp"
#include "Container.hpp"
#include "EdgeHandlers.hpp"
#include "Vec.hpp"

namespace mp {
template<int Dim, typename ScalarType = double>
class World
{
    using VecType = Vec<Dim, ScalarType>;
    using Particle_t = Particle<Dim, ScalarType>;
    using Force_t = Force<Dim, ScalarType>;
    using Medium_t = Medium<Dim, ScalarType>;

public:
    World() : edgeHandler(new EdgeHandlerBase<Dim, ScalarType>()) {}
    World(EdgeHandlerBase<Dim, ScalarType> *edgeHandler) : edgeHandler(edgeHandler) {}
    ~World() { delete edgeHandler; }
    void setRandomSeed(float s) { randomSeed = s; }
    void addParticle(Particle_t *particle) { particles.push_back(particle); }
    void addForce(Force_t force) { forces.push_back(force); }
    void addConstraint(Constraint<Dim, ScalarType> *constraint) { constraints.push_back(constraint); }
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
                static int i = 0;    
                VecType newPosition =
                    particle->position +
                    (particle->linearVelocity * stepSize) +
                    particle->acceleration * (stepSize * stepSize * ScalarType{0.5});
                
                VecType force = std::accumulate(
                    forces.begin(), forces.end(), VecType{},
                    [& particle](VecType &a, const Force_t &force) { a += force(*particle); return a; });
                
                VecType newAcceleration = force * particle->inverseMass;

                VecType newVelocity = 
                    particle->linearVelocity +
                    (particle->acceleration + newAcceleration ) * (stepSize * ScalarType{0.5});
                
//                std::cout << "force " << force << "\n";
//                std::cout << "position " << particle->position << "\n";
//                std::cout << "linearVelocity " << particle->linearVelocity << "\n";
//                std::cout << "acceleration " << particle->acceleration << "\n";
//                std::cout << "newAcceleration " << newAcceleration << "\n";
//                std::cout << "newVelocity " << newVelocity << "\n";
//                std::cout << "newPosition " << newPosition << "\n";
//                std::cout << "i " << i << "\n";
//                if (std::isnan(force[0]))
//                    exit(99);
                particle->position = newPosition;
                particle->linearVelocity = newVelocity;
                particle->acceleration = newAcceleration;
                edgeHandler->handleEdge(particle);
                i++;
            }
            int iterationCount = 10;
            ScalarType iterationDt = stepSize / static_cast<ScalarType>(iterationCount);
            for (int i = 0; i < iterationCount; ++i)
                for (Constraint<Dim, ScalarType> *constraint : constraints)
                {
                    auto impulses = constraint->solve(iterationDt);
                     
                    impulses.first.particle.applyImpulse(impulses.first.deltaV);
                    impulses.second.particle.applyImpulse(impulses.second.deltaV);
                }


            dtAccumulator -= stepSize;
        }
        
        
        static ScalarType prevDt = 0.0;
        ScalarType diff = dt - prevDt;
        prevDt = dt;
        isDeathSpiralling = diff > stepSize;

    }     
    VecType gravity{};
    vector<Particle_t *> particles;
    vector<Force_t> forces;
    vector<Constraint<Dim, ScalarType> *> constraints;
    EdgeHandlerBase<Dim, ScalarType> *edgeHandler;
    float randomSeed = 0.0f;
    ScalarType stepSize = 0.01;
    ScalarType dtAccumulator{};
    bool isDeathSpiralling = false;
};
}
#endif
