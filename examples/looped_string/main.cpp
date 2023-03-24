#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <mp/World.hpp>
#include <mp/utility/maths.hpp>
#include <mp/dynamics/spring_force.hpp>
#include <mp/rendering/shape.hpp>
#include <etl/vector.h>

using namespace mp;

#include "renderer.hpp"
#include "logistic_medium.hpp"

Adafruit_NeoPixel leds(120, 17, NEO_GRB + NEO_KHZ800);
Renderer renderer(120, 0.0f, 1.0f, leds);

using Vec_t = Vec<2, float>;
using Particle_t = Particle<2, float>;
using Constraint_t = Constraint<2, float>;


inline Vec_t wrappedDistance(const Vec_t &lhs, const Vec_t &rhs)
{
    static mp::wrapped_distance<float> wrap(1.0);
    Vec_t diff = lhs - rhs;
    diff.x() = wrap(diff.x());
    return diff;
}

class WrappedDistanceConstraint : public Constraint<2, float>
{
    using Particle_t = Particle<2, float>;
    using Vec_t = Vec<2, float>;
public:
    WrappedDistanceConstraint(Particle_t &p1, Particle_t &p2) 
        : Constraint<2, float>(p1, p2), wrap(1.0), length((wrappedDistance(this->p1.position, this->p2.position)).length()) 
    {
    }
    void solve(float dt) override
    {
        float constraintMass = this->p1.inverseMass + this->p2.inverseMass;
        if (constraintMass <= 0)
            return;

        Vec<2, float> relativePosition = wrappedDistance(this->p1.position, this->p2.position);
        float distance = relativePosition.length();
        float offset = length - distance;
        offset *= strength;
        Vec<2, float> offsetDir = relativePosition.normalised();
        Vec<2, float> relativeVelocity = this->p1.linearVelocity - this->p2.linearVelocity;
        float velocityDot = Vec<2, float>::dot(relativeVelocity, offsetDir);
        float bias = -(biasFactor / dt) * offset;
        float lambda = -(velocityDot + bias) / constraintMass;
        
        this->p1.applyImpulse(offsetDir * lambda);
        this->p2.applyImpulse(-offsetDir * lambda);
    }
    mp::wrapped_distance<float> wrap;
    bool log = false;
    float length;
    float strength = 1.0;
    float biasFactor = 0.6;
};

void handleEdge(Particle_t &particle)
{
    const Vec_t min = {0.0, -0.01};
    const Vec_t max = {1.0, 1000000.0};

    if (particle.position.x() > max.x())
        particle.position.x() = min.x();
    if (particle.position.x() < min.x())
        particle.position.x() = max.x();
}

constexpr int nParticles = 30;
etl::vector<Particle_t, nParticles> particles;
etl::vector<WrappedDistanceConstraint, nParticles> constraints;

etl::vector<std::reference_wrapper<Constraint_t>, nParticles> constraint_refs;
mp::World<2, float> world;
LogisticMedium medium(-1.0, 1.0, 2.0, 0.0, 1.0, world.gravity); 

Vec_t applyForces(Particle_t &particle)
{
    return medium.calculateForce(particle);
}

void setup()
{
    Serial.begin(9600);

    leds.begin();
    leds.setBrightness(255);
    for (int i = 0; i < leds.numPixels(); ++i)
    {
        leds.setPixelColor(i, 0, 30, 30);
        leds.show();
        delay(10);
    }
    for (int i = 0; i < leds.numPixels(); ++i)
    {
        leds.setPixelColor(i, 0, 0, 0);
        leds.show();
        delay(10);
    }
    world.setPositionCB(handleEdge);
    for (int i = 0; i < nParticles; ++i)
    {
        Particle_t p;
        p.position.x() = i / (static_cast<float>(nParticles));
        particles.push_back(p);
    }
    
    for (size_t i = 0; i < particles.size(); ++i)
    {
        WrappedDistanceConstraint d(particles[i], particles[(i + 1) % particles.size()]);
        constraints.push_back(d);
    }
    
    world.addParticles({particles});
    
    for (Constraint_t &c : constraints)
        constraint_refs.push_back(std::ref(c));
    world.addConstraints({constraint_refs});
    world.setForceCB(applyForces);
    world.setDamping(0.3);
    world.gravity = {0.0, -9.5};
    world.timeStretch = 1.0f;
    world.iterationCount = 2;
    world.stepSize = 0.045f;
}

void loop()
{
    static elapsedMicros physTimer = 0;
    float step = physTimer / 1'000'000.0f;
    physTimer = 0;
    world.step(step);   
    
   static elapsedMillis renderMs;
   if (renderMs > 20)
   {
       for (Constraint_t &spring : constraint_refs)
       {
           mp::Line<2, float> line(spring.p1.position, spring.p2.position);
           renderer.drawShape(line);
       }
       renderer.show();
       renderMs = 0;
   }
   if (world.isDeathSpiralling)
   {
       float stepSize = world.stepSize + 0.01;
       Serial.print(world.dtAccumulator);
       Serial.print("\t");
       Serial.print(world.stepSize);
        Serial.print("\tbad, changing stepsize to ");
        Serial.println(stepSize);
        world.stepSize = stepSize;
   }

   static elapsedMillis reportms;
   if (reportms > 1000)
    {
        Serial.println("ok");
        reportms = 0;
    }

   static elapsedMillis impulsems = 5000;
   if (impulsems > 7500)
   {
       int index = random() % particles.size();
       particles[index].applyImpulse({0.0f, 0.75f});
       impulsems = random() % 5000;
    }
}

