#include <Arduino.h>
#define MP_USE_ETL
#include <MiniPhysics/World.hpp>


using Vec3 = Vec<3, float>;
using Particle3 = Particle<3, float>;

class Damping
{
public:
    Damping(float d) : d(d) {}
    Vec3 operator()(const Particle3 &particle) 
    {
        return particle.linearVelocity * -d; 
    }

private:
    float d;
};

Vec3 Gravity(const Particle3 &particle) { return Vec3{0, -13.0, 0} * particle.mass; }

etl::vector<etl::vector<Particle3, 64>, 64> makeGrid(Vec3 min, Vec3 max, Vec<3, float> dim)
{
    Vec<3, float> inc = (max - min) / (dim - 1);
 etl::vector<etl::vector<Particle3, 64>, 64> vec;
    for (int y = 0; y < static_cast<int>(dim.y()); ++y)
    {
        etl::vector<Particle3, 64> row;
        for (int x = 0; x < static_cast<int>(dim.x()); ++x)
        {
            Particle3 particle(1.0, 1.0);
            particle.position.x() = min.x() + x * inc.x();
            particle.position.y() = min.y() + y * inc.y();
            row.push_back(particle);
        }
        vec.push_back(row);
    }
    return vec;
}
etl::vector<etl::vector<Particle3, 64>, 64> particleRows;
mp::EdgeHandlerBase<3, float> edgeHandler;
mp::World<3, float> world(&edgeHandler);

Damping damping(0.4);
void setup()
{
    Serial.begin(9600);
    while (!Serial && millis() < 5000);
    if (CrashReport) 
    {
        delay(1000);
        Serial.print(CrashReport);
        delay(1000);
    }
    Vec3 min{-70.0, -20.0, -5.0};
    Vec3 max{70.0, 20.0, 5.0};
    particleRows = makeGrid(min, max, {15.0, 5.0, 0.0});
    

    for (auto &p : particleRows[particleRows.size() - 1])
        p.inverseMass = 0.0;
    for (auto &row : particleRows)
    {
        row[0].inverseMass = 0.0;
        row[row.size() - 1].inverseMass = 0.0;
    }
    etl::vector<DistanceConstraint<3, float>, 128> joins;
    for (int r = 1; r < particleRows.size(); ++r)
    {
        for (int c = 0; c < particleRows[r].size(); ++c)
        {
            joins.emplace_back(&(particleRows[r][c]), &(particleRows[r - 1][c]));
            if (c > 0)
                joins.emplace_back(&(particleRows[r][c]), &(particleRows[r][c - 1]));
        }
    }


    for (auto &row : particleRows)
        for (Particle3 &particle : row)
            world.addParticle(&particle);
    for (DistanceConstraint<3, float> &join : joins)
    {
        world.addConstraint(&join);
    }
    
    world.addForce(Gravity);
    world.addForce(std::ref(damping));

}
        
void loop()
{
    static elapsedMillis timer = 0;
    world.step((double)timer / 1'000.0);

    static elapsedMillis outputMs = 0;
    if (outputMs > 10)
    {
        for (auto &row : particleRows)
            for (auto &p : row)
            {
                Serial.print(p.position.x());
                Serial.print(",");
                Serial.print(p.position.y());
                Serial.print(",");
                Serial.print(p.position.z());
                Serial.print(",");
            }
        Serial.println();
        outputMs = 0;
    }

    static elapsedMillis impulseMs = 0;
    if (impulseMs > 15000)
    {
        int index = random(0, 15);
        particleRows[2][index].applyImpulse({0.0f, 0.0f, 200.0f});
        impulseMs = random(0, 2000);
    }
    timer = 0;
}




