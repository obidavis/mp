#pragma once

#include "Dynamics.hpp"
#include <utility>

template <int Dim, typename T>
class Constraint
{
public:

    Constraint(Particle<Dim, T> &p1, Particle<Dim, T> &p2) : p1(p1), p2(p2) {}
    virtual void solve(T dt) = 0;
    Particle<Dim, T> &p1;
    Particle<Dim, T> &p2;
};



template <int Dim, typename T>
class DistanceConstraint : public Constraint<Dim, T>
{
public:
    DistanceConstraint(Particle<Dim, T> &p1, Particle<Dim, T> &p2) 
        : Constraint<Dim, T>(p1, p2), length((p1.position - p2.position).length()) {}
    void solve(T dt) override
    {
        T constraintMass = this->p1.inverseMass + this->p2.inverseMass;
        if (constraintMass <= 0)
            return;

        Vec<Dim, T> relativePosition = this->p1.position - this->p2.position;
        T distance = relativePosition.length();
        T offset = length - distance;
        offset *= strength;
        Vec<Dim, T> offsetDir = relativePosition.normalised();
        Vec<Dim, T> relativeVelocity = this->p1.linearVelocity - this->p2.linearVelocity;
        T velocityDot = Vec<Dim, T>::dot(relativeVelocity, offsetDir);
        T bias = -(biasFactor / dt) * offset;
        T lambda = -(velocityDot + bias) / constraintMass;
        
        this->p1.applyImpulse(offsetDir * lambda);
        this->p2.applyImpulse(-offsetDir * lambda);
    }
private:
    T length;
    T strength = 0.2;
    T biasFactor = 0.3;
};
