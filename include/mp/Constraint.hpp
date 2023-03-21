#pragma once

#include "Dynamics.hpp"
#include <mp/utility/maths.hpp>
#include <utility>

template <int Dim, typename T>
struct Impulse
{
    Particle <Dim, T> &particle;
    Vec<Dim, T> deltaV;
};
template <int Dim, typename T>
class Constraint
{
public:

    Constraint(Particle<Dim, T> &p1, Particle<Dim, T> &p2) : p1(p1), p2(p2) {}
    virtual std::pair<Impulse<Dim, T>, Impulse<Dim, T>> solve(T dt) = 0;
    Particle<Dim, T> &p1;
    Particle<Dim, T> &p2;
};


template <int Dim, typename T>
class DistanceConstraint : public Constraint<Dim, T>
{
public:
    DistanceConstraint(Particle<Dim, T> &p1, Particle<Dim, T> &p2) 
        : Constraint<Dim, T>(p1, p2), length((p1.position - p2.position).length()) {}
    std::pair<Impulse<Dim, T>, Impulse<Dim, T>> solve(T dt) override
    {
        Impulse<Dim, T> impulse1 = {this->p1, {}};
        Impulse<Dim, T> impulse2 = {this->p2, {}};

        T constraintMass = this->p1.inverseMass + this->p2.inverseMass;
        if (constraintMass <= 0)
            return {impulse1, impulse1};

        Vec<Dim, T> relativePosition = this->p1.position - this->p2.position;
        T distance = relativePosition.length();
        T offset = length - distance;
        offset *= strength;
        Vec<Dim, T> offsetDir = relativePosition.normalised();
        Vec<Dim, T> relativeVelocity = this->p1.linearVelocity - this->p2.linearVelocity;
        T velocityDot = Vec<Dim, T>::dot(relativeVelocity, offsetDir);
        T bias = -(biasFactor / dt) * offset;
        T lambda = -(velocityDot + bias) / constraintMass;
        impulse1.deltaV = offsetDir * lambda;
        impulse2.deltaV = -1 * offsetDir * lambda;
        
        return {impulse1, impulse2};
    }
    T length;
    T strength = 0.2;
    T biasFactor = 0.3;
};
