#ifndef EdgeHandlers_h_
#define EdgeHandlers_h_

#include "Dynamics.hpp"
namespace mp {

template <int Dim, typename ScalarType> struct EdgeHandlerBase {
  virtual void handleEdge(Particle<Dim, ScalarType> *particle) {};
  virtual ~EdgeHandlerBase() {}
};

template <int Dim, typename ScalarType> class InfiniteBounds {
  void handleEdge(Particle<Dim, ScalarType> *) override {}
};

template <int Dim, typename ScalarType>
class Pong : public EdgeHandlerBase<Dim, ScalarType> {
public:
  Pong(Vec<Dim, ScalarType> min, Vec<Dim, ScalarType> max,
       ScalarType restitution)
      : min(min), max(max), restitution(-restitution) {}
  void handleEdge(Particle<Dim, ScalarType> *particle) override {
    for (int i = 0; i < Dim; ++i) {
      if (particle->position[i] > max[i]) {
        particle->position[i] = max[i];
        particle->linearVelocity[i] = restitution * particle->linearVelocity[i];
      }
      if (particle->position[i] < min[i]) {
        particle->position[i] = min[i];
        particle->linearVelocity[i] = restitution * particle->linearVelocity[i];
      }
    }
  }

private:
  Vec<Dim, ScalarType> min;
  Vec<Dim, ScalarType> max;
  ScalarType restitution;
};

template <int Dim, typename ScalarType>
class Asteroids : public EdgeHandlerBase<Dim, ScalarType> {
public:
  Asteroids(Vec<Dim, ScalarType> min, Vec<Dim, ScalarType> max)
      : min{min}, max{max} {}
  void handleEdge(Particle<Dim, ScalarType> *particle) override {
    for (int i = 0; i < Dim; ++i) {
      if (particle->position[i] > max[i])
        particle->position[i] = min[i];
      if (particle->position[i] < min[i])
        particle->position[i] = max[i];
    }
  }

private:
  Vec<Dim, ScalarType> min;
  Vec<Dim, ScalarType> max;
};

}; // namespace mp
#endif
