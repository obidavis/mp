#include "../../include/mp/Surface.hpp"
#include "../../include/mp/Shader.hpp"
#include "../../include/mp/utility/maths.hpp"

#include "SDL_Renderer.hpp"

namespace mp {

class NurbsRenderer : public MP_SDL_Renderer<3>
{
public:
    NurbsRenderer(unsigned int height, unsigned int width, Vec_t inputMin, Vec_t inputMax)
        : MP_SDL_Renderer<3>(height, width), mapPosition(inputMin, inputMax, {0, 0, -5}, {width, height, 5})
    {
    }

    void drawConstraint(const Constraint_t &constraint) override
    {
        RGBA<uint8_t> colour{50, 50, 50, 255};
        const Vec_t position1 = mapPosition(constraint.p1.position);
        const Vec_t position2 = mapPosition(constraint.p2.position);
        drawLine(position1.x(), position1.y(), position2.x(), position2.y(), colour);
    }

    void drawParticle(const Particle_t &particle) override
    {
        RGBA<uint8_t> colour = {127, 0, 0, 255};
        const Vec_t mappedPosition = mapPosition(particle.position);
        colour.r += mappedPosition.z();
        colour.g += mappedPosition.z();
        colour.b += mappedPosition.z();
        drawCircle(mappedPosition.x(), mappedPosition.y(), 3, colour);
    }
    

    void drawPolygon(const Surface<double> surface)
    {
        const Vec_t mapped_v1 = mapPosition(surface.v1);
        const Vec_t mapped_v2 = mapPosition(surface.v2);
        const Vec_t mapped_v3 = mapPosition(surface.v3);
        PointLight<3, double> light{
            .colour = {1.0, 1.0, 1.0},
            .ambientIntensity = 0.1,
            .diffuseIntensity = 1.0,
            .position = {0.0, 30.0, 10.0},
        };
        
        const Vec_t eyePos = {0.0, 0.0, 10.0};
        const Vec_t lightDirection = (light.position - surface.centre()).normalised();
        Vec<4, double> colour = _calculateLight(light, lightDirection, surface.normal(), eyePos, surface.centre());
        
        const RGBA<uint8_t> c = {(uint8_t)(colour[0] * 255), (uint8_t)(colour[1] * 255), (uint8_t)(colour[2] * 255), (uint8_t)(colour[3] * 255)};   

        SDL_Vertex v1 =  {{(float)mapped_v1.x(), (float)mapped_v1.y()}, {c.r, c.g, c.b, 255}, {1, 1}};
        SDL_Vertex v2 =  {{(float)mapped_v2.x(), (float)mapped_v2.y()}, {c.r, c.g, c.b, 255}, {1, 1}};
        SDL_Vertex v3 =  {{(float)mapped_v3.x(), (float)mapped_v3.y()}, {c.r, c.g, c.b, 255}, {1, 1}};
        SDL_Vertex vertices[] = {v1, v2, v3};
        SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
    }
protected:
    map_linear<Vec_t> mapPosition;
};

}
