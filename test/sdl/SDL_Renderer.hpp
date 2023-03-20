#include "../../include/mp/Renderer.hpp"
#include "../../include/mp/Surface.hpp"
#include "../../include/mp/Shader.hpp"
#include <SDL2/SDL.h>

class MP_SDL_Renderer : public Renderer<3, double>
{
public:
    MP_SDL_Renderer(unsigned int height, unsigned int width, Vec_t inputMin, Vec_t inputMax)
        : Renderer<3, double>(inputMin, inputMax, {0.0, 0.0, -127.0}, {static_cast<double>(width), static_cast<double>(height), 127.0})
    {
        SDL_Init(SDL_INIT_VIDEO);       // Initializing SDL as Video
        SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);      // setting draw color
        SDL_RenderClear(renderer);      // Clear the newly created window
        SDL_RenderPresent(renderer);    // Reflects the changes done in the
    }

    ~MP_SDL_Renderer()
    {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    void show(void) override { SDL_RenderPresent(renderer); }
    void clear(void) override 
    {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);
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
//
//        drawLine(mapped_v1.x(), mapped_v1.y(), mapped_v2.x(), mapped_v2.y(), surfaceColour);
//        drawLine(mapped_v1.x(), mapped_v1.y(), mapped_v3.x(), mapped_v3.y(), surfaceColour);
//        drawLine(mapped_v2.x(), mapped_v2.y(), mapped_v3.x(), mapped_v3.y(), surfaceColour);
    }
private:
    void drawCircle(int center_x, int center_y, int radius, RGBA<uint8_t> colour)
    {
        SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);

        // Drawing circle
        for(int x=center_x-radius; x<=center_x+radius; x++){
            for(int y=center_y-radius; y<=center_y+radius; y++){
                if((std::pow(center_y-y,2)+std::pow(center_x-x,2)) <= 
                    std::pow(radius,2)){
                    SDL_RenderDrawPoint(renderer, x, y);
                }
            }
        }
    }
        
    void drawLine(int x1, int y1, int x2, int y2, RGBA<uint8_t> colour)
    {
        SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    }
    

    SDL_Renderer *renderer = nullptr;
    SDL_Window *window = nullptr;
};
