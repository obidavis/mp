#include <mp/Renderer.hpp>
#include <SDL2/SDL.h>

template <int Dim>
class MP_SDL_Renderer : public Renderer<Dim, double>
{
public:
    MP_SDL_Renderer(unsigned int height, unsigned int width)
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

protected:
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
