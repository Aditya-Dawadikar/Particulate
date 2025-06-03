#pragma once

#include <SDL2/SDL.h>
#include <vector>
#include "particle.hpp"

class Renderer{
    private:
        SDL_Window* window;
        SDL_Renderer* sdl_renderer;
        const int width;
        const int height;
    
    public:
        Renderer(int width = 420, int height = 420);
        ~Renderer();

        void clear();
        void draw(const std::vector<Particle>& particles);
        void present();
        bool shouldClose();
};