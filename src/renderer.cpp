#include "renderer.hpp"
#include <SDL2/SDL2_gfxPrimitives.h>

void drawFilledCircle(SDL_Renderer* renderer, int cx, int cy, int radius);

Renderer::Renderer(int width, int height) : width(width), height(height) {
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Particle Sim",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width, height, SDL_WINDOW_SHOWN);
    sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
}

Renderer::~Renderer() {
    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void Renderer::clear() {
    SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 255);
    SDL_RenderClear(sdl_renderer);
}

void Renderer::draw(const std::vector<Particle>& particles) {
    SDL_SetRenderDrawColor(sdl_renderer, 255, 255, 255, 255);
    for (const auto& p : particles) {
        SDL_SetRenderDrawColor(sdl_renderer, p.getR(), p.getG(), p.getB(), 255);
        // SDL_Rect rect {
        //     static_cast<int>(p.getX()),
        //     static_cast<int>(p.getY()),
        //     p.getRadius(),
        //     p.getRadius()
        // };
        // SDL_RenderFillRect(sdl_renderer, &rect);
        // drawFilledCircle(sdl_renderer, static_cast<int>(p.getX()), static_cast<int>(p.getY()), p.getRadius());
        filledCircleRGBA(sdl_renderer, p.getX(), p.getY(), p.getRadius(), p.getR(), p.getG(), p.getB(), 255);
    }
}

void Renderer::present() {
    SDL_RenderPresent(sdl_renderer);
}

bool Renderer::shouldClose() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) return true;
    }
    return false;
}

void drawFilledCircle(SDL_Renderer* renderer, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy <= radius * radius) {
                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}
