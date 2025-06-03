#include "renderer.hpp"
#include <SDL2/SDL2_gfxPrimitives.h>
#include <algorithm>
#include <numeric>

void drawFilledCircle(SDL_Renderer* renderer, int cx, int cy, int radius);
void drawSoftCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color baseColor);

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
    SDL_SetRenderDrawBlendMode(sdl_renderer, SDL_BLENDMODE_BLEND);
    for (const auto& p : particles) {
        SDL_SetRenderDrawColor(sdl_renderer, p.getR(), p.getG(), p.getB(), 255);
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

void drawSoftCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color baseColor) {
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            float distSq = dx * dx + dy * dy;
            if (distSq <= radius * radius) {
                float dist = std::sqrt(distSq);
                float alphaRatio = std::clamp(std::pow(1.0f - dist / radius, 2.0f), 0.0f, 1.0f);  // smoother falloff

                SDL_SetRenderDrawColor(renderer, 
                    baseColor.r, baseColor.g, baseColor.b,
                    static_cast<Uint8>(alphaRatio * 255));

                SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
            }
        }
    }
}
