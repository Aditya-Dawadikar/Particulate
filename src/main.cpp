#include "renderer.hpp"
#include <chrono>
#include <thread>
#include "particle.hpp"
#include "simulation.hpp"
#include <SDL2/SDL.h>
#include "utils.hpp"

int main() {
    const int width = 512;
    const int height = 512;
    Renderer renderer(width, height);
    Simulation sim(width, height);

    std::vector<Particle> particles;
    // Spawn 100 random particles
    for (int i = 0; i < 500; ++i) {
        float x = randomFloat(10.0f, width - 10.0f);
        float y = randomFloat(10.0f, height - 10.0f);
        float vx = randomFloat(-50.0f, 50.0f);
        float vy = randomFloat(-50.0f, 50.0f);
        // float mass = randomFloat(0.5f, 2.0f);
        float density = 1;
        int radius = static_cast<int>(randomFloat(2.0f, 10.0f));
        // int radius = 10;
        // int r = static_cast<int>(randomFloat(50, 255));
        // int g = static_cast<int>(randomFloat(50, 255));
        // int b = static_cast<int>(randomFloat(50, 255));

        if (i%2==0){
            particles.emplace_back(x, y, vx, vy, density, radius, 0, 255, 0);
        }else{
            particles.emplace_back(x, y, vx, vy, density, radius, 255, 0, 0);
        }
    }

    bool paused = false;
    bool quit = false;

    auto last = std::chrono::high_resolution_clock::now();

    while (!quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT){
                quit = true;
            }else if(event.type == SDL_KEYDOWN){
                if (event.key.keysym.sym == SDLK_SPACE) {
                    paused = !paused;
                }
            }
        }

        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = now - last;
        last = now;

        float dt = elapsed.count();

        if (!paused){
            sim.step(particles, dt);
        }

        renderer.clear();
        renderer.draw(particles);
        renderer.present();

        // Limit FPS to ~60
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
