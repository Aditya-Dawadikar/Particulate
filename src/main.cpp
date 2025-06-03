#include "renderer.hpp"
#include <chrono>
#include <thread>
#include "particle.hpp"
#include "simulation.hpp"
#include <SDL2/SDL.h>
#include "utils.hpp"
#include "emitter.hpp"

int main() {
    const int width = 420;
    const int height = 420;
    Renderer renderer(width, height);
    Simulation sim(width, height);

    std::vector<Particle> particles;

    // large particles
    Emitter emitter1(width/2, 10, 10, 5000,2);

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
            emitter1.start(dt, particles);
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
