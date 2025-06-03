#include "emitter.hpp"
#include <random>
#include "utils.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Emitter::Emitter(int x, int y, int rate, int particle_count, int particle_radius)
                    :x(x), y(y), rate(rate), particle_count(particle_count), particle_radius(particle_radius){}

void Emitter::start(float dt, std::vector<Particle>& particles){
    if (emitted >= particle_count) return;

    timeSinceLast += dt;
    float interval = 1.0f / static_cast<float>(rate);

    while(timeSinceLast >= interval && emitted < particle_count){
        
        float vx = randomFloat(-0.5f, 0.5f);
        float vy = randomFloat(0.0f, 2.0f);

        int radius = particle_radius;

        float density = 1.0f;
        int r, g, b;
        r=0;
        g=255;
        b=255;
        // hsvToRgb(hue, 1.0f, 1.0f, r, g, b);
        // // hue += 0.1;
        // hue += 30;
        // if (hue >= 360.0f) hue -= 360.0f;

        particles.emplace_back(x, y, vx, vy, density, radius, r, g, b);
        emitted++;
        timeSinceLast -= interval;
    }
}