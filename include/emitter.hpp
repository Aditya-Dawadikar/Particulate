#pragma once
#include "particle.hpp"
#include <vector>

class Emitter{
    private:
        int x;
        int y;
        int rate;
        int particle_count;
        float timeSinceLast;
        int emitted = 0;
        
        float hue = 300.0f;
        int particle_radius;
    
    public:
        Emitter(int x, int y, int rate, int particle_count, int particle_radius);

        void start(float dt, std::vector<Particle>& particles);
        // void stop();

        ~Emitter(){};
};