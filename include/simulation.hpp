#pragma once
#include <vector>
#include "particle.hpp"

class Simulation {
private:
    float gravity;
    int width, height;

public:
    Simulation(int windowWidth, int windowHeight, float gravity = 300.0f);

    void step(std::vector<Particle>& particles, float dt);
    void handleCollisions(std::vector<Particle>& particles);
};
