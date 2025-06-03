#pragma once
#include <vector>
#include "particle.hpp"
#include <unordered_map>
#include <vector>
#include "utils.hpp"

class Simulation {
private:
    float gravity;
    int width, height;

    int cellSize = 40;
    std::unordered_map<std::pair<int, int>, std::vector<int>, pair_hash> grid;

public:
    Simulation(int windowWidth, int windowHeight, float gravity = 300.0f);

    void populate_grid(std::vector<Particle>& particles);

    void step(std::vector<Particle>& particles, float dt);
    void handleCollisions(int pass_idx, std::vector<Particle>& particles);
    void computeContactDetails(const Particle& a, const Particle& b,
                                       float& nx, float& ny, float& overlap);
    bool checkCollision(const Particle& a, const Particle& b);
    void resolveImpulse(Particle& a, Particle& b, float nx, float ny);
    void applyFriction(Particle& a, Particle& b, float nx, float ny);
    void positionalCorrection(int pass_idx, Particle& a, Particle& b, float nx, float ny, float overlap);

    void applyMouseForce(std::vector<Particle>& particles, int mx, int my, bool active);
};
