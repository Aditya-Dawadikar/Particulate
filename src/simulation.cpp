#include "simulation.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>


Simulation::Simulation(int windowWidth, int windowHeight, float gravity)
    : gravity(gravity), width(windowWidth), height(windowHeight) {}

void Simulation::populate_grid(std::vector<Particle>& particles){
    // clear grid
    grid.clear();

    // populate
    for(int i=0; i<particles.size();++i){
        int cx = static_cast<int>(particles[i].getX() / cellSize);
        int cy = static_cast<int>(particles[i].getY() / cellSize);
        grid[{cx, cy}].push_back(i);
    }
}

void Simulation::step(std::vector<Particle>& particles, float dt) {
    // Clear forces
    for(auto& p:particles){
        p.clearForce();
    }

    // Apply gravity
    for(auto& p:particles){
        p.applyForce(0.0f, p.getMass()*gravity);
    }

    // compute particle-particle forces
    int numPasses = 1;

    for (int iter = 0; iter < numPasses; ++iter) {
        handleCollisions(iter, particles);
    }

    // integrate
    for(auto& p:particles){
        p.update(dt);
    }

    for (auto& p : particles) {

        // Collision with walls
        float radius = p.getRadius();

        if (p.getX() - radius < 0) {
            p.setVelocity(-p.getVX() * 0.8f, p.getVY());
            p.setPosition(radius, p.getY());
        }
        if (p.getX() + radius > width) {
            p.setVelocity(-p.getVX() * 0.8f, p.getVY());
            p.setPosition(width - radius, p.getY());
        }
        if (p.getY() - radius < 0) {
            p.setVelocity(p.getVX(), -p.getVY() * 0.8f);
            p.setPosition(p.getX(), radius);
        }
        if (p.getY() + radius > height) {
            p.setVelocity(p.getVX(), -p.getVY() * 0.8f);
            p.setPosition(p.getX(), height - radius);
        }
    }
}

void Simulation::handleCollisions(int pass_idx, std::vector<Particle>& particles){

    bool reverse = pass_idx % 2 != 0;

    // Sort particle indices based on Y position (bottom to top or top to bottom)
    std::vector<int> particleIndices(particles.size());
    std::iota(particleIndices.begin(), particleIndices.end(), 0);

    std::sort(particleIndices.begin(), particleIndices.end(), [&](int i, int j) {
        return reverse ? particles[i].getY() > particles[j].getY()
                       : particles[i].getY() < particles[j].getY();
    });

    populate_grid(particles);

    for(int i : particleIndices){
        Particle &a = particles[i];

        int cx = static_cast<int>(a.getX() / cellSize);
        int cy = static_cast<int>(a.getY() / cellSize);

        // search neighbors on 3D grid
        for(int dx = -1; dx<=1;++dx){
            for(int dy = -1; dy<=1;++dy){
                std::pair<int,int> neighborCell = {cx+dx, cy+dy};
                if(grid.find(neighborCell) == grid.end()) continue;

                for (int j : grid[neighborCell]) {
                    if ((reverse && i <= j) || (!reverse && i >= j)) continue;

                    Particle& b = particles[j];

                    if (checkCollision(a, b)) {
                        float nx, ny, overlap;
                        computeContactDetails(a, b, nx, ny, overlap);

                        resolveImpulse(a, b, nx, ny);
                        // resolveImpulse2(a, b, nx, ny, overlap);
                        // applyFriction(a, b, nx, ny);
                        positionalCorrection(pass_idx, a, b, nx, ny, overlap);
                    }
                }
            }
        }
    }
}

void Simulation::computeContactDetails(const Particle& a, const Particle& b,
                                       float& nx, float& ny, float& overlap) {
    float dx = b.getX() - a.getX();
    float dy = b.getY() - a.getY();
    float dist = std::sqrt(dx * dx + dy * dy + 1e-4f);  // add epsilon
    overlap = (a.getRadius() + b.getRadius()) - dist;
    
    // std::cout << "Particle A: (" << a.getX() << ", " << a.getY() << ")  "
    //       << "Particle B: (" << b.getX() << ", " << b.getY() << ")  "
    //       << "Overlap: " << overlap << "\n";

    nx = dx / dist;
    ny = dy / dist;
}


bool Simulation::checkCollision(const Particle& a, const Particle& b) {
    float dx = b.getX() - a.getX();
    float dy = b.getY() - a.getY();
    float rSum = a.getRadius() + b.getRadius();
    return (dx * dx + dy * dy) < (rSum * rSum);
}


void Simulation::resolveImpulse(Particle& a, Particle& b, float nx, float ny) {
    float rvx = b.getVX() - a.getVX();
    float rvy = b.getVY() - a.getVY();

    float velAlongNormal = rvx * nx + rvy * ny;
    if (velAlongNormal > 0) return;

    float e = 0.6f;  // restitution
    float m1 = a.getMass();
    float m2 = b.getMass();

    float impulse = -(1.0f + e) * velAlongNormal / (1.0f / m1 + 1.0f / m2);

    float impulseX = impulse * nx;
    float impulseY = impulse * ny;

    const float velocityThreshold = 0.05f;

    float newVxA = a.getVX() - impulseX / m1;
    float newVyA = a.getVY() - impulseY / m1;
    float newVxB = b.getVX() + impulseX / m2;
    float newVyB = b.getVY() + impulseY / m2;

    if (std::abs(newVxA - a.getVX()) > velocityThreshold || std::abs(newVyA - a.getVY()) > velocityThreshold)
        a.setVelocity(newVxA, newVyA);

    if (std::abs(newVxB - b.getVX()) > velocityThreshold || std::abs(newVyB - b.getVY()) > velocityThreshold)
        b.setVelocity(newVxB, newVyB);

}

void Simulation::applyFriction(Particle& a, Particle& b, float nx, float ny) {
    float rvx = b.getVX() - a.getVX();
    float rvy = b.getVY() - a.getVY();

    float tx = -ny;
    float ty = nx;

    float velAlongTangent = rvx * tx + rvy * ty;
    float m1 = a.getMass();
    float m2 = b.getMass();

    float jt = -velAlongTangent / (1.0f / m1 + 1.0f / m2);

    // coefficient of friction
    // float mu = 0.9f;
    // float normalImpulse = std::abs(rvx * nx + rvy * ny);  // estimate

    float baseMu = 0.2f;
    float normalImpulse = std::abs(rvx * nx + rvy * ny);
    float weightFactor = std::abs((a.getMass() + b.getMass()) * gravity * ny);
    float mu = baseMu + 0.002f * weightFactor; // scaled contribution

    jt = std::clamp(jt, -mu * normalImpulse, mu * normalImpulse);

    float frictionX = jt * tx;
    float frictionY = jt * ty;

    a.setVelocity(a.getVX() - frictionX / m1, a.getVY() - frictionY / m1);
    b.setVelocity(b.getVX() + frictionX / m2, b.getVY() + frictionY / m2);
}

void Simulation::positionalCorrection(int pass_idx, Particle& a, Particle& b, float nx, float ny, float overlap) {
    float slop = 0.01f;
    float effectiveOverlap = std::max(overlap - slop, 0.0f);

    // Normalize correction vector (already done via nx, ny)
    float m1 = a.getMass();
    float m2 = b.getMass();
    float totalMass = m1 + m2;

    float moveA = (m2 / totalMass) * effectiveOverlap;
    float moveB = (m1 / totalMass) * effectiveOverlap;

    // Final movement along normal vector
    a.setPosition(a.getX() - nx * moveA, a.getY() - ny * moveA);
    b.setPosition(b.getX() + nx * moveB, b.getY() + ny * moveB);
}
