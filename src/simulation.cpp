#include "simulation.hpp"
#include <cmath>

Simulation::Simulation(int windowWidth, int windowHeight, float gravity)
    : gravity(gravity), width(windowWidth), height(windowHeight) {}

void Simulation::step(std::vector<Particle>& particles, float dt) {

    // clear forces
    for(auto& p:particles){
        p.clearForce();
    }

    // Apply gravity
    for(auto& p:particles){
        p.applyForce(0.0f, p.getMass()*gravity);
    }

    // compute particle-particle forces
    handleCollisions(particles);

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

void Simulation::handleCollisions(std::vector<Particle>& particles){
    const float k = 10000.0f; // spring constant
    const float minDist = 1e-4f;
    float damping = 5.0f;
    const float restitution = 0.6f;

    for(size_t i=0;i<particles.size();++i){
        for(size_t j=i+1;j<particles.size();++j){
            if(i==j){continue;}
            Particle& a = particles[i];
            Particle& b = particles[j];

            float dx = b.getX() - a.getX();
            float dy = b.getY() - a.getY();
            float distsq = dx*dx + dy*dy;
            float rSum = a.getRadius() + b.getRadius();

            if(distsq < rSum*rSum){
                float dist = std::max(std::sqrt(distsq), minDist);
                float overlap = rSum - dist;

                // normalize direction vector (unit vector)
                float nx = dx / dist;
                float ny = dy / dist;

                // apply repulsive forces
                // float forceMag = k*overlap;

                float rvx = b.getVX() - a.getVX();
                float rvy = b.getVY() - a.getVY();

                // velocity along normal
                float velAlongNormal = rvx * nx + rvy * ny;

                if (velAlongNormal > 0){
                    continue;
                }

                float m1 = a.getMass();
                float m2 = b.getMass();

                // impulse scalar
                float impulse = -(1.0f + restitution) * velAlongNormal / (1.0f / m1 + 1.0f / m2);

                // Apply impulse to velocities
                float impulseX = impulse * nx;
                float impulseY = impulse * ny;

                a.setVelocity(a.getVX() - impulseX / m1, a.getVY() - impulseY / m1);
                b.setVelocity(b.getVX() + impulseX / m2, b.getVY() + impulseY / m2);

                // Optional: Positional correction to eliminate overlap
                float percent = 0.8f; // push 80% out to correct
                float slop = 0.01f;   // soft threshold
                float correctionMag = std::max(overlap - slop, 0.0f) / (m1 + m2) * percent;

                float correctionX = correctionMag * nx;
                float correctionY = correctionMag * ny;

                a.setPosition(a.getX() - correctionX * m2, a.getY() - correctionY * m2);
                b.setPosition(b.getX() + correctionX * m1, b.getY() + correctionY * m1);

                // // float damping = 5.0f; // tuned based on k
                // float forceMag = k * overlap - damping * relativeNormal;

                // float fx = forceMag *nx;
                // float fy = forceMag *ny;

                // a.applyForce(-fx,-fy);
                // b.applyForce(fx,fy);

                // // float correction = 0.5f * (overlap + 0.01f);
                // // a.setPosition(a.getX() - nx * correction, a.getY() - ny * correction);
                // // b.setPosition(b.getX() + nx * correction, b.getY() + ny * correction);
            }
        }
    }
}