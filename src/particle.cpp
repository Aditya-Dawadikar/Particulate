#include "particle.hpp"

Particle::Particle(float x, float y, float vx, float vy, float density, int r, int cr=255, int cg=255, int cb=255)
    : x(x), y(y), vx(vx), vy(vy), density(density), r(r), cr(cr), cg(cg), cb(cb) {
        mass = density * 3.14 * r * r;
    }

void Particle::update(float dt) {

    float ax = fx/mass;
    float ay = fy/mass;

    vx += ax*dt;
    vy += ay*dt;

    x += vx * dt;
    y += vy * dt;
}

void Particle::applyGravity(float g, float dt) {
    vy += g * dt;
}

// Getters
float Particle::getX() const { return x; }
float Particle::getY() const { return y; }
float Particle::getVX() const { return vx; }
float Particle::getVY() const { return vy; }
float Particle::getMass() const { return mass; }
int Particle::getRadius() const {return r;}
int Particle::getR() const { return cr; }
int Particle::getG() const { return cg; }
int Particle::getB() const { return cb; }


// Setters
void Particle::setVelocity(float vx_, float vy_) {
    vx = vx_;
    vy = vy_;
}

void Particle::setColor(int r_, int g_, int b_) {
    cr = r_;
    cg = g_;
    cb = b_;
}

void Particle::setPosition(float newX, float newY) {
    x = newX;
    y = newY;
}

void Particle::applyForce(float dfx, float dfy){
    fx += dfx;
    fy += dfy;
}

void Particle::clearForce(){
    this->fx = 0.0f;
    this->fy = 0.0f;
}