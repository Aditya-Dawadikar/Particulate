#pragma once

class Particle{
    private:
        float x,y;
        float vx,vy;
        float mass;
        int r;
        int cr,cg,cb;
        float density;

        float fx,fy;
    
    public:
        Particle(float x, float y, float vx, float vy, float density, int r, int cr, int cg, int cb);

        void update(float dt);
        void applyGravity(float g, float dt);

        // Getters
        float getX() const;
        float getY() const;
        float getVX() const;
        float getVY() const;
        float getMass() const;
        int getRadius() const;

        int getR() const;
        int getG() const;
        int getB() const;

        // Setters
        void setVelocity(float vx, float vy);
        void setColor(int r_, int g_, int b_);
        void setPosition(float x, float y);

        // force
        void applyForce(float dfx, float dfy);
        void clearForce();

        // GPU compatibility
        void serialize(float* pos, float* vel, float* force, float* mass, int* radius, int idx) const;
        void deserialize(const float* pos, const float* vel, int* radius, int idx);

};