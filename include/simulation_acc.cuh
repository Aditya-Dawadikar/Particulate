#pragma once

#include <vector>
#include "particle.hpp"

class SimulationAcc{
    public:
        SimulationAcc(int width, int height, float gravity, int cellSize);
    
        void step(std::vector<Particle>& particle, float dt);

        ~SimulationAcc();
    
    private:
        int width, height;
        float gravity;
        int cellSize;

        // CUDA buffers
        float* d_positions;
        float* d_velocities;
        float* d_forces;
        float* d_masses;
        int* d_radii;
        int num_particles;

        // internal methods
        void uploadParticles(const std::vector<Particle>& particles);
        void downloadParticles(std::vector<Particle>& particles);
        void allocateDeviceMemory(int count);
        void freeDeviceMemory();
        void computePhysicsGPU(float dt);
};