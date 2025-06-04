#include "simulation_acc.cuh"
#include <cuda_runtime.h>
#include <iostream>
#include "globals.hpp"

__global__ void applyGravityKernel(float* forces, float* masses, float gravity, int count);
__global__ void resolveImpulseKernel(float* forces, float* positions, float* velocities, float* masses, int* radii, int count, float dt);
__global__ void applyFrictionKernel(float* velocities, float* positions, int count);
__global__ void integrateKernel(float* positions, float* velocities, float* forces, float* masses, float dt, int count);
__global__ void wallCollisionKernel(int height, int width, float* positions, float* velocities, float* forces, float* masses, int* radii, float dt, int count);
__global__ void clearForcesKernel(float* forces, int count);

SimulationAcc::SimulationAcc(int width, int height, int cellSize)
    : width(width), height(height), gravity(980.0f), cellSize(cellSize),
        d_positions(nullptr), d_velocities(nullptr), d_forces(nullptr),
        d_masses(nullptr), d_radii(nullptr), num_particles(0){}

SimulationAcc::~SimulationAcc() {
    freeDeviceMemory();
}

void SimulationAcc::allocateDeviceMemory(int count){
    num_particles = count;

    cudaMalloc(&d_positions, 2*sizeof(float)*count);
    cudaMalloc(&d_velocities, 2*sizeof(float)*count);
    cudaMalloc(&d_forces, 2*sizeof(float)*count);
    cudaMalloc(&d_masses, 2*sizeof(float)*count);
    cudaMalloc(&d_radii, 2*sizeof(int)*count);
}

void SimulationAcc::freeDeviceMemory() {
    cudaFree(d_positions);
    cudaFree(d_velocities);
    cudaFree(d_forces);
    cudaFree(d_masses);
    cudaFree(d_radii);

    d_positions = d_velocities = d_forces = d_masses = nullptr;
    d_radii = nullptr;
    num_particles = 0;
}

void SimulationAcc::uploadParticles(const std::vector<Particle>& particles){
    int n = particles.size();

    if(n!=num_particles){
        freeDeviceMemory();
        allocateDeviceMemory(n);
    }

    std::vector<float> h_positions(2*n);
    std::vector<float> h_velocities(2*n);
    std::vector<float> h_forces(2*n);
    std::vector<float> h_masses(n);
    std::vector<int> h_radii(n);

    for(int i=0; i< n; ++i){
        h_positions[2*i] = particles[i].getX();
        h_positions[2*i+1] = particles[i].getY();

        h_velocities[2*i] = particles[i].getVX();
        h_velocities[2*i+1] = particles[i].getVY();

        h_masses[i] = particles[i].getMass();
        h_radii[i] = particles[i].getRadius();
    }

    cudaMemcpy(d_positions, h_positions.data(), sizeof(float)*2*n, cudaMemcpyHostToDevice);
    cudaMemcpy(d_velocities, h_velocities.data(), sizeof(float)*2*n, cudaMemcpyHostToDevice);
    cudaMemcpy(d_forces, h_forces.data(), sizeof(float)*2*n, cudaMemcpyHostToDevice);
    cudaMemcpy(d_masses, h_masses.data(), sizeof(float)*n, cudaMemcpyHostToDevice);
    cudaMemcpy(d_radii, h_radii.data(), sizeof(int)*n, cudaMemcpyHostToDevice);

}

void SimulationAcc::downloadParticles(std::vector<Particle>& particles){
    int n = particles.size();

    std::vector<float> h_positions(2*n);
    std::vector<float> h_velocities(2*n);

    cudaMemcpy(h_positions.data(), d_positions, sizeof(float)*2*n, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_velocities.data(), d_velocities, sizeof(float)*2*n, cudaMemcpyDeviceToHost);

    for(int i=0;i<n;++i){
        particles[i].setPosition(h_positions[2*i], h_positions[2*i+1]);
        particles[i].setVelocity(h_velocities[2*i], h_velocities[2*i+1]);
    }
}

void SimulationAcc::step(std::vector<Particle>& particles, float dt) {
    
    if (particles.empty()) return;

    memoryTransferInProgress = true;

    // 1. Upload data to GPU
    std::cout<<"Uploading Particles...\n";
    uploadParticles(particles);


    // 2. Compute physics using CUDA kernel(s)
    std::cout<<"Doing the Math...\n";
    // computePhysicsGPU(dt);
    int num_substeps = 4;
    float substep_dt = dt / num_substeps;

    // Run simulation in substeps
    for (int i = 0; i < num_substeps; ++i) {
        computePhysicsGPU(substep_dt);
    }


    // 3. Download updated data back to CPU
    downloadParticles(particles);

    
    memoryTransferInProgress = false;
}

void SimulationAcc::computePhysicsGPU(float dt) {
    int threadsPerBlock = 256;
    int blocks = (num_particles + threadsPerBlock - 1) / threadsPerBlock;

    clearForcesKernel<<<blocks, threadsPerBlock>>>(d_forces, num_particles);

    // 1. Apply gravity
    applyGravityKernel<<<blocks, threadsPerBlock>>>(d_forces, d_masses, gravity, num_particles);

    // 2. Resolve impulse (naive n^2 for now, will optimize later)
    resolveImpulseKernel<<<blocks, threadsPerBlock>>>(d_forces, d_positions, d_velocities, d_masses, d_radii, num_particles, dt);

    // 3. Apply friction
    // applyFrictionKernel<<<blocks, threadsPerBlock>>>(d_velocities, d_positions, num_particles);

    // 4. Integrate motion
    integrateKernel<<<blocks, threadsPerBlock>>>(
        d_positions, d_velocities, d_forces, d_masses, dt, num_particles
    );

    wallCollisionKernel<<<blocks, threadsPerBlock>>>(height, width, d_positions, d_velocities, d_forces, d_masses, d_radii, dt, num_particles);

    cudaDeviceSynchronize(); // Optional: For error debugging
}

__global__ void applyGravityKernel(float* forces, float* masses, float gravity, int count){
    int idx = blockIdx.x* blockDim.x + threadIdx.x;

    if(idx >= count) return;

    forces[2*idx] += 0.0f;
    forces[2*idx+1] += masses[idx]*gravity;
}

__global__ void resolveImpulseKernel(float* forces, float* positions, float* velocities, float* masses, int* radii, int count, float dt){
    int i = blockIdx.x* blockDim.x + threadIdx.x;
    if (i>=count) return;

    float xi = positions[2*i];
    float yi = positions[2*i+1];
    float vxi = velocities[2*i];
    float vyi = velocities[2*i+1];
    float mi = masses[i];
    int ri = radii[i];

    for(int j=0; j<count;++j){
        if(i==j) continue;

        float xj = positions[2*j];
        float yj = positions[2*j+1];
        float vxj = velocities[2*j];
        float vyj = velocities[2*j+1];
        float mj = masses[j];
        int rj = radii[j];

        float dx = xj - xi;
        float dy = yj - yi;
        float dist2 = dx*dx + dy*dy;
        float minDist = ri + rj;

        if(dist2 < minDist*minDist && dist2 > 0.0001f){
            float dist = sqrtf(dist2);
            float nx = dx / dist;
            float ny = dy / dist;

            // Relative velocity
            float rvx = vxj - vxi;
            float rvy = vyj - vyi;

            // Velocity along normal
            float velAlongNormal = rvx * nx + rvy * ny;

            // Only resolve if particles are moving toward each other
            if (velAlongNormal > 0) continue;

            // Coefficient of restitution (0 = perfectly inelastic, 1 = perfectly elastic)
            float e = 0.2f;  // Adjust as needed

            // Calculate impulse scalar
            float impulse = -(1.0f + e) * velAlongNormal;
            impulse /= (1.0f/mi + 1.0f/mj);

            // Apply impulse to both particles (equal and opposite)
            float impulseX = impulse * nx;
            float impulseY = impulse * ny;

            // Update velocities (atomic operations for thread safety)
            atomicAdd(&velocities[2*i], -impulseX / mi);
            atomicAdd(&velocities[2*i+1], -impulseY / mi);
            atomicAdd(&velocities[2*j], impulseX / mj);
            atomicAdd(&velocities[2*j+1], impulseY / mj);

            // Positional correction to prevent sinking
            float percent = 0.2f; // Usually 20% to 80%
            float slop = 0.01f;   // Usually 0.01 to 0.1
            float penetration = minDist - dist;
            penetration=(penetration, 2.0f);
            float correction = fmaxf(penetration - slop, 0.0f) / (1.0f/mi + 1.0f/mj) * percent;

            float correctionX = correction * nx;
            float correctionY = correction * ny;

            // Apply positional correction (atomic operations for thread safety)
            atomicAdd(&positions[2*i], -correctionX / mi);
            atomicAdd(&positions[2*i+1], -correctionY / mi);
            atomicAdd(&positions[2*j], correctionX / mj);
            atomicAdd(&positions[2*j+1], correctionY / mj);

        }
    }
}

__global__ void applyFrictionKernel(float* velocities, float* positions, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    float friction_coefficient = 0.3f; // Tunable constant
    velocities[2 * idx] *= friction_coefficient;
    velocities[2 * idx + 1] *= friction_coefficient;
}

__global__ void integrateKernel(float* positions, float* velocities, float* forces, float* masses, float dt, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    float ax = forces[2 * idx] / masses[idx];
    float ay = forces[2 * idx + 1] / masses[idx];

    velocities[2 * idx] += ax * dt;
    velocities[2 * idx + 1] += ay * dt;

    positions[2 * idx] += velocities[2 * idx] * dt;
    positions[2 * idx + 1] += velocities[2 * idx + 1] * dt;
   
}

__global__ void wallCollisionKernel(int height, int width, float* positions, float* velocities, float* forces, float* masses, int* radii,float dt, int count){
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    float radius = radii[idx];

    float x = positions[idx*2];
    float y = positions[idx*2+1];

    if(x - radius < 0){
        velocities[idx*2] = - velocities[idx*2]*0.8f;
        positions[idx*2] = radius;
    }
    if(x+radius > width){
        velocities[idx*2] = - velocities[idx*2]*0.8f;
        positions[idx*2] = width - radius;
    }
    if(y - radius < 0){
        velocities[idx*2+1] = -velocities[idx*2+1]*0.8f;
        positions[idx*2+1] = radius;
    }
    if(y+radius > height){
        velocities[idx*2+1] = -velocities[idx*2+1]*0.8f;
        positions[idx*2+1] = height - radius;
    }
}

__global__ void clearForcesKernel(float* forces, int count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;
    forces[2 * idx] = 0.0f;
    forces[2 * idx + 1] = 0.0f;
}
