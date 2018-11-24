#pragma once
#include "math_types.hpp"
#include "SPHSimulator.hpp"

#include <merely3d/frame.hpp>
#include <Eigen/Geometry>

namespace Simulator
{
    class Simulation
    {
    public:
        Simulation(int N, Real dt, Real eta, Real B, Real rest_density);

        void timestep(Real dt);
        void render(merely3d::Frame &frame);
        void update();

        SPHSimulator sphSimulator;

        // You probably want some methods to add bodies to the system
        // void addBody(const RigidBody & body);

    private:
        void render_particles(merely3d::Frame &frame);
    };
}
