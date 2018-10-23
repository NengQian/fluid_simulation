#pragma once

#include "math_types.hpp"
#include <merely3d/frame.hpp>
#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>

#include "SPHSimulator.hpp"

namespace Simulator
{
    class Simulation
    {
    public:
        Simulation();

        void timestep(Real dt);
        void render(merely3d::Frame &frame);

        SPHSimulator sphSimulator;

        // You probably want some methods to add bodies to the system
        // void addBody(const RigidBody & body);

    private:
        void render_bounding_box(merely3d::Frame &frame);
        void render_random_particles(merely3d::Frame &frame);
    };
}
