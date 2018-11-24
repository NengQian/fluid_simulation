#include "simulation.hpp"

#include "imgui/imgui.h"
#include "math_types.hpp"

#include <merely3d/merely3d.hpp>
#include <cassert>

using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::Line;
using merely3d::Color;
using merely3d::Material;
using merely3d::Sphere;
using merely3d::red;
using merely3d::Particle;

using Eigen::AngleAxisf;
using Eigen::Vector3f;

namespace Simulator
{
    void Simulation::timestep(Simulator::Real dt)
    {
        assert(dt >= 0.0);
        (void) dt;
    }


    Simulation::Simulation(int N, Real dt, Real eta, Real B, Real rest_density) : sphSimulator(N, dt, eta, B, rest_density)
    {

    }

    void Simulation::render(merely3d::Frame &frame)
    {
        render_particles(frame);
    }

    void Simulation::update()
    {
        //sphSimulator.update_freefall_motion();
        //sphSimulator.update_two_cubes_collision();
    	sphSimulator.update_rigid_body_simulation();
    }

    void Simulation::render_particles(merely3d::Frame &frame)
    {
    	std::vector<RealVector3> particles = sphSimulator.get_positions();
    	std::vector<RealVector3> boundary_particles = sphSimulator.get_boundary_positions();
        Real particle_radius = sphSimulator.get_particle_radius();

    	for (size_t i = 0; i < particles.size(); ++i)
        {
       		Eigen::Vector3f p(static_cast<float>(particles[i][0]), static_cast<float>(particles[i][1]), static_cast<float>(particles[i][2]));
        	frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(0.0f, 0.0f, 1.0f)));
        }

    	for (size_t i = 0; i < boundary_particles.size(); ++i)
        {
       		Eigen::Vector3f p(static_cast<float>(boundary_particles[i][0]), static_cast<float>(boundary_particles[i][1]), static_cast<float>(boundary_particles[i][2]));
       		frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(0.5f, 0.5f, 0.5f)));
        }
    }
}

