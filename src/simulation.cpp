#include "simulation.hpp"

#include "imgui/imgui.h"
#include "math_types.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <cassert>
#include <chrono>
#include <string>

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


    Simulation::Simulation()
    {

    }

    void Simulation::render(merely3d::Frame &frame)
    {
    	render_bounding_box(frame);

        static bool do_neighbor_searching = false;
        static bool do_particle_generating = true;
        static bool change_center = false;
        static bool do_compactN = true;
        static std::string time_string;

        if (do_particle_generating)
        {
        	sphSimulator.generate_random_particles();

        	do_neighbor_searching = true;
        	do_particle_generating = false;
        }

        if (do_neighbor_searching)
        {
        	if (change_center)
        	{
        		sphSimulator.index_of_source_particle = (sphSimulator.index_of_source_particle + 1) % sphSimulator.number_of_particles;
        		change_center = false;
        	}

        	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        	sphSimulator.neighbors = sphSimulator.neighborSearcher.find_neighbors_within_radius(sphSimulator.particles, sphSimulator.index_of_source_particle, (double)sphSimulator.neighbor_search_radius, do_compactN);

        	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        	time_string = std::to_string(elapsed_time.count());

        	for (size_t i=0; i < sphSimulator.neighbors.size(); ++i)
        	{
            	std::cout << "error of M4 gradient approx.: " << sphSimulator.kernelHandler.test_M4_gradient(sphSimulator.particles[sphSimulator.index_of_source_particle], sphSimulator.particles[sphSimulator.neighbors[i]], (double)sphSimulator.neighbor_search_radius, 3) << std::endl;
            	std::cout << "error of M5 gradient approx.: " << sphSimulator.kernelHandler.test_M5_gradient(sphSimulator.particles[sphSimulator.index_of_source_particle], sphSimulator.particles[sphSimulator.neighbors[i]], (double)sphSimulator.neighbor_search_radius, 3) << std::endl;
            	std::cout << "error of M6 gradient approx.: " << sphSimulator.kernelHandler.test_M6_gradient(sphSimulator.particles[sphSimulator.index_of_source_particle], sphSimulator.particles[sphSimulator.neighbors[i]], (double)sphSimulator.neighbor_search_radius, 3) << std::endl;
        	}
        	printf("\n");

        	do_neighbor_searching = false;
        }

        render_random_particles(frame);

        /// Set up a new ImGui window
        if (ImGui::Begin("Settings", NULL, ImVec2(300, 200)))
        {
            ImGui::TextWrapped("Find neighbors with radius r");
            ImGui::SliderFloat("r", &sphSimulator.neighbor_search_radius, 0.0, 20.0);
            if (ImGui::Button("reset radius"))
            	do_neighbor_searching = true;

            ImGui::TextWrapped("");
            if (ImGui::Button("Choose Random Particle"))
            {
            	change_center = true;
            	do_neighbor_searching = true;
            }

            ImGui::TextWrapped("");
            if (ImGui::Button("Generate Random Particles"))
            	do_particle_generating = true;

            ImGui::TextWrapped("");
            if (ImGui::Checkbox("CompactN", &do_compactN))
            {
            	do_neighbor_searching = true;
            }

            ImGui::TextWrapped("elapsed time: %s ms", time_string.c_str());
        }
        ImGui::End();
    }

    void Simulation::render_bounding_box(merely3d::Frame &frame)
    {
        const auto floor_color = Color(0.5f, 0.35f, 0.35f);
        frame.draw(renderable(Rectangle(20.0f, 20.0f))
                           .with_position(0.0f, 0.0f, 0.0f)
                           .with_material(Material().with_color(floor_color)));
        frame.draw(renderable(Rectangle(20.0f, 20.0f))
                           .with_position(0.0f, 10.0f, 10.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(1.0f, 0.0f, 0.0f)))
                           .with_material(Material().with_color(floor_color)));
        frame.draw(renderable(Rectangle(20.0f, 20.0f))
                           .with_position(10.0f, 0.0f, 10.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(0.0f, 1.0f, 0.0f)))
                           .with_material(Material().with_color(floor_color)));
    }

    void Simulation::render_random_particles(merely3d::Frame &frame)
    {
        std::vector<bool> drawn(sphSimulator.particles.size(), false);
    	for (size_t i = 0; i < sphSimulator.neighbors.size(); ++i)
        {
    		Eigen::Vector3f n((float)sphSimulator.particles[sphSimulator.neighbors[i]][0], (float)sphSimulator.particles[sphSimulator.neighbors[i]][1], (float)sphSimulator.particles[sphSimulator.neighbors[i]][2]);
        	if (i == 0)
        	{
            	frame.draw_particle(Particle(n).with_radius(sphSimulator.particle_radius).with_color(Color(1.0f, 0.0f, 0.0f)));
        	}
        	else
        	{
            	frame.draw_particle(Particle(n).with_radius(sphSimulator.particle_radius).with_color(Color(0.0f, 1.0f, 0.0f)));
        	}
        	drawn[sphSimulator.neighbors[i]] = true;
        }

    	for (size_t i = 0; i < sphSimulator.particles.size(); ++i)
        {
        	if (!drawn[i])
        	{
        		Eigen::Vector3f p((float)sphSimulator.particles[i][0], (float)sphSimulator.particles[i][1], (float)sphSimulator.particles[i][2]);
        		frame.draw_particle(Particle(p).with_radius(sphSimulator.particle_radius).with_color(Color(0.0f, 0.0f, 1.0f)));
        	}
        }
    }
}

