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
using namespace std;


namespace Simulator
{
    void Simulation::timestep(Simulator::Real dt)
    {
        assert(dt >= 0.0);
        (void) dt;
    }


//    Simulation::Simulation(Real dt, int N) : sphSimulator(dt, N)
//    {
//    	//neighbor_search_radius = radius;
//    }


    Simulation::Simulation(Real dt, int N_particles, int N_boundary,int N_frame, string fp)
        :p_sphSimulator(0),sphSimulator(* new SPHSimulator_rigid_body(dt,N_particles))// but this is not a C++ style...
    {
        time_step = dt;
        fluid_particals_num = N_particles;
        boundary_particals_num = N_boundary;
        total_frame_num = N_frame;
        frame_count = 0;
        file_path = fp;
    }

     Simulation::~Simulation(){
        std::cout<<"now output data to "<< file_path <<std::endl;
        sphSimulator.output_sim_record_bin(file_path);
        std::cout<<"output done, now delete the sphsimulator"<< std::endl;
        delete &sphSimulator;
        std::cout<<"delete done!"<<std::endl;
    }

    bool Simulation::is_simulation_finshed(){
        return frame_count > total_frame_num;
    }


    void Simulation::render(merely3d::Frame &frame)
    {
    	//render_bounding_box(frame);
    	//render_sweep_line(frame);

        static bool do_neighbor_searching = false;
        static bool do_particle_generating = false;
        static bool change_center = false;
        static bool do_compactN = true;
        static std::string time_string;

        if (do_particle_generating)
        {
        	//sphSimulator.set_neighbor_search_radius(neighbor_search_radius);
        	sphSimulator.generate_particles();

        	do_neighbor_searching = true;
        	do_particle_generating = false;
        }

        if (do_neighbor_searching)
        {
        	if (change_center)
        	{
        		int current_idx = sphSimulator.get_index_of_source_particle();
        		size_t number_of_particles = sphSimulator.get_number_of_particles();
        		sphSimulator.set_index_of_source_particle((current_idx + 1) % number_of_particles);
        		change_center = false;
        	}

        	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        	//sphSimulator.set_neighbor_search_radius(neighbor_search_radius);
        	//sphSimulator.find_and_set_neighbors( do_compactN );

        	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        	time_string = std::to_string(elapsed_time.count());


        	//std::cout << "average error of M4 gradient approx.: " << sphSimulator.compute_average_error_of_kernel_gradient( 4 ) << std::endl;
        	//std::cout << "average error of M5 gradient approx.: " << sphSimulator.compute_average_error_of_kernel_gradient( 5 ) << std::endl;
        	//std::cout << "average error of M6 gradient approx.: " << sphSimulator.compute_average_error_of_kernel_gradient( 6 ) << std::endl;

        	//printf("\n");

        	do_neighbor_searching = false;
        }

        render_particles(frame);

        /// Set up a new ImGui window
        /*
        if (ImGui::Begin("Settings", NULL, ImVec2(300, 200)))
        {
            ImGui::TextWrapped("Find neighbors with radius r");
            ImGui::SliderFloat("r", &neighbor_search_radius, 0.0, 2.0);
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
        */
    }

    void Simulation::update()
    {
        //sphSimulator.update_freefall_motion();
        //sphSimulator.update_two_cubes_collision();
        //sphSimulator.update_rigid_body_simulation();
        sphSimulator.update_simulation();
        sphSimulator.update_sim_record_state();
        frame_count++;
        std::cout<<"iterate "<<frame_count<<std::endl;
    }

    void Simulation::render_sweep_line(merely3d::Frame &frame)
    {
    	merely3d::Line line(Vector3f(-2.0, 0.0, 0.0), Vector3f(2.0, 0.0, 0.0));
    	line.color = Color(1.0f, 1.0f, 0.0f);
        frame.draw_line(line);
    }

    void Simulation::render_bounding_box(merely3d::Frame &frame)
    {
        const auto floor_color = Color(0.5f, 0.35f, 0.35f);
        frame.draw(renderable(Rectangle(2.0f, 2.0f))
                           .with_position(0.0f, 0.0f, -1.0f)
                           .with_material(Material().with_color(floor_color)));
        frame.draw(renderable(Rectangle(2.0f, 2.0f))
                           .with_position(0.0f, 1.0f, 0.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(1.0f, 0.0f, 0.0f)))
                           .with_material(Material().with_color(floor_color)));
        frame.draw(renderable(Rectangle(2.0f, 2.0f))
                           .with_position(1.0f, 0.0f, 0.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(0.0f, 1.0f, 0.0f)))
                           .with_material(Material().with_color(floor_color)));
    }

    void Simulation::render_particles(merely3d::Frame &frame)
    {
    	std::vector<RealVector3> particles = sphSimulator.get_positions();
        std::vector<RealVector3> boundary_particles = sphSimulator.get_boundary_positions();

    	//std::vector<bool> drawn(particles.size(), false);

        //size_t source_index = sphSimulator.get_index_of_source_particle();
        Real particle_radius = sphSimulator.get_particle_radius();
        /*
		Eigen::Vector3f s(static_cast<float>(particles[source_index][0]), static_cast<float>(particles[source_index][1]), static_cast<float>(particles[source_index][2]));
		frame.draw_particle(Particle(s).with_radius(particle_radius).with_color(Color(1.0f, 0.0f, 0.0f)));
    	drawn[source_index] = true;

        //std::vector<size_t> neighbors = sphSimulator.get_neighbors();

        for (size_t i = 0; i < neighbors.size(); ++i)
        {
    		Eigen::Vector3f n(static_cast<float>(particles[neighbors[i]][0]), static_cast<float>(particles[neighbors[i]][1]), static_cast<float>(particles[neighbors[i]][2]));
    		frame.draw_particle(Particle(n).with_radius(particle_radius).with_color(Color(0.0f, 1.0f, 0.0f)));
        	drawn[neighbors[i]] = true;
        }

        //sphSimulator.intersection_with_sweep_line();
        */
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

