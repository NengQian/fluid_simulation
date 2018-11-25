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
using namespace std;


namespace Simulator
{
    void Simulation::timestep(Simulator::Real dt)
    {
        assert(dt >= 0.0);
        (void) dt;
    }


    Simulation::Simulation(int N, int mode, Real dt, Real eta, Real B, Real alpha, Real rest_density)
    {
        frame_count = 0;

    	switch(mode) {
    		case 1:
    			p_sphSimulator = new SPHSimulator_rigid_body(N, dt, eta, B, alpha, rest_density);
    			break;
    		case 2:
    			p_sphSimulator = new SPHSimulator_free_fall_motion(N, dt, eta, B, alpha, rest_density);
    			break;
    		case 3:
    			p_sphSimulator = new SPHSimulator_2cubes(N, dt, eta, B, alpha, rest_density);
    			break;
    		default:
    			std::cout << "Unknown model." << std::endl;
    			break;
    	}
    }

//    Simulation::Simulation(Real dt, int N) : sphSimulator(dt, N)
//    {
//    	//neighbor_search_radius = radius;
//    }

/*
    Simulation::Simulation(Real dt, int N_particles, int N_boundary,int N_frame, string fp)
        :p_sphSimulator(0),sphSimulator(* new SPHSimulator_rigid_body(dt,N_particles))// but this is not a C++ style...
    {
        //time_step = dt;
        //fluid_particals_num = N_particles;
        //boundary_particals_num = N_boundary;
        //total_frame_num = N_frame;
        //frame_count = 0;
        //file_path = fp;
    }
*/
    Simulation::~Simulation(){
        //std::cout<<"now output data to "<< file_path <<std::endl;
        //sphSimulator.output_sim_record_bin(file_path);
        //std::cout<<"output done, now delete the sphsimulator"<< std::endl;
        delete p_sphSimulator;
        //std::cout<<"delete done!"<<std::endl;
//>>>>>>> neng3
    }

    //bool Simulation::is_simulation_finshed(){
    //    return frame_count > total_frame_num;
    //}


    void Simulation::render(merely3d::Frame &frame)
    {
        render_particles(frame);
    }

    void Simulation::update()
    {
        //sphSimulator.update_freefall_motion();
        //sphSimulator.update_two_cubes_collision();
        //sphSimulator.update_rigid_body_simulation();
    	p_sphSimulator->update_simulation();
    	//p_sphSimulator->update_sim_record_state();
        frame_count++;
        std::cout<<"iterate "<<frame_count<<std::endl;
    }

    void Simulation::render_particles(merely3d::Frame &frame)
    {
    	std::vector<RealVector3> particles = p_sphSimulator->get_positions();
    	std::vector<RealVector3> boundary_particles = p_sphSimulator->get_boundary_positions();

        Real particle_radius = p_sphSimulator->get_particle_radius();

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

