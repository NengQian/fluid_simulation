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


    Simulation::Simulation(int N, int mode, Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, string fp, bool if_print, int with_viscosity, int with_XSPH, int solver_type)
    {
        file_path = fp;
        frame_count = 0;
        if_print_iteration = if_print;
        time_step = dt;
        this->eta = eta;
        this->B = B;
        this->alpha = alpha;
  //      is_finished = false;
    	switch(mode) {
    		case 1:
                p_sphSimulator = new SPHSimulator_dam_breaking(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
    			break;
    		case 2:
                p_sphSimulator = new SPHSimulator_drop_center(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
    			break;
    		case 3:
                p_sphSimulator = new SPHSimulator_free_fall_motion(N, uParticle_len, dt,eta, B, alpha, rest_density);
    			break;
    		case 4:
                p_sphSimulator = new SPHSimulator_2cubes(N,uParticle_len, dt, eta, B, alpha, rest_density, solver_type);
    			break;
            case 5:
                p_sphSimulator = new SPHSimulator_dam_breaking_thin(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
                break;
            case 6:
                p_sphSimulator = new SPHSimulator_double_dam_breaking(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
                break;
            case 7:
                p_sphSimulator = new SPHSimulator_drop_on_water(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
                break;
            case 8:
                p_sphSimulator = new SPHSimulator_fluid_pillar(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type);
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
        std::cout<<"now output data to "<< file_path <<std::endl;
        p_sphSimulator->output_sim_record_bin(file_path);
        std::cout<<"output done, now delete the sphsimulator"<< std::endl;
        delete p_sphSimulator;
        std::cout<<"delete done!"<<std::endl;
    }

//    bool Simulation::is_simulation_finshed(){
//        return is_finished;
//    }


    void Simulation::render(merely3d::Frame &frame)
    {
        render_particles(frame);
    }

    void Simulation::update()
    {
    	p_sphSimulator->update_simulation();
        p_sphSimulator->update_sim_record_state();
        frame_count++;
        if(if_print_iteration)
            std::cout<<"iterate "<<frame_count<<std::endl;
    }

    void Simulation::render_particles(merely3d::Frame &frame)
    {
    	static bool render_boundary = true;

    	const std::vector<RealVector3>& particles = p_sphSimulator->get_positions();
        Real particle_radius = p_sphSimulator->get_particle_radius();

    	for (size_t i = 0; i < particles.size(); ++i)
        {
       		Eigen::Vector3f p(static_cast<float>(particles[i][0]), static_cast<float>(particles[i][1]), static_cast<float>(particles[i][2]));
        	frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(0.0f, 0.0f, 1.0f)));
        }

    	if (render_boundary)
    	{
        	const std::vector<RealVector3>& boundary_particles = p_sphSimulator->get_boundary_positions();
            for (size_t i = 0; i < boundary_particles.size(); ++i)
            {
                Eigen::Vector3f p(static_cast<float>(boundary_particles[i][0]), static_cast<float>(boundary_particles[i][1]), static_cast<float>(boundary_particles[i][2]));
                frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(0.5f, 0.5f, 0.5f)));
            }
    	}


        /// Begin begins a new ImGui window that you can move around as you please
        if (ImGui::Begin("Parameters", NULL, ImVec2(300, 200)))
        {
            ImGui::TextWrapped("B = %0.1f", B  );
            ImGui::TextWrapped("dt = %f", time_step  );
            ImGui::TextWrapped("eta = %0.2f", eta  );
            ImGui::TextWrapped("alpha = %0.2f", alpha  );

            ImGui::Checkbox("boundary", &render_boundary);
        }
        ImGui::End();
    }
}

