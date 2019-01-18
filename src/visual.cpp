#include "visual.hpp"

#include <merely3d/merely3d.hpp>

#include "imgui/imgui.h"
#include <Eigen/Dense>

#include <cassert>
#include <math.h>       /* cbrt */
#include <chrono>
#include <thread>

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
using namespace Eigen;
namespace Simulator
{

    Visualization::Visualization()
    {

    }

    Visualization::Visualization(std::string fp)
    {
        sim_count = 0;
        input_sim_record_bin(fp);
        SimulationState simu_state = sim_rec.states[0];
        particles_num = simu_state.particles.size();
        particle_radius = sim_rec.unit_particle_length/2.0;
        total_frame_num = sim_rec.states.size();
        eta = sim_rec.eta;
        rest_density = sim_rec.rest_density;
        B = sim_rec.B;
        alpha = sim_rec.alpha;
        real_time_step = sim_rec.timestep;
        solver_type = sim_rec.solver_type;

        speed_ratio = 1.0;
        playback_flag = false;
        pausing_flag = true;
        render_velocity_flag = false;
        render_density_flag = false;
        render_max_density = 1000.0f;
        render_max_velocity = 0.25f;
        counter = 0;
        render_step = 1;

        file_name = fp;

        boundary_particle_size = 0.2*particle_radius;
        particle_size = particle_radius;
    }


    void Visualization::input_sim_record_xml(std::string fp)
    {
        std::ifstream file(fp);
        cereal::XMLInputArchive input(file); // stream to cout
        input(sim_rec);  //not good... maybe directly ar the vector
    }


    void Visualization::input_sim_record_bin(std::string fp)
    {
        std::ifstream file(fp);
        cereal::BinaryInputArchive input(file); // stream to cout
        input(sim_rec);  //not good... maybe directly ar the vector
    }


    float Visualization::velocity_to_float(const Eigen::Vector3f& velocity)
    {

        float velocity_norm = velocity.norm();
        velocity_norm = std::min(velocity.norm(),render_max_velocity);
        return velocity_norm/render_max_velocity;
    }

    /*
    float Visualization::acc_to_float(const Eigen::Vector3f& acc)
    {
        float acc_norm = acc.norm();
        acc_norm = std::min(acc_norm,render_max_acc);
        return acc_norm/render_max_acc;
    }
	*/

    void Visualization::render(merely3d::Frame &frame)
    {
    	// Draw floor
		const auto floor_color = Color(0.9, 0.9, 0.9);

		frame.draw(renderable(Rectangle(20.0f, 20.0f))
						   .with_position(0.0f, 0.0f, 0.0f)
						   .with_material(Material().with_color(floor_color)));

        // Draw some (big) particles

        std::vector<mParticle>& particles = sim_rec.states[sim_count].particles;
        for (size_t i = 0; i < particles.size(); ++i)
        {
            Simulator::mParticle& mparticle = particles[i];
            Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));

            if(render_density_flag == render_velocity_flag) //if we set both rendering, we see it as no rendering
            {
            	frame.draw_particle(Particle(p).with_radius(particle_size).with_color(Color(0.0f, 0.0f, 1.0f)));
            }
            else if(render_velocity_flag)
            {
                Eigen::Vector3f v(static_cast<float>(mparticle.velocity[0]), static_cast<float>(mparticle.velocity[1]), static_cast<float>(mparticle.velocity[2]));
                float r = velocity_to_float(v);
                float b = 1.0f - r;
                frame.draw_particle(Particle(p).with_radius(particle_size).with_color(Color(r, 0.0f, b)));
            }
            else
            {
                //render density
                float d = static_cast<float>(mparticle.density);

                float f = std::min(d, render_max_density) / render_max_density;
                float r = f * f * f * f;
                float b = 1.0f - r;
                frame.draw_particle(Particle(p).with_radius(particle_size).with_color(Color(r, 0.0f, b)));
            }
        }

        // render boundary particles
        {
            std::vector<mParticle>& bp = sim_rec.boundary_particles;
            for (size_t i =0;i<bp.size();++i)
            {
                Simulator::mParticle& mparticle = bp[i];
                Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));
                frame.draw_particle(Particle(p).with_radius(boundary_particle_size).with_color(Color(0.2f, 0.2f, 0.2f)));
            }
        }


        if (pausing_flag==false)
        {
            if (speed_ratio<1.0)
            {
                counter++;
                if(counter < std::abs(static_cast<int>(1.0/speed_ratio+0.5)))
                {
                    render_step = 0; // the counter hasn't reach the bound we set, so we continue render the same frame
                }else{
                    render_step = 1; //reach, could go to next one
                    counter = 0;
                }
            }else{// speed_ratio>1.0
                render_step = std::abs(static_cast<int>(speed_ratio+0.5)); // if speed_ratio biger than 1.0,
                                                                           // we do somehow like downsampling
                                                                           // sampling rate is simply the speed_ratio's round int.
            }

            if(playback_flag==false)
            {
                sim_count = std::min(sim_count+render_step,total_frame_num-1);
            }
            else{
                sim_count = std::max(sim_count-render_step,0);
            }
        }




        /// Control panel GUI
        if (ImGui::Begin("Control Panel", NULL, ImVec2(400, 300)))
        {
        	ImGui::SetWindowPos("Control Panel", ImVec2(10, 10));
            // various ImGui widgets.
            ImGui::SliderInt("current frame", &sim_count, 0, total_frame_num-1);

            ImGui::SliderFloat("play speed", &speed_ratio, 0.1, 5, "%.1f");

            ImGui::Checkbox("backward", &playback_flag);
            ImGui::SameLine();
            ImGui::Checkbox("pause", &pausing_flag);

            ImGui::Text("Rendering Options: ");
            ImGui::SameLine();
            ImGui::Checkbox("velocity", &render_velocity_flag);
            ImGui::SameLine();
            ImGui::Checkbox("density", &render_density_flag);

            ImGui::SliderFloat("max velocity", &render_max_velocity, 0.1f, 10.0f, "%.1f");
            ImGui::SliderFloat("max density", &render_max_density, 100.0f, 3000.0f, "%.1f");
            ImGui::SliderFloat("boundary particle size", &boundary_particle_size, 0.0f, 0.1f, "%.2f");
            ImGui::SliderFloat("particle size", &particle_size, 0.01f, 0.1f, "%.2f");
        }
        ImGui::End();

        // Setting GUI
        if (ImGui::Begin("Setting", NULL, ImVec2(300, 200)))
        {
        	ImGui::SetWindowPos("Setting", ImVec2(10, 260));

            ImGui::TextWrapped("particle number: %d", particles_num);
        	ImGui::TextWrapped("timestep: %0.4f s", real_time_step);
        	ImGui::TextWrapped("eta: %0.1f", eta);
        	ImGui::TextWrapped("rest density: %0.1f", rest_density);
        	ImGui::TextWrapped("stiffness: %0.1f", B);
        	ImGui::TextWrapped("alpha: %0.2f", alpha);

        	if (solver_type == 0)
            	ImGui::Text("solver: WCSPH");
        	else if (solver_type == 1)
            	ImGui::Text("solver: PBF");
        }
        ImGui::End();

    }
}

