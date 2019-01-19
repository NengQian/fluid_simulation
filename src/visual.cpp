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

    Visualization::Visualization(std::string simfile)
    {
        sim_count = 0;
        input_sim_record_bin(simfile);
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

        boundary_particle_size = 0.2*sim_rec.unit_particle_length;
        particle_size = sim_rec.unit_particle_length;

        render_mesh_flag = false;
        no_mesh = true;

        render_particle_flag = true;
        render_bounding_box_flag = false;
        render_discarded_particle_flag = false;
    }

    Visualization::Visualization(std::string simfile, std::string meshfile)
    {
        sim_count = 0;
        input_sim_record_bin(simfile);
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

        boundary_particle_size = 0.2*sim_rec.unit_particle_length;
        particle_size = sim_rec.unit_particle_length;


        render_mesh_flag = true;
        no_mesh = false;
        input_mesh_record_bin(meshfile);

        unit_voxel_length = mesh_rec.unit_voxel_length;
        c = mesh_rec.c;
        bounding_box = mesh_rec.meshSeries[0].bounding_box;
        origin = mesh_rec.meshSeries[0].origin;

        total_grid = bounding_box[0]/unit_voxel_length * bounding_box[1]/unit_voxel_length * bounding_box[2]/unit_voxel_length;

        render_particle_flag = true;
        render_bounding_box_flag = false;
        render_discarded_particle_flag = true;
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

    void Visualization::input_mesh_record_bin(std::string fp)
    {
        std::ifstream file(fp);
        cereal::BinaryInputArchive input(file); // stream to cout
        input(mesh_rec);  //not good... maybe directly ar the vector
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
    	// set total grid number
    	Vector3f bb = mesh_rec.meshSeries[sim_count].bounding_box;
        total_grid = bb[0]/unit_voxel_length * bb[1]/unit_voxel_length * bb[2]/unit_voxel_length;

    	// Draw floor
		const auto floor_color = Color(0.9, 0.7, 0.5);

		frame.draw(renderable(Rectangle(100.0f, 100.0f))
						   .with_position(-20.0f, -20.0f, -3.0f)
						   .with_material(Material().with_color(floor_color)));
		frame.draw(renderable(Rectangle(100.0f, 100.0f))
						   .with_position(30.0f, -20.0f, 47.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(0.0f, 1.0f, 0.0f)))
						   .with_material(Material().with_color(floor_color)));
		frame.draw(renderable(Rectangle(100.0f, 100.0f))
						   .with_position(-20.0f, 30.0f, 47.0f)
						   .with_orientation(AngleAxisf(0.5*M_PI, Vector3f(1.0f, 0.0f, 0.0f)))
						   .with_material(Material().with_color(floor_color)));


        // Draw some (big) particles

		if (render_particle_flag)
		{
			std::vector<mParticle>& particles = sim_rec.states[sim_count].particles;

			for (size_t i = 0; i < particles.size(); ++i)
			{
				Simulator::mParticle& mparticle = particles[i];
				Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));


				if (render_discarded_particle_flag)
				{
	    			const auto dp_color = Color(0., 0., 0.);

					if (mparticle.density < 185.0)
					{
						frame.draw(renderable(Sphere(particle_size*0.5))
								    					   .with_position(p)
								    					   .with_material(Material().with_pattern_grid_size(particle_size*0.5).with_color(dp_color)));
						continue;
					}
				}

				if(render_density_flag == render_velocity_flag) //if we set both rendering, we see it as no rendering
				{
					frame.draw_particle(Particle(p).with_radius(particle_size*0.5).with_color(Color(0.0f, 0.5f, 1.0f)));
				}
				else if(render_velocity_flag)
				{
					Eigen::Vector3f v(static_cast<float>(mparticle.velocity[0]), static_cast<float>(mparticle.velocity[1]), static_cast<float>(mparticle.velocity[2]));
					float r = velocity_to_float(v);
					float b = 1.0f - r;
					frame.draw_particle(Particle(p).with_radius(particle_size*0.5).with_color(Color(r, 0.5f-r*0.5, b)));
				}
				else
				{
					//render density
					float d = static_cast<float>(mparticle.density);

					float f = std::min(d, render_max_density) / render_max_density;
					float r = f * f * f * f;
					float b = 1.0f - r;
					frame.draw_particle(Particle(p).with_radius(particle_size*0.5).with_color(Color(r, 0.5f-r*0.5, b)));
				}
			}
		} else if (render_discarded_particle_flag) {
			std::vector<mParticle>& particles = sim_rec.states[sim_count].particles;

			for (size_t i = 0; i < particles.size(); ++i)
			{
				Simulator::mParticle& mparticle = particles[i];
				Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));

    			const auto dp_color = Color(0., 0., 0.);

				if (render_discarded_particle_flag)
				{
					if (mparticle.density < 185.0)
					{
		    			frame.draw(renderable(Sphere(particle_size*0.5))
		    					   .with_position(p)
		    					   .with_material(Material().with_pattern_grid_size(particle_size*0.5).with_color(dp_color))
		    					  );
		    			continue;
					}
				}
			}
		}

        // render boundary particles
        {
            std::vector<mParticle>& bp = sim_rec.boundary_particles;
            for (size_t i =0;i<bp.size();++i)
            {
                Simulator::mParticle& mparticle = bp[i];
                Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));
                frame.draw_particle(Particle(p).with_radius(boundary_particle_size*0.5).with_color(Color(0.2f, 0.2f, 0.2f)));
            }
        }

        if (!no_mesh) // show mesh
        {
        	if (render_mesh_flag)
        	{
    			const auto model_color = Color(1., 1., 1.);

            	auto md = mesh_rec.meshSeries[sim_count];

            	merely3d::StaticMesh mesh(md.vertices_and_normals, md.faces);
    			frame.draw(renderable(mesh)
    					   .with_position(0.0, 0.0, 0.0)
    					   .with_material(Material().with_pattern_grid_size(0).with_color(model_color))
    					  );
        	}

        	if (render_bounding_box_flag)
        	{
        		bounding_box = mesh_rec.meshSeries[sim_count].bounding_box;
        		origin = mesh_rec.meshSeries[sim_count].origin;

    			const auto box_color = Color(1.0, 1.0, 0.0);

            	// bottom
            	Line l0( origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin - Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, 0.0), box_color);
            	Line l1( origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin - Vector3f(-bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), box_color);
            	Line l2( origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin + Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, 0.0), box_color);
            	Line l3( origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin + Vector3f(-bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), box_color);

            	// top
            	Line l4( origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, -bounding_box[2]), origin - Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, -bounding_box[2]), box_color);
            	Line l5( origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, -bounding_box[2]), origin - Vector3f(-bounding_box[0]/2.0, bounding_box[1]/2.0, -bounding_box[2]), box_color);
            	Line l6( origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, bounding_box[2]), origin + Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, bounding_box[2]), box_color);
            	Line l7( origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, bounding_box[2]), origin + Vector3f(-bounding_box[0]/2.0, bounding_box[1]/2.0, bounding_box[2]), box_color);

            	// sides
            	Line l8( origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin - Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, -bounding_box[2]), box_color);
            	Line l9( origin - Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, 0.0), origin - Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, -bounding_box[2]), box_color);
            	Line l10( origin + Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, 0.0), origin + Vector3f(bounding_box[0]/2.0, -bounding_box[1]/2.0, bounding_box[2]), box_color);
            	Line l11( origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, 0.0), origin + Vector3f(bounding_box[0]/2.0, bounding_box[1]/2.0, bounding_box[2]), box_color);

            	frame.draw_line(l0);
            	frame.draw_line(l1);
            	frame.draw_line(l2);
            	frame.draw_line(l3);
            	frame.draw_line(l4);
            	frame.draw_line(l5);
            	frame.draw_line(l6);
            	frame.draw_line(l7);
            	frame.draw_line(l8);
            	frame.draw_line(l9);
            	frame.draw_line(l10);
            	frame.draw_line(l11);
        	}

        	if (render_discarded_particle_flag)
        	{

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
        if (ImGui::Begin("Control Panel"))
        {
            // various ImGui widgets.
            ImGui::SliderInt("current frame", &sim_count, 0, total_frame_num-1);

            ImGui::SliderFloat("play speed", &speed_ratio, 0.1, 5, "%.1f");

            ImGui::Checkbox("backward", &playback_flag);
            ImGui::SameLine();
            ImGui::Checkbox("pause", &pausing_flag);
        }
        ImGui::End();

        /// Control panel GUI
        if (ImGui::Begin("Rendering Options"))
        {
            ImGui::Checkbox("particle", &render_particle_flag);
            ImGui::SameLine();
            ImGui::Checkbox("velocity", &render_velocity_flag);
            ImGui::SameLine();
            ImGui::Checkbox("density", &render_density_flag);

            ImGui::Checkbox("mesh", &render_mesh_flag);
            ImGui::SameLine();
            ImGui::Checkbox("bounding box", &render_bounding_box_flag);
            ImGui::SameLine();
            ImGui::Checkbox("isolated particle", &render_discarded_particle_flag);

            ImGui::SliderFloat("max velocity", &render_max_velocity, 0.1f, 10.0f, "%.1f");
            ImGui::SliderFloat("max density", &render_max_density, 100.0f, 3000.0f, "%.1f");
            ImGui::SliderFloat("boundary particle size", &boundary_particle_size, 0.0f, static_cast<float>(sim_rec.unit_particle_length), "%.2f");
            ImGui::SliderFloat("particle size", &particle_size, 0.01f, static_cast<float>(sim_rec.unit_particle_length), "%.2f");
        }
        ImGui::End();

        // Setting GUI
        if (ImGui::Begin("Setting"))
        {
        	ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "Simulation Setting: ");
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

        	if (!no_mesh)
        	{
            	ImGui::TextColored(ImVec4(1.0f,0.0f,0.0f,1.0f), "Rendering Setting: ");
            	ImGui::TextWrapped("grid size: %0.2f", unit_voxel_length);
            	ImGui::TextWrapped("c: %0.1f", c);
                ImGui::TextWrapped("grid number: %d", total_grid);
        	}
        }
        ImGui::End();
    }
}

