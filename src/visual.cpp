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
        particle_radius = 1.0/cbrt(static_cast<Real>(particles_num)); // could be change
        total_frame_num = sim_rec.states.size();
        real_time_step = sim_rec.timestep;
        total_time = real_time_step*total_frame_num;

        speed_ratio = 1;
        playback_flag = false;
        pausing_flag = false;
        render_velocity_flag = false;
        render_acc_flag = false;
        render_max_acc = 0.25;
        render_max_velocity = 0.25;
        time_interval = std::chrono::milliseconds(static_cast<unsigned int>(1000*real_time_step*1.0/speed_ratio));

        file_name = fp;
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


    float Visualization::velocity_to_float(Eigen::Vector3f velocity)
    {

        float velocity_norm = velocity.norm();
        std::cout << velocity_norm << std::endl;

        velocity_norm = std::min(velocity.norm(),render_max_velocity);
        return velocity_norm/render_max_velocity;
    }

    float Visualization::acc_to_float(Eigen::Vector3f acc)
    {
        float max_acc = 0.5f;
        float acc_norm = acc.norm();
        acc_norm = std::min(acc_norm,render_max_acc);

        return acc_norm/render_max_acc;
    }


    void Visualization::render(merely3d::Frame &frame)
    {
//        // Below we demonstrate how to render various primitives with merely3d.
//        frame.draw(renderable(Rectangle(0.5, 0.5))
//                           .with_position(1.0, 0.0, 0.5)
//                           .with_orientation(AngleAxisf(0.78, Vector3f(1.0f, 0.0f, 0.0f)))
//                           .with_material(Material().with_color(Color(0.5, 0.3, 0.3))));

//        frame.draw(renderable(Box(1.0, 1.0, 1.0))
//                           .with_position(4.0, 0.0, 1.1));

//        frame.draw(renderable(Box(0.2, 5.0, 1.0))
//                           .with_position(0.0, 0.0, 1.0));

//        frame.draw(renderable(Box(0.2, 1.0, 1.0))
//                           .with_position(0.0, 0.0, 5.0)
//                           .with_orientation(AngleAxisf(0.5, Vector3f(1.0, 1.0, 1.0)))
//                           .with_material(Material().with_color(red())));

//        const auto floor_color = Color(0.5f, 0.35f, 0.35f);
//        frame.draw(renderable(Rectangle(20.0f, 20.0f))
//                           .with_position(0.0f, 0.0f, 0.0f)
//                           .with_material(Material().with_color(floor_color)));

//        frame.draw(renderable(Sphere(1.0))
//                           .with_position(3.0, 3.0, 3.0));

//        // Note that if you use RealVector3 for your position, you might need to cast it into a Vector3f
//        // since merely3d uses float for positions. Here's an example:
//        const RealVector3 my_position(1.0, 5.0, 10.0);
//        frame.draw(renderable(Sphere(3.0))
//                    .with_position(my_position.cast<float>()));

//        frame.draw_line(Line(Vector3f(0.0, 0.0, 0.0), Vector3f(10.0, -5.0, 10.0)));

        // Draw some (big) particles
        const auto prev_time = std::chrono::steady_clock::now();
        std::vector<mParticle> particles = sim_rec.states[sim_count].particles;
        for (size_t i = 0; i < particles.size(); ++i)
        {
            Simulator::mParticle mparticle(particles[i]);
            Eigen::Vector3f p(static_cast<float>(mparticle.position[0]), static_cast<float>(mparticle.position[1]), static_cast<float>(mparticle.position[2]));

            if(render_acc_flag == render_velocity_flag) //if we set both rendering, we see it as no rendering
                frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(0.0f, 0.0f, 1.0f)));
            else if(render_velocity_flag){
                Eigen::Vector3f v(static_cast<float>(mparticle.velocity[0]), static_cast<float>(mparticle.velocity[1]), static_cast<float>(mparticle.velocity[2]));
                float r = velocity_to_float(v);
                float b = 1.0f- r;
                frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(r, 0.0f, b)));
            }
            else{
                //render accerlation
                Eigen::Vector3f a(static_cast<float>(mparticle.acceleration[0]), static_cast<float>(mparticle.acceleration[1]), static_cast<float>(mparticle.acceleration[2]));
                float r = velocity_to_float(a);
                float b = 1.0f- r;
                frame.draw_particle(Particle(p).with_radius(particle_radius).with_color(Color(r, 0.0f, b)));            }
        }


        if(pausing_flag==false){
            if(playback_flag==false)
            {
                if(sim_count < total_frame_num-1)
                    sim_count++;
            }
            else{
                if(sim_count>0)
                    sim_count--;
            }
        }
        //        /// Set up a new ImGui window
        //        /*
        //        if (ImGui::Begin("Settings", NULL, ImVec2(300, 200)))
        //        {
        //            ImGui::TextWrapped("Find neighbors with radius r");
        //            ImGui::SliderFloat("r", &neighbor_search_radius, 0.0, 2.0);
        //            if (ImGui::Button("reset radius"))
        //            	do_neighbor_searching = true;

        //            ImGui::TextWrapped("");
        //            if (ImGui::Button("Choose Random Particle"))
        //            {
        //            	change_center = true;
        //            	do_neighbor_searching = true;
        //            }

        //            ImGui::TextWrapped("");
        //            if (ImGui::Button("Generate Random Particles"))
        //            	do_particle_generating = true;

        //            ImGui::TextWrapped("");
        //            if (ImGui::Checkbox("CompactN", &do_compactN))
        //            {
        //            	do_neighbor_searching = true;
        //            }

        //            ImGui::TextWrapped("elapsed time: %s ms", time_string.c_str());
        //        }
        //        ImGui::End();
        // Draw GUI stuff with Dear ImGui



//        ImGui::SetNextWindowPos(ImVec2(650, 650), ImGuiCond_FirstUseEver); // Normally user code doesn't need/want to call this because positions are saved in .ini file anyway. Here we just want to make the demo initial state a bit more friendly!
//        ImGui::ShowDemoWindow();

        /// Begin begins a new ImGui window that you can move around as you please
        if (ImGui::Begin(file_name.c_str(), NULL, ImVec2(300, 200)))
        {
            // See the code for ImGui::ShowDemoWindow() for examples of how to use
            // various ImGui widgets.
            ImGui::SliderInt("current frame", &sim_count, 0, total_frame_num-1);
            ImGui::Checkbox("play backward", &playback_flag);
            ImGui::TextWrapped("current simulation time: %0.2f s", real_time_step*sim_count);
            ImGui::TextWrapped("remaining simulation time: %0.2f s",total_time - real_time_step*(sim_count+1));

            if(ImGui::Button("back"))
            {
                if(sim_count > 0)
                    sim_count--;
            }
            ImGui::SameLine();
            ImGui::Checkbox("pausing", &pausing_flag); ImGui::SameLine();
            if(ImGui::Button("forward"))
            {
                if(sim_count < total_frame_num-1)
                    sim_count++;
            }

            if(ImGui::SliderFloat("speed_ratio", &speed_ratio, 0.1f, 10.0f))
            {
                time_interval = std::chrono::milliseconds(static_cast<unsigned int>(1000*real_time_step*1.0/speed_ratio));
            }
            ImGui::TextWrapped("simulation time step: %0.3f s",real_time_step);
            ImGui::TextWrapped("particles number %d",particles_num);

            ImGui::Checkbox("render velocity", &render_velocity_flag);ImGui::SameLine();
            ImGui::Checkbox("render acceleration", &render_acc_flag);
            ImGui::SliderFloat("max render velocity", &render_max_velocity, 0.1f, 10.0f);
            ImGui::SliderFloat("max render acc", &render_max_acc, 0.1f, 10.0f);



        }
        ImGui::End();

        const auto now = std::chrono::steady_clock::now();
        //const duration<double> elapsed = now - prev_time;
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_time);
        std::this_thread::sleep_for(std::chrono::milliseconds(time_interval - elapsed_time));  //elapsed_time could be already exceed time_interval



    }
}

