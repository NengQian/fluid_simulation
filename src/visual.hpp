#pragma once

#include <merely3d/frame.hpp>

#include "math_types.hpp"  //maybe even math_types is not needs
#include "sim_record.hpp"
#include "Particle.hpp"

#include <iostream>
#include <fstream>
#include <string.h>

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>

#include <chrono>



namespace Simulator
{
    class Visualization
    {
    public:

        Visualization();
        Visualization(std::string fp);


        /// Render the current state of the simulation onto the given frame.
        //void render(merely3d::Frame & frame);


        /*------------for cereals to read data---*/
        void input_sim_record_xml(std::string fp);
        void input_sim_record_bin(std::string fp);

        void render(merely3d::Frame &frame);

        // You probably want some methods to add bodies to the system
        // void addBody(const RigidBody & body);

    private:
        SimulationRecord sim_rec;
        int sim_count;
        int total_frame_num;
        int particles_num;
        Real particle_radius;
        Real real_time_step;
        Real total_time;
        float speed_ratio;
        std::chrono::milliseconds  time_interval;

        std::string file_name;


        /*-----these for gui state--------*/
        bool playback_flag;
        bool pausing_flag;
        bool render_velocity_flag;
        bool render_acc_flag;
        float render_max_velocity;
        float render_max_acc;

        float acc_to_float(Eigen::Vector3f a);
        float velocity_to_float(Eigen::Vector3f v);

    };
}
