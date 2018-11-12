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

        /// Update the simulation by advancing the global state by
        /// dt time.
        void timestep(Real dt);

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
        std::chrono::milliseconds  time_interval;   // 10 milliseconds


        /*-----these for gui state--------*/
        bool playback_flag;
        bool pausing_flag;

    };
}
