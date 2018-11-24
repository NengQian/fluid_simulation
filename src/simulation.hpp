#pragma once

#include "math_types.hpp"
#include <merely3d/frame.hpp>
#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>

#include "SPHSimulator.hpp"
#include "SPHSimulator_2cubes.hpp"
#include "SPHSimulator_rigid_body.hpp"
#include <string>
using namespace std;
namespace Simulator
{
    class Simulation
    {
    public:
        //Simulation(Real dt, int N=5);
        Simulation(Real dt, int N_particles, int N_boundary=1,int N_frame=1500 , string fp = "../../sim_result/test_rigid.bin" ); // mode); here mode indicate which sphsimulator will be called.
        ~Simulation(); // when deconstruct, it start to output data
        void timestep(Real dt);
        void render(merely3d::Frame &frame);
        void update();
        bool is_simulation_finshed();

        SPHSimulator& sphSimulator;
        SPHSimulator* p_sphSimulator;
        //float neighbor_search_radius;

        // You probably want some methods to add bodies to the system
        // void addBody(const RigidBody & body);

    private:
        void render_bounding_box(merely3d::Frame &frame);
        void render_particles(merely3d::Frame &frame);
        void render_sweep_line(merely3d::Frame &frame);
        Real time_step;
        int fluid_particals_num;
        int boundary_particals_num; //could change, how we assign the boundary particals num? or through the volume?
        int total_frame_num;
        int frame_count;
        string file_path;
    };
}
