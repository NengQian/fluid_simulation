#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include <SPHSimulator.hpp>
#include <SPHSimulator_2cubes.hpp>
#include "simulation.hpp"
#include <CLI11.hpp>
#include <string>

using namespace std;

std::string fold = "./sim_result/";

void generate_sim_rec(int N, int mode, int total_simulation, int step_size, std::string& output_file, Real dt, Real eta, Real B, Real rest_density)  // maybe I should directly put all particles in all frame to one vector...
{
    // a for loop to generate every thing, and then run...
//<<<<<<< HEAD
	Simulation simulation(N, mode, dt, eta, B, rest_density);
    //SPHSimulator test_simulator(5);
    //int step_size = 10;

    //for(int i=0;i<15000;++i)
    //{
    //    test_simulator.update_rigid_body_simulation();
//=======
    //SPHSimulator test_simulator(0.01,10);
    //SPHSimulator_2cubes test_simulator_1(0.01, 5);
    //SPHSimulator& test_simulator(test_simulator_1);


    for(int i=0;i<total_simulation;++i)
    {
        simulation.p_sphSimulator->update_simulation();
//>>>>>>> neng3

        if (i % step_size == 0)
        	simulation.p_sphSimulator->update_sim_record_state();

        std::cout<<"iteration "<< i <<std::endl;
    }

    simulation.p_sphSimulator->output_sim_record_bin(fold+output_file);

    return;
}



int main(int argc, char **argv)
{
    CLI::App CLIapp{"Cereal_Recorder"};

    // Define options
    float eta = 1.2f;
    CLIapp.add_option("-e, --eta", eta, "Eta: normally 1.0~1.5");

    float rest_density = 1000.0f;
    CLIapp.add_option("-d, --rest_density", rest_density, "Fluid Rest density: 1000 (kg/m^3) on water for instance");

    float B = 1000.0f;
    CLIapp.add_option("-b, --stiffness", B, "Stiffness of pressure force");

    float dt = 0.01f;
    CLIapp.add_option("-t, --dt", dt, "Elapsed time");

    std::vector<float> cuboid_side_lengths = {2.0f};
    CLIapp.add_option("-l, --side_lengths", cuboid_side_lengths, "Side length of cuboid(if existed)");

    float particle_radius = 0.1f;
    CLIapp.add_option("-r, --particle_radius", particle_radius, "Radius of particles");

    int step_size = 10;
    CLIapp.add_option("-z, --step_size", step_size, "record once every <step_size> frames");

    CLIapp.option_defaults()->required();
    int N;
    CLIapp.add_option("-n, --N", N, "Number of particles per edge");

    int mode;
    CLIapp.add_option("-m, --mode", mode, "Simulation mode: 1 for rigid_body | 2 for free fall | 3 for 2-cube collision");

    int total_simulation;
    CLIapp.add_option("-s, --total_simulation", total_simulation, "Number of simulations to record");

    std::string output_file;
    CLIapp.add_option("-o, --output_file", output_file, "output filename");



    CLI11_PARSE(CLIapp, argc, argv);

    generate_sim_rec(N, mode, total_simulation, step_size, output_file, static_cast<Real>(dt), static_cast<Real>(eta), static_cast<Real>(B), static_cast<Real>(rest_density));


    return 0;

}




