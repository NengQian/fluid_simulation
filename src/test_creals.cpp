#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

#include "simulation.hpp"
#include <CLI11.hpp>
#include <string>

#include "SPHSimulator.hpp"
#include "SPHSimulator_2cubes.hpp"
#include "SPHSimulator_rigid_body.hpp"
#include "SPHSimulator_free_fall_motion.hpp"
#include "SPHSimulator_dam_breaking.hpp"
#include "SPHSimulator_drop_center.hpp"

using namespace std;

std::string fold = "./sim_result/";

void generate_sim_rec(int N, int mode, int total_simulation, int step_size, std::string& output_file, Real dt, Real eta, Real B, Real alpha, Real rest_density, int viscosity, int XSPH)  // maybe I should directly put all particles in all frame to one vector...
{
    // a for loop to generate every thing, and then run...
    Simulation simulation(N, mode, 0.05, dt, eta, B, alpha, rest_density, fold+output_file, false, viscosity, XSPH);

    for(int i=0;i<total_simulation;++i)
    {
        simulation.p_sphSimulator->update_simulation();

        if (i % step_size == 0)
        	simulation.p_sphSimulator->update_sim_record_state();

        std::cout<<"iteration "<< i <<std::endl;
    }

    //simulation.p_sphSimulator->output_sim_record_bin(fold+output_file);

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
    CLIapp.add_option("-r, --particle_radius", particle_radius, "Radius of particles( useless now)");

    int step_size = 10;
    CLIapp.add_option("-z, --step_size", step_size, "record once every <step_size> frames");

    float alpha = 0.08f;
    CLIapp.add_option("-a, --alpha", alpha, "parameter of viscosity");

    int with_viscosity = 1;
    CLIapp.add_option("-v, --with_viscosity", with_viscosity, "add viscosity");

    int with_XSPH = 1;
    CLIapp.add_option("-x, --with_XSPH", with_XSPH, "use XSPH to update position");

    CLIapp.option_defaults()->required();
    int N = 3;
    CLIapp.add_option("-n, --N", N, "Number of particles per edge");

    int mode = 1;
    CLIapp.add_option("-m, --mode", mode, "Simulation mode: 1 for dam breaking | 2 for dropping the water from the center of boundary | 3 for free fall | 4 for 2-cube collision");

    int total_simulation = 1000;
    CLIapp.add_option("-s, --total_simulation", total_simulation, "Number of simulations to record");

    std::string output_file = "../../sim_result/test.bin";
    CLIapp.add_option("-o, --output_file", output_file, "output filename");

    float unit_particle_length ;
    CLIapp.add_option("-u, --unit_particle_length", unit_particle_length, " the intervel length between two particles per axis.");


    CLI11_PARSE(CLIapp, argc, argv);

    generate_sim_rec(N, mode, total_simulation, step_size, output_file, static_cast<Real>(dt), static_cast<Real>(eta), static_cast<Real>(B), static_cast<Real>(alpha), static_cast<Real>(rest_density), with_viscosity, with_XSPH);


    // a for loop to generate every thing, and then run...
    Simulation simulation(N, mode,unit_particle_length, dt, eta, B, alpha, rest_density, fold+output_file, false, with_viscosity, with_XSPH);

    for(int i=0;i<total_simulation;++i)
    {
        simulation.p_sphSimulator->update_simulation();

        if (i % step_size == 0)
            simulation.p_sphSimulator->update_sim_record_state();

        std::cout<<"iteration "<< i <<std::endl;
    }


    return 0;

}




