#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

#include "simulation.hpp"
#include <CLI11.hpp>
#include <string>

#include "SPHSimulator.hpp"

using namespace std;

////////////////////////////////////////////////////
std::atomic<bool> quit(false);    // signal flag
void got_signal(int)
{
    quit.store(true);
}
/////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    CLI::App CLIapp{"Cereal_Recorder"};

    // Define options
    float eta = 1.2f;
    CLIapp.add_option("-e, --eta", eta, "Eta: normally 1.0~1.5");

    float rest_density = 1000.0f;
    CLIapp.add_option("-d, --rest_density", rest_density, "Fluid Rest density: 1000 (kg/m^3) on water for instance");

    float B = 1000.0f;
    CLIapp.add_option("-s, --stiffness", B, "Stiffness of pressure force, the B");

    float dt = 0.01f;
    CLIapp.add_option("-t, --dt", dt, "Elapsed time");


    int step_size = 10;
    CLIapp.add_option("-z, --step_size", step_size, "record once every <step_size> frames");

    float alpha = 0.08f;
    CLIapp.add_option("-a, --alpha", alpha, "parameter of viscosity");


    int with_viscosity = 1;
    CLIapp.add_option("-v, --with_viscosity", with_viscosity, "add viscosity");

    int with_XSPH = 1;
    CLIapp.add_option("-x, --with_XSPH", with_XSPH, "use XSPH to update position");

    float unit_particle_length = 0.1f;
    CLIapp.add_option("-u, --unit_particle_length", unit_particle_length, " the intervel length between two particles per axis.");


    CLIapp.option_defaults()->required();

    int N;
    CLIapp.add_option("-n, --N", N, "Number of particles per edge");

    int mode;
    CLIapp.add_option("-m, --mode", mode, "Simulation mode: 1 for dam breaking | 2 for dropping the water from the center of boundary | 3 for free fall | 4 for 2-cube collision | 5 for thin dam break | 6 for double dam break | 7 for water drop on water | 8 for fluid pillar | 9 for drop water on the spherical boundary");

    int total_simulation;
    CLIapp.add_option("-f, --total_simulation_frame_number", total_simulation, "Number of simulations to record");

    std::string output_file;
    CLIapp.add_option("-o, --output_file", output_file, "path to output file");

    int solver_type;
    CLIapp.add_option("-c, --solver", solver_type, "Solver Type: 0 for WCSPH | 1 for PBF");

    try {
    	CLIapp.parse(argc, argv);
    } catch(const CLI::ParseError &e) {
        return CLIapp.exit(e);
    }
    //CLI11_PARSE(CLIapp, argc, argv);

    cout << "eta = "         				<< eta << endl;
    cout << "rest_density = "				<< rest_density << endl;
    cout << "stiffness = "					<< B << endl;
    cout << "elapsed time = " 				<< dt << endl;
    cout << "record step_size = "			<< step_size << endl;
    cout << "alpha = " 						<< alpha << endl;
    cout << "with_viscosity = " 			<< with_viscosity << endl;
    cout << "with_XSPH = " 					<< with_XSPH << endl;
    cout << "particles number per edge = " 	<< N << endl;
    cout << "mode = " 						<< mode << endl;
    cout << "total number of simulation frames = " << total_simulation << endl;
    cout << "unit_particle_length = " 		<< unit_particle_length << endl;
    cout << "output_file = " 				<< output_file << endl;
    if (solver_type == 0)
    	cout << "solver = WCSPH" << endl;
    else if (solver_type == 1)
    	cout << "solver = PBF" << endl;
    cout << endl;

    ////////////////////////////////////////////////////////////////
    // code allow we use ctrl-c to quit the simulation while it still store data.
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);
    //////////////////////////////////////////////////////////////////////////
    // a for loop to generate every thing, and then run...
    Simulation simulation(N, mode, unit_particle_length, dt, eta, B, alpha, rest_density, output_file, false, with_viscosity, with_XSPH, solver_type);

    for(int i=0;i<total_simulation;++i)
    {
        simulation.p_sphSimulator->update_simulation();

        if (i % step_size == 0){
            simulation.p_sphSimulator->update_sim_record_state();
            ////////////////////////////////////////////////////////////
            if( quit.load() ) break;    // exit normally after SIGINT
            ////////////////////////////////////////////////////////////
        }

        std::cout<<"iteration "<< i <<std::endl;
    }


    return 0;

}




