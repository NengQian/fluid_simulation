#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include <SPHSimulator.hpp>
#include "simulation.hpp"
using namespace std;

std::string fold = "./sim_result/";

void generate_sim_rec()  // maybe I should directly put all particles in all frame to one vector...
{
    // a for loop to generate every thing, and then run...
    SPHSimulator test_simulator(5);
    int step_size = 10;

    for(int i=0;i<15000;++i)
    {
        test_simulator.update_rigid_body_simulation();

        if (i % step_size == 0)
        	test_simulator.update_sim_record_state();

        std::cout<<"iteration "<< i <<std::endl;
    }

    test_simulator.output_sim_record_bin(fold+"two_cubes_dt_fuck.bin");

    return;
}



int main()
{

    generate_sim_rec(); 


    return 0;

}




