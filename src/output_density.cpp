#include <memory>
#include <iostream>
#include <algorithm>
#include <CLI11.hpp>
#include <time.h>

#include "math_types.hpp"
#include "sim_record.hpp"

#include <cereal/types/vector.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>

using Simulator::Real;
using std::chrono::duration;

int main(int argc, char* argv[])
{

	CLI::App CLIapp{"output density of serialized simulation data"};

	CLIapp.option_defaults()->required();

	std::string file;
	CLIapp.add_option("-i, --input_file", file, "path to input serialized simulation file");

	try {
		CLIapp.parse(argc, argv);
	} catch(const CLI::ParseError &e) {
		return CLIapp.exit(e);
	}

    SimulationRecord sim_record;

    std::ifstream is(file);
    cereal::BinaryInputArchive infile(is);
    infile(sim_record);

    int total_frame = sim_record.states.size();
    std::vector<double> avg_density(total_frame, 0.0);
    std::vector<double> max_density(total_frame, 0.0);

    for (int i=0; i<sim_record.states.size(); ++i)
    {
        auto state = sim_record.states[i];
        int particle_number = state.particles.size();

        for (int j=0; j<particle_number; ++j)
        {
            auto p = state.particles[j];
            double d = p.density;
            
            avg_density[i] += d;

            if (d > max_density[i])
                max_density[i] = d;
        }

        avg_density[i] /= static_cast<double>(particle_number);
    }

    std::cout << "avg max" << std::endl;
    
    for (int i=0; i<total_frame; ++i)
    {
        std::cout << avg_density[i] << " " << max_density[i] << std::endl;
    }

    return 0;
}
