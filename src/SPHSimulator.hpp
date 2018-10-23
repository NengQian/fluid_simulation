#pragma once

#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>

#include "math_types.hpp"
#include "NeighborSearcher.hpp"
#include "KernelHandler.hpp"

using namespace Simulator;

class SPHSimulator
{
public:
	SPHSimulator();

	std::vector<RealVector3> particles;
	size_t number_of_particles = 100;
	const Real particle_radius = 0.1;

    float neighbor_search_radius = 5.0;
    std::vector<size_t> neighbors;
    int index_of_source_particle = 0;

	NeighborSearcher neighborSearcher;
	KernelHandler kernelHandler;

	void generate_random_particles();

private:

};
