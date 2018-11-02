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
	const Real particle_radius = 0.1;

	SPHSimulator(float neighbor_search_radius);

    void 					 generate_particles();
    std::vector<RealVector3> get_particles();

    void   set_number_of_particles(size_t n);
    size_t get_number_of_particles();

    void  set_neighbor_search_radius(float r);
    float get_neighbor_search_radius();

    void set_index_of_source_particle(int idx);
    int  get_index_of_source_particle();

    void 			    set_neighbors(std::vector<size_t>& ns);
    std::vector<size_t> get_neighbors();

	void generate_random_particles();

	void find_and_set_neighbors(bool do_compactN);

	std::vector< std::vector<size_t> > find_neighbors_of_all(bool do_compactN);

	Real compute_average_error_of_kernel_gradient(int kernel_type);

private:
	NeighborSearcher neighborSearcher;
	KernelHandler 	 kernelHandler;

	std::vector<RealVector3> particles;
	size_t number_of_particles;

	float neighbor_search_radius;
    int index_of_source_particle;
    std::vector<size_t> neighbors;
};
