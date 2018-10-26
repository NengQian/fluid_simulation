#pragma once

#include "math_types.hpp"

#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>

using namespace Simulator;

class NeighborSearcher
{
public:
	NeighborSearcher();
	NeighborSearcher(Real radius);

	void set_neighbor_search_radius(Real radius);
	void set_particles_ptr(std::vector<RealVector3>& particles);

	std::vector<size_t> find_neighbors_within_radius( size_t selected_particle_index, bool use_compactN );
	std::vector<size_t> compactN_neighbor_search( size_t selected_particle_index );
	std::vector<size_t> brute_force_neighbor_search( size_t selected_particle_index );
	void brute_force_search( std::vector<std::vector<int> >& n_neighbors_indices );

	std::vector<std::array<CompactNSearch::Real, 3>> convect_to_CompactN_position();
private:
	std::shared_ptr<std::vector<RealVector3>> particles_ptr;
	Real neighbor_search_radius;
};
