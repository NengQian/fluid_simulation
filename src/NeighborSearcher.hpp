#pragma once

#include <math_types.hpp>

#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>

using namespace Simulator;

class NeighborSearcher
{
public:
	NeighborSearcher();
	std::vector<size_t> find_neighbors_within_radius(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius, bool use_compactN);
	std::vector<size_t> compactN_neighbor_search(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius);
	std::vector<size_t> brute_force_neighbor_search(std::vector<RealVector3> &particles, size_t selected_particle_index, double radius);
	void brute_force_search(std::vector<RealVector3> &particles, std::vector<std::vector<int> >& n_neighbors_indices, double radius);

	std::vector<std::array<CompactNSearch::Real, 3>> convect_to_CompactN_position(std::vector<RealVector3> &particles);
private:

};
