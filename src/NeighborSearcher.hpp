#pragma once

#include "math_types.hpp"

#include <Eigen/Geometry>
#include <CompactNSearch/CompactNSearch>
#include <iostream>
#include <memory>


using namespace Simulator;

class NeighborSearcher
{
public:
	NeighborSearcher();
	NeighborSearcher(Real radius);

	void set_neighbor_search_radius(Real radius);
	void set_particles_ptr(std::vector<RealVector3>& particles);
	void set_boundary_particles_ptr(std::vector<RealVector3>& boundary_particles);

	std::vector<size_t> 			   find_neighbors_within_radius( size_t selected_particle_index, bool use_compactN );
	std::vector< std::vector<size_t> > find_neighbors_within_radius( std::vector<size_t> point_set );
	std::vector< std::vector<size_t> > find_neighbors_within_radius( bool use_compactN );
	std::vector< std::vector<size_t> > find_neighbors_within_radius( std::vector<RealVector3>& point_set );

	std::vector< std::vector<size_t> > find_boundary_neighbors( );
	std::vector< std::vector<size_t> > find_neighbors_in_boundary( );

private:
    std::shared_ptr<std::vector<RealVector3>> particles_ptr;
    std::shared_ptr<std::vector<RealVector3>> boundary_particles_ptr;
	Real neighbor_search_radius;

	std::vector<size_t> 			   compactN_neighbor_search( size_t selected_particle_index );
	std::vector< std::vector<size_t> > compactN_neighbor_search();

	std::vector<size_t> 			   brute_force_neighbor_search( size_t selected_particle_index );
	std::vector< std::vector<size_t> > brute_force_neighbor_search();

	std::vector<std::array<CompactNSearch::Real, 3>> convect_to_CompactN_position();
	std::vector<std::array<CompactNSearch::Real, 3>> convect_to_CompactN_position( std::vector<RealVector3>& point_set );

};
