#pragma once

#include "Particle.hpp"
#include "math_types.hpp"

#include <vector>

using namespace Simulator;

class ParticleFunc {
public:

	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius );
	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& particles, Real radius );
	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<Real>& volumes, Real radius );

	void update_position( std::vector<mParticle>& particles, Real dt );
	void update_velocity( std::vector<mParticle>& particles, Real dt ); // semi-implicit euler
	void update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<Real>& densities, std::vector<RealVector3>& external_forces, Real rest_density, Real radius, Real B);
	void update_acceleration( std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<Real>& densities, std::vector<Real>& boundary_volumes, std::vector<RealVector3>& external_forces, Real rest_density, Real radius, Real B);

	std::vector<Real> initialize_boundary_particle_volumes(std::vector<RealVector3>& boundary_positions, Real neighbor_search_radius);

private:
	//pressure_force(mParticle p);
};
