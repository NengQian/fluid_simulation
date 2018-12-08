#pragma once

#include "Particle.hpp"
#include "math_types.hpp"

#include <vector>

using namespace Simulator;

class ParticleFunc {
public:
	ParticleFunc(Real rest_density, Real B, Real alpha);

	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius );
	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& particles, Real radius );
	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, Real radius);

	void update_position( std::vector<mParticle>& particles, Real dt ); // without XSPH
	void update_position( std::vector<mParticle>& particles, Real dt, std::vector<std::vector<size_t>>& neighbors_set, Real radius, std::vector<Real>& densities); // with XSPH

	void update_velocity( std::vector<mParticle>& particles, Real dt ); // semi-implicit euler
	void update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<Real>& densities, std::vector<RealVector3>& external_forces, Real radius);
	void update_acceleration( std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<Real>& densities, std::vector<RealVector3>& external_forces, Real radius, bool with_viscosity);

	void initialize_boundary_particle_volumes(std::vector<Real>& boundary_volumes, std::vector<RealVector3>& boundary_positions, Real neighbor_search_radius);

	//std::vector<std::vector<Real>> compute_viscosity(std::vector<mParticle>& particles, std::vector<Real>& densities, Real neighbor_search_radius);
	std::vector<Real> compute_viscosity(std::vector<mParticle>& particles, std::vector<Real>& densities, size_t idx_i, std::vector<size_t>& neighbors_of_i, Real neighbor_search_radius);

private:
	//pressure_force(mParticle p);
	Real rest_density;
	Real B;
	Real alpha;
};
