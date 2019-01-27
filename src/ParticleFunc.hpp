#pragma once

#include "Particle.hpp"
#include "math_types.hpp"

#include <vector>

using namespace Simulator;

class ParticleFunc {
public:
	ParticleFunc(Real rest_density, Real B, Real alpha);

	void update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius );
	void update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& particles, Real radius );
	void update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, Real radius);

	void update_position( std::vector<mParticle>& particles, Real dt ); // without XSPH
	void update_position( std::vector<mParticle>& particles, Real dt, std::vector<std::vector<size_t>>& neighbors_set, Real radius); // with XSPH

	void update_boundary_position( std::vector<mParticle>& boundary_particles, int start_idx, Real mid, Real amp, int iter ); // without XSPH

	void update_velocity( std::vector<mParticle>& particles, Real dt, Eigen::Ref<const RealVector3> a); // semi-implicit euler
	void update_velocity( std::vector<mParticle>& particles, Real dt, std::vector<RealVector3>& as);

	std::vector<RealVector3> update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<RealVector3>& external_forces, Real radius);
	std::vector<RealVector3> update_acceleration( std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<RealVector3>& external_forces, Real radius, bool with_viscosity);

	void initialize_boundary_particle_volumes(std::vector<Real>& boundary_volumes, std::vector<RealVector3>& boundary_positions, Real neighbor_search_radius);

	//std::vector<std::vector<Real>> compute_viscosity(std::vector<mParticle>& particles, std::vector<Real>& densities, Real neighbor_search_radius);
	std::vector<Real> compute_viscosity(std::vector<mParticle>& particles, size_t idx_i, std::vector<size_t>& neighbors_of_i, Real neighbor_search_radius);

private:
	//pressure_force(mParticle p);
	Real rest_density;
	Real B;
	Real alpha;
};
