#pragma once

#include "Particle.hpp"
#include "math_types.hpp"

#include <vector>

using namespace Simulator;

class ParticleFunc {
public:
	//update_acceleration(mParticle p, Eigen::Ref<RealVector3> force);
	std::vector<Real> update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius );
	void update_position( std::vector<mParticle>& particles, Real dt );
	void update_velocity( std::vector<mParticle>& particles, Real dt ); // semi-implicit euler
private:
	//pressure_force(mParticle p);
};
