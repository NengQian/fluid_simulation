#include "ParticleFunc.hpp"
#include "math_types.hpp"
#include "KernelHandler.hpp"
#include <iostream>

using namespace Simulator;

std::vector<Real> ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius )
{
	std::vector<Real> densities;
	KernelHandler kh(radius);
	for (size_t i=0; i<neighbors_of_set.size(); ++i)
	{
		Real d = 0.0;
		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			Real m = particles[neighbors_of_set[i][j]].mass;
			std::cout << m << std::endl;
			d += m * kh.compute_kernel( samples[i].position, particles[neighbors_of_set[i][j]].position, 4 );
		}
		densities.push_back(d);
	}
	return densities;
}

void ParticleFunc::update_velocity( std::vector<mParticle>& particles, Real dt )
{
	for (auto& p : particles)
	{
		p.velocity += p.acceleration * dt;
	}
}

void ParticleFunc::update_position( std::vector<mParticle>& particles, Real dt )
{
	for (auto& p : particles)
	{
		p.position += p.velocity * dt;
	}
}




