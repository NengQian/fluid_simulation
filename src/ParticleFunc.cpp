#include "ParticleFunc.hpp"
#include "math_types.hpp"
#include "KernelHandler.hpp"
#include <iostream>
#include <algorithm>

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

std::vector<Real> ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& particles, Real radius )
{
	std::vector<Real> densities;
	KernelHandler kh(radius);
	for (size_t i=0; i<neighbors_of_set.size(); ++i)
	{
		// add itself into neighbors list
		//neighbors_of_set[i].push_back(i);

		Real d = 0.0;
		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			Real m = particles[neighbors_of_set[i][j]].mass;
			d += m * kh.compute_kernel( particles[i].position, particles[neighbors_of_set[i][j]].position, 4 );
		}
		densities.push_back(d);

		//std::cout << "density of " << i << ": " << d << std::endl;
		// remove itself from neighbors list
		//neighbors_of_set[i].pop_back();
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

void ParticleFunc::update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<Real>& densities, std::vector<RealVector3>& external_forces, Real rest_density, Real radius, Real B)
{
	KernelHandler kh(radius);
	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 a;
		a << 0.0, 0.0, 0.0 ;
		RealVector3 a1;
		a1 << 0.0, 0.0, 0.0 ;
		RealVector3 a2;
		a2 << 0.0, 0.0, 0.0 ;

		//std::cout << "a1 before " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		// add itself
		//neighbors_of_set[i].push_back(i);

		Real d_i = densities[i];
		//std::cout << "density " << i << ": " << d_i << std::endl;

		//std::cout << "neighbor of " << i << ": " << neighbors_of_set[i].size() << std::endl;


		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			RealVector3 gradient = kh.gradient_of_kernel( particles[i].position, particles[neighbors_of_set[i][j]].position, 4 );
			//std::cout << "gradient (" << i << ", " << j << "): " << "(" << gradient[0] << " " << gradient[1] << " " << gradient[2] << ")" << std::endl;
			Real p_i, p_j, d_j, m_j;

			p_i = std::max(0.0, B * (densities[i] - rest_density));
			p_j = std::max(0.0, B * (densities[neighbors_of_set[i][j]] - rest_density));

			//std::cout << "pressure " << i << ": " << p_i << std::endl;
			//std::cout << "pressure " << n_idx << ": " << p_j << std::endl;

			d_j = densities[neighbors_of_set[i][j]];

			m_j = particles[neighbors_of_set[i][j]].mass;

			a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j));
		}
		//std::cout << "a1 after " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		a2 = external_forces[i] / d_i;
		a = a1 + a2;


		// deep copy
		particles[i].acceleration[0] = a[0];
		particles[i].acceleration[1] = a[1];
		particles[i].acceleration[2] = a[2];

		// remove itself
		//neighbors_of_set[i].pop_back();
	}
}



