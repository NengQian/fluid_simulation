#include "ParticleFunc.hpp"
#include "math_types.hpp"
#include "KernelHandler.hpp"
#include "NeighborSearcher.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace Simulator;

ParticleFunc::ParticleFunc(Real rest_density, Real B, Real alpha) : rest_density(rest_density), B(B), alpha(alpha)
{

}

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

std::vector<Real> ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<Real>& volumes, Real radius )
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

		RealVector3 p_i = particles[i].position;

		for (size_t k=0; k<neighbors_in_boundary[i].size(); ++k)
		{
			Real V_k = volumes[neighbors_in_boundary[i][k]];
			RealVector3 bp_k = boundary_particles[neighbors_in_boundary[i][k]].position;
			d += rest_density * V_k * kh.compute_kernel( p_i, bp_k, 4 );
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

// without XSPH
void ParticleFunc::update_position( std::vector<mParticle>& particles, Real dt )
{
	for (auto& p : particles)
	{
		p.position += p.velocity * dt;
	}
}


// with XSPH
void ParticleFunc::update_position( std::vector<mParticle>& particles, Real dt, std::vector<std::vector<size_t>>& neighbors_set, Real radius, std::vector<Real>& densities)
{
	KernelHandler kh(radius);

	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 sum(0.0, 0.0, 0.0);
		mParticle p_i = particles[i];

		for (size_t k=0; k<neighbors_set[i].size(); ++k)
		{
			size_t j = neighbors_set[i][k];
			mParticle p_j = particles[j];

			sum += 2.0 * p_j.mass / (densities[i] + densities[j]) * kh.compute_kernel(p_i.position, p_j.position, 4) * (p_j.velocity - p_i.velocity);
		}

		RealVector3 v_i_star = p_i.velocity + 0.5 * sum;

		particles[i].position += v_i_star * dt;
	}
}


void ParticleFunc::update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<Real>& densities, std::vector<RealVector3>& external_forces, Real radius)
{
	KernelHandler kh(radius);
	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 a(0.0, 0.0, 0.0);
		RealVector3 a1(0.0, 0.0, 0.0);
		RealVector3 a2(0.0, 0.0, 0.0);

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
        //a2 = external_forces[i]/particles[i].mass;
		a = a1 + a2;


		// deep copy
		particles[i].acceleration = a;

		// remove itself
		//neighbors_of_set[i].pop_back();
	}
}

void ParticleFunc::update_acceleration( std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<Real>& densities, std::vector<Real>& boundary_volumes, std::vector<RealVector3>& external_forces, Real radius, bool with_viscosity)
{
	KernelHandler kh(radius);

	std::vector<std::vector<Real>> viscosity;
	if (with_viscosity)
		viscosity = compute_viscosity(particles, densities, radius/2.0);


	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 a(0.0, 0.0, 0.0);
		RealVector3 a1(0.0, 0.0, 0.0);
		RealVector3 a2(0.0, 0.0, 0.0);
		RealVector3 a3(0.0, 0.0, 0.0);

		//std::cout << "a1 before " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;


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

			if (with_viscosity)
				a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j) + viscosity[i][j]);
			else
				a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j));

		}
		//std::cout << "a1 after " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		for (size_t k=0; k<neighbors_in_boundary[i].size(); ++k)
		{
			RealVector3 gradient = kh.gradient_of_kernel( particles[i].position, boundary_particles[neighbors_in_boundary[i][k]].position, 4 );
			//std::cout << "gradient (" << i << ", " << j << "): " << "(" << gradient[0] << " " << gradient[1] << " " << gradient[2] << ")" << std::endl;
			Real p_i;

			p_i = std::max(0.0, B * (densities[i] - rest_density));

			//std::cout << "pressure " << i << ": " << p_i << std::endl;
			//std::cout << "pressure " << n_idx << ": " << p_j << std::endl;


			a2 -= boundary_volumes[neighbors_in_boundary[i][k]] * rest_density * gradient * (p_i / (d_i * d_i));
		}


        //a3 = external_forces[i] / d_i;
        a3 = external_forces[i] / particles[i].mass;

		a = a1 + a2 + a3;


		// deep copy
		particles[i].acceleration = a;

		// remove itself
		//neighbors_of_set[i].pop_back();
	}
}

std::vector<Real> ParticleFunc::initialize_boundary_particle_volumes(std::vector<RealVector3>& boundary_positions, Real neighbor_search_radius)
{
	std::vector<Real> volumes;
//    if(boundary_positions.size()==0)  // if we don't need to generate volume
//        return volumes;

	NeighborSearcher nb(neighbor_search_radius);
	nb.set_boundary_particles_ptr(boundary_positions);

	KernelHandler kh(neighbor_search_radius);

	std::vector< std::vector<size_t> > neighbors_of_boundary = nb.find_boundary_neighbors( );


	for (size_t k=0; k<boundary_positions.size(); ++k)
	{
		Real V_k = 0.0;
		RealVector3 bp_k = boundary_positions[k];
		for (size_t l=0; l<neighbors_of_boundary[k].size(); ++l)
		{
			RealVector3 bp_l = boundary_positions[neighbors_of_boundary[k][l]];
			V_k += kh.compute_kernel( bp_k, bp_l, 4 );
			//std::cout << "k: " << kh.compute_kernel( bp_k, bp_l, 4 ) << std::endl;
		}
		volumes.push_back(1.0/V_k);
	}

	return volumes;
}

std::vector<std::vector<Real>> ParticleFunc::compute_viscosity(std::vector<mParticle>& particles, std::vector<Real>& densities, Real neighbor_search_radius)
{
	std::vector<std::vector<Real>> vs;

	size_t l = particles.size();
	for (size_t i=0; i<l; ++i)
	{
		std::vector<Real> v_i;
		for (size_t j=0; j<l; ++j)
		{
			RealVector3 v_ij = particles[i].velocity - particles[j].velocity;
			RealVector3 x_ij = particles[i].position - particles[j].position;

			Real dotProduct = v_ij[0] * x_ij[0] + v_ij[1] * x_ij[1] + v_ij[2] * x_ij[2];

			if (dotProduct >= 0.0)
				v_i.push_back(0.0);
			else {
				Real u_ij = 2.0 * alpha * neighbor_search_radius * sqrt(B) / (densities[i] + densities[j]);
				Real squaredNorm_x_ij = x_ij[0] * x_ij[0] + x_ij[1] * x_ij[1] + x_ij[2] * x_ij[2];

				Real v_ij = -u_ij * dotProduct / (squaredNorm_x_ij + 0.01 * neighbor_search_radius * neighbor_search_radius);
				v_i.push_back(v_ij);
			}
		}

		vs.push_back(v_i);
	}

	return vs;
}

