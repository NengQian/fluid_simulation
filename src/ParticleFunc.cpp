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

void ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& samples, std::vector<mParticle>& particles, Real radius )
{
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
		particles[i].density = d;
	}
}

void ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<mParticle>& particles, Real radius )
{
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

		particles[i].density = d;
	}
}

void ParticleFunc::update_density(std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, Real radius )
{
	KernelHandler kh(radius);
	for (size_t i=0; i<neighbors_of_set.size(); ++i)
	{
		Real d = 0.0;
		RealVector3 p_i = particles[i].position;

		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			size_t k = neighbors_of_set[i][j];
			mParticle P_k = particles[k];
			Real m = P_k.mass;
			d += m * kh.compute_kernel( p_i, P_k.position, 4 );
		}

		for (size_t k=0; k<neighbors_in_boundary[i].size(); ++k)
		{
			size_t l = neighbors_in_boundary[i][k];
			Real m = boundary_particles[l].mass;
			RealVector3 bp_k = boundary_particles[l].position;
			d += m * kh.compute_kernel( p_i, bp_k, 4 );
		}

		particles[i].density = d;
	}
}

void ParticleFunc::update_boundary_position( std::vector<mParticle>& boundary_particles, int start_idx, Real mid, Real amp, Real dt, int iter ) // without XSPH
{
	if (dt * iter <= 1.0) return;
	for (int i=start_idx; i<boundary_particles.size(); ++i)
	{
		boundary_particles[i].position[1] = mid + amp * (cos(0.5*M_PI*(dt*iter-1.0)));
	}
}

void ParticleFunc::update_velocity( std::vector<mParticle>& particles, Real dt, Eigen::Ref<const RealVector3> a )
{
	for (auto& p : particles)
	{
		p.velocity += a * dt;
	}
}

void ParticleFunc::update_velocity( std::vector<mParticle>& particles, Real dt, std::vector<RealVector3>& as )
{
	for (size_t i=0; i<particles.size(); ++i)
	{
		particles[i].velocity += as[i] * dt;
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
void ParticleFunc::update_position( std::vector<mParticle>& particles, Real dt, std::vector<std::vector<size_t>>& neighbors_set, Real radius)
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

			sum += 2.0 * p_j.mass / (p_i.density + p_j.density) * kh.compute_kernel(p_i.position, p_j.position, 4) * (p_j.velocity - p_i.velocity);
		}

		RealVector3 v_i_star = p_i.velocity + 0.5 * sum;

		particles[i].position += v_i_star * dt;
	}
}


std::vector<RealVector3> ParticleFunc::update_acceleration( std::vector<mParticle>& particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<RealVector3>& external_forces, Real radius)
{
	std::vector<RealVector3> as;

	KernelHandler kh(radius);

	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 a(0.0, 0.0, 0.0);
		RealVector3 a1(0.0, 0.0, 0.0);
		RealVector3 a2(0.0, 0.0, 0.0);

		//std::cout << "a1 before " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		// add itself
		//neighbors_of_set[i].push_back(i);

		Real d_i = particles[i].density;
		//std::cout << "density " << i << ": " << d_i << std::endl;

		//std::cout << "neighbor of " << i << ": " << neighbors_of_set[i].size() << std::endl;


		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			RealVector3 gradient = kh.gradient_of_kernel( particles[i].position, particles[neighbors_of_set[i][j]].position, 4 );
			//std::cout << "gradient (" << i << ", " << j << "): " << "(" << gradient[0] << " " << gradient[1] << " " << gradient[2] << ")" << std::endl;
			Real p_i, p_j, d_j, m_j;

			d_j = particles[neighbors_of_set[i][j]].density;

			p_i = std::max(0.0, B * (d_i - rest_density));
			p_j = std::max(0.0, B * (d_j - rest_density));

			//std::cout << "pressure " << i << ": " << p_i << std::endl;
			//std::cout << "pressure " << n_idx << ": " << p_j << std::endl;

			m_j = particles[neighbors_of_set[i][j]].mass;

			a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j));
		}
		//std::cout << "a1 after " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;



        a2 = external_forces[i] / d_i;
        //a2 = external_forces[i]/particles[i].mass;
		a = a1 + a2;


		// deep copy
		as.push_back(a);

		// remove itself
		//neighbors_of_set[i].pop_back();
	}

	return as;
}

std::vector<RealVector3> ParticleFunc::update_acceleration( std::vector<mParticle>& particles, std::vector<mParticle>& boundary_particles, std::vector<std::vector<size_t>>& neighbors_of_set, std::vector<std::vector<size_t>>& neighbors_in_boundary, std::vector<RealVector3>& external_forces, Real radius, bool with_viscosity)
{
	std::vector<RealVector3> as;

	KernelHandler kh(radius);

	/*
	std::vector<std::vector<Real>> viscosity;
	if (with_viscosity)
		viscosity = compute_viscosity(particles, densities, radius);
	*/

	for (size_t i=0; i<particles.size(); ++i)
	{
		RealVector3 a(0.0, 0.0, 0.0);
		RealVector3 a1(0.0, 0.0, 0.0);
		RealVector3 a2(0.0, 0.0, 0.0);
		RealVector3 a3(0.0, 0.0, 0.0);

		//std::cout << "a1 before " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		mParticle Pi = particles[i];
		Real d_i = particles[i].density;
		Real p_i = std::max(0.0, B * (d_i - rest_density));
		//std::cout << "density " << i << ": " << d_i << std::endl;

		//std::cout << "neighbor of " << i << ": " << neighbors_of_set[i].size() << std::endl;


		/*--------- compute viscosity of i -------*/
		std::vector<Real> v_i;
		if (with_viscosity)
			v_i = compute_viscosity(particles, i, neighbors_of_set[i], radius);

		for (size_t j=0; j<neighbors_of_set[i].size(); ++j)
		{
			size_t idx_n = neighbors_of_set[i][j];
			mParticle Pj = particles[idx_n];
			RealVector3 gradient = kh.gradient_of_kernel( Pi.position, Pj.position, 4 );
			//std::cout << "gradient (" << i << ", " << j << "): " << "(" << gradient[0] << " " << gradient[1] << " " << gradient[2] << ")" << std::endl;
			Real p_j, d_j, m_j;

			d_j = particles[idx_n].density;
			p_j = std::max(0.0, B * (d_j - rest_density));

			//std::cout << "pressure " << i << ": " << p_i << std::endl;
			//std::cout << "pressure " << n_idx << ": " << p_j << std::endl;


			m_j = Pj.mass;

			if (with_viscosity)
				a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j) + v_i[j]);
			else
				a1 -= gradient * m_j * (p_i / (d_i * d_i) + p_j / (d_j * d_j));

		}
		//std::cout << "a1 after " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;

		for (size_t k=0; k<neighbors_in_boundary[i].size(); ++k)
		{
			size_t idx_nb = neighbors_in_boundary[i][k];
			mParticle BPk = boundary_particles[idx_nb];
			RealVector3 gradient = kh.gradient_of_kernel( Pi.position, BPk.position, 4 );
			//std::cout << "gradient (" << i << ", " << j << "): " << "(" << gradient[0] << " " << gradient[1] << " " << gradient[2] << ")" << std::endl;
			//std::cout << "pressure " << i << ": " << p_i << std::endl;
			//std::cout << "pressure " << n_idx << ": " << p_j << std::endl;

			Real mb = BPk.mass;

			a2 -= mb * gradient * (p_i / (d_i * d_i));
		}


        //a3 = external_forces[i] / d_i;
        a3 = external_forces[i] / particles[i].mass;

		//std::cout << "a1 after " << i << ": (" << a1[0] << " " << a1[1] << " " << a1[2] << ")" << std::endl;
		//std::cout << "a2 after " << i << ": (" << a2[0] << " " << a2[1] << " " << a2[2] << ")" << std::endl;
		//std::cout << "a3 after " << i << ": (" << a3[0] << " " << a3[1] << " " << a3[2] << ")" << std::endl;


		a = a1 + a2 + a3;


		// deep copy
		as.push_back(a);

		// remove itself
		//neighbors_of_set[i].pop_back();
	}

	return as;
}

void ParticleFunc::initialize_boundary_particle_volumes(std::vector<Real>& boundary_volumes, std::vector<RealVector3>& boundary_positions, Real neighbor_search_radius)
{
//    if(boundary_positions.size()==0)  // if we don't need to generate volume
//        return volumes;

	if (!boundary_volumes.empty())
		boundary_volumes.clear();

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
		boundary_volumes.push_back(1.0/V_k);
	}
}

// most time-costly function: take 20% of the whole time
// complexity: O(n^2), n : number of particles,
// for example, when N is 10, we have 1K particles => the matrix has size 1k * 1k * sizeof(Real) = 8GB
/*
std::vector<std::vector<Real>> ParticleFunc::compute_viscosity(std::vector<mParticle>& particles, std::vector<Real>& densities, Real neighbor_search_radius)
{
	std::vector<std::vector<Real>> vs;

	size_t l = particles.size();
	for (size_t i=0; i<l; ++i)
	{
		std::vector<Real> v_i;
		mParticle Pi = particles[i];
		for (size_t j=0; j<l; ++j)
		{
			mParticle Pj = particles[j];

			RealVector3 v_ij = Pi.velocity - Pj.velocity;
			RealVector3 x_ij = Pi.position - Pj.position;

			Real dotProduct = v_ij[0] * x_ij[0] + v_ij[1] * x_ij[1] + v_ij[2] * x_ij[2];

			if (dotProduct >= 0.0)
				v_i.push_back(0.0);
			else {
				Real h = neighbor_search_radius / 2.0; // assume we use m4 kernel
				Real u_ij = 2.0 * alpha * h * sqrt(B) / (densities[i] + densities[j]);
				Real squaredNorm_x_ij = x_ij[0] * x_ij[0] + x_ij[1] * x_ij[1] + x_ij[2] * x_ij[2];

				Real v_ij = -u_ij * dotProduct / (squaredNorm_x_ij + 0.01 * h * h);
				v_i.push_back(v_ij);
			}
		}

		vs.push_back(v_i);
	}

	return vs;
}
*/

// Instead of computing the whole viscosity matrix, we compute one row each time
std::vector<Real> ParticleFunc::compute_viscosity(std::vector<mParticle>& particles, size_t idx_i, std::vector<size_t>& neighbors_of_i, Real neighbor_search_radius)
{
	std::vector<Real> v_i;

	mParticle Pi = particles[idx_i];
	for (size_t k=0; k<neighbors_of_i.size(); ++k)
	{
		size_t j = neighbors_of_i[k];
		mParticle Pj = particles[j];

		RealVector3 v_ij = Pi.velocity - Pj.velocity;
		RealVector3 x_ij = Pi.position - Pj.position;

		Real dotProduct = v_ij[0] * x_ij[0] + v_ij[1] * x_ij[1] + v_ij[2] * x_ij[2];

		if (dotProduct >= 0.0)
			v_i.push_back(0.0);
		else {
			Real h = neighbor_search_radius / 2.0; // assume we use m4 kernel
			Real u_ij = 2.0 * alpha * h * sqrt(B) / (particles[idx_i].density + particles[j].density);
			Real squaredNorm_x_ij = x_ij[0] * x_ij[0] + x_ij[1] * x_ij[1] + x_ij[2] * x_ij[2];

			Real v_ij = -u_ij * dotProduct / (squaredNorm_x_ij + 0.01 * h * h);
			v_i.push_back(v_ij);
		}
	}

	return v_i;
}

