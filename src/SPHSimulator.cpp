#include "SPHSimulator.hpp"

#include "math_types.hpp"
#include "Particle.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <random>

using merely3d::renderable;
using merely3d::Rectangle;
using merely3d::Box;
using merely3d::Line;
using merely3d::Color;
using merely3d::Material;
using merely3d::Sphere;
using merely3d::red;
using merely3d::Particle;

using Eigen::AngleAxisf;
using Eigen::Vector3f;

SPHSimulator::SPHSimulator(float neighbor_search_radius, Real dt) : dt(dt), kernelHandler(static_cast<Real>(neighbor_search_radius)), neighborSearcher(static_cast<Real>(neighbor_search_radius))
{
	set_N(50);
	set_particle_radius(1.0/N);
	generate_particles();
	set_neighbor_search_radius(neighbor_search_radius);
	set_index_of_source_particle(0);
}

void SPHSimulator::generate_particles()
{
	//generate_random_particles();
	generate_celling_particles_at_center();
	neighborSearcher.set_particles_ptr(positions);
}

std::vector<RealVector3> SPHSimulator::get_positions()
{
	update_positions();
	return positions;
}

void SPHSimulator::set_particle_radius(Real r)
{
	particle_radius = r;
}

Real SPHSimulator::get_particle_radius()
{
	return particle_radius;
}

void   SPHSimulator::set_N(size_t n)
{
	N = n;
	set_number_of_particles(n * n * n);
}

size_t SPHSimulator::get_N()
{
	return N;
}

void SPHSimulator::set_number_of_particles(size_t n)
{
	number_of_particles = n;
}

size_t SPHSimulator::get_number_of_particles()
{
	return number_of_particles;
}

void SPHSimulator::set_neighbor_search_radius(float r)
{
	neighbor_search_radius = r;
	Real rr = static_cast<Real>(r);
	neighborSearcher.set_neighbor_search_radius(rr);
	kernelHandler.set_neighbor_search_radius(rr);
}

float SPHSimulator::get_neighbor_search_radius()
{
	return neighbor_search_radius;
}

void SPHSimulator::set_index_of_source_particle(int idx)
{
	index_of_source_particle = idx;
}

int SPHSimulator::get_index_of_source_particle()
{
	return index_of_source_particle;
}

void SPHSimulator::set_neighbors(std::vector<size_t>& ns)
{
	neighbors = ns;
}

std::vector<size_t> SPHSimulator::get_neighbors()
{
	return neighbors;
}

void SPHSimulator::find_and_set_neighbors(bool do_compactN)
{
	neighbors = neighborSearcher.find_neighbors_within_radius( index_of_source_particle, do_compactN );
}

std::vector< std::vector<size_t> > SPHSimulator::find_neighbors_of_all(bool do_compactN)
{
	return neighborSearcher.find_neighbors_within_radius( do_compactN );
}


Real SPHSimulator::compute_average_error_of_kernel_gradient(int kernel_type)
{
	Real error = 0;
	for (size_t i=0; i < neighbors.size(); ++i)
	{
		error += kernelHandler.test_gradient(positions[index_of_source_particle], positions[neighbors[i]], kernel_type);
	}

	return error/neighbors.size();
}

void SPHSimulator::sample_density()
{
	std::vector<mParticle> sample_particles;
	std::vector<RealVector3> sample_positions;
	size_t number_of_samples = N*10;

	for (size_t i=0; i<number_of_samples; ++i)
	{
		mParticle p;
		RealVector3 vec(-2.0 + i * 4.0 / number_of_samples, 0.0, 0.0);
		p.position = vec;
		sample_particles.push_back(p);
		sample_positions.push_back(vec);
	}

	set_neighbor_search_radius( 2.4/N * 2 );

	std::vector< std::vector<size_t> > neighbors_of_samples = neighborSearcher.find_neighbors_within_radius(sample_positions);

	std::vector<Real> densities;
	Real r = static_cast<Real>(neighbor_search_radius);
	densities = particleFunc.update_density(neighbors_of_samples, sample_particles, particles, r);

	for (size_t i=0; i<neighbors_of_samples.size(); ++i)
	{
		std::cout << sample_particles[i].position[0] << " " << densities[i] << std::endl;
	}
}

/*
void SPHSimulator::generate_random_particles()
{
	if (!particles.empty())
		particles.clear();

	// Set up random number generator
	std::default_random_engine random_number_generator;
	std::random_device rd;
	random_number_generator.seed(rd());
	std::uniform_real_distribution<Real> distribution(-1,1);

	for (size_t i = 0; i < number_of_particles; ++i)
	{
		const auto x = distribution(random_number_generator);
		const auto y = distribution(random_number_generator);
		const auto z = distribution(random_number_generator);
		particles.push_back(RealVector3(x,y,z));
	}
}
*/
/*
void SPHSimulator::randomly_generate_celling_particles()
{
	if (!particles.empty())
		particles.clear();

	// Set up random number generator
	std::default_random_engine random_number_generator;
	std::random_device rd;
	random_number_generator.seed(rd());
	std::uniform_real_distribution<Real> distribution(0.0, 1.0);

	Real step_size = 2.0/N;
	for (size_t i = 0; i < number_of_particles; ++i)
	{
		Real x = distribution(random_number_generator) * step_size;
		Real y = distribution(random_number_generator) * step_size;
		Real z = distribution(random_number_generator) * step_size;
		particles.push_back(RealVector3(x,y,z));
	}


	size_t idx = 0;
	for (Real i = -1.0; i < 1.0; i+=step_size)
	{
		for (Real j = -1.0; j < 1.0; j+=step_size)
		{
			for (Real k = -1.0; k < 1.0; k+=step_size)
			{
				particles[idx][0] += i;
				particles[idx][1] += j;
				particles[idx][2] += k;

				++idx;
			}
		}
	}
}
*/
void SPHSimulator::generate_celling_particles_at_center()
{
	if (!particles.empty())
		particles.clear();

	if (!positions.empty())
		positions.clear();

	Real step_size = 2.0/N;
	size_t idx = 0;
	for (Real i = -1.0; i < 1.0; i+=step_size)
	{
		for (Real j = -1.0; j < 1.0; j+=step_size)
		{
			for (Real k = -1.0; k < 1.0; k+=step_size)
			{
				mParticle p;

				Real x = i + step_size/2.0;
				Real y = j + step_size/2.0;
				Real z = k + step_size/2.0;

				p.position = RealVector3(x,y,z);
				p.mass = step_size * step_size * step_size * 1000.0;

				particles.push_back(p);
				positions.push_back(p.position);
				++idx;
			}
		}
	}
}

void SPHSimulator::update_positions()
{
	for (size_t i=0; i<particles.size(); ++i)
	{
		positions[i] = particles[i].position;
	}
}

void SPHSimulator::update_freefall_motion()
{
	for (auto& p : particles)
	{
		p.acceleration = gravity;
	}
	particleFunc.update_velocity(particles, dt);
	particleFunc.update_position(particles, dt);

	update_positions();
}
