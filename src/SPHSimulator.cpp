#include "SPHSimulator.hpp"

#include "math_types.hpp"

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

SPHSimulator::SPHSimulator(float neighbor_search_radius) : kernelHandler(static_cast<Real>(neighbor_search_radius)), neighborSearcher(static_cast<Real>(neighbor_search_radius))
{
	set_number_of_particles(10000);
	generate_particles();
	set_neighbor_search_radius(neighbor_search_radius);
	set_index_of_source_particle(0);
}

void SPHSimulator::generate_particles()
{
	generate_random_particles();
	neighborSearcher.set_particles_ptr(particles);
}

std::vector<RealVector3> SPHSimulator::get_particles()
{
	return particles;
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

void SPHSimulator::generate_random_particles()
{
	if (!particles.empty())
		particles.clear();

	// Set up random number generator
	std::default_random_engine random_number_generator;
	random_number_generator.seed(std::random_device()());
	std::uniform_real_distribution<Real> distribution(-9,9);

	for (size_t i = 0; i < number_of_particles; ++i)
	{
		const auto x = distribution(random_number_generator);
		const auto y = distribution(random_number_generator);
		const auto z = distribution(random_number_generator)+10.0;
		particles.push_back(RealVector3(x,y,z));
	}
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
		error += kernelHandler.test_gradient(particles[index_of_source_particle], particles[neighbors[i]], kernel_type);
	}

	return error/neighbors.size();
}

