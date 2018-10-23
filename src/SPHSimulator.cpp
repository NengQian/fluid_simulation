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

SPHSimulator::SPHSimulator()
{

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


