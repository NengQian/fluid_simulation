/*
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


// need to generate cube first
void sample_density(SPHSimulator& s)
{
	std::vector<mParticle> sample_particles;
	std::vector<RealVector3> sample_positions;
	size_t number_of_samples = s.get_N()*10;

	for (size_t i=0; i<number_of_samples; ++i)
	{
		mParticle p;
		RealVector3 vec(-2.0 + i * 4.0 / number_of_samples, 0.0, 0.0);
		p.position = vec;
		sample_particles.push_back(p);
		sample_positions.push_back(vec);
	}

	s.set_neighbor_search_radius( 2.4/N * 2 );

	std::vector< std::vector<size_t> > neighbors_of_samples = neighborSearcher.find_neighbors_within_radius(sample_positions);

	std::vector<Real> densities;
	Real r = static_cast<Real>(neighbor_search_radius);
	densities = particleFunc.update_density(neighbors_of_samples, sample_particles, particles, r);

	for (size_t i=0; i<neighbors_of_samples.size(); ++i)
	{
		std::cout << sample_particles[i].position[0] << " " << densities[i] << std::endl;
	}
}

int main(void)
{
	int N = 100;
	SPHSimulator sim(0.01, N);
	sample_density(sim);
	return 0;
}
*/
