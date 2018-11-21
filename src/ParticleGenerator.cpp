#include "ParticleGenerator.hpp"

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

void ParticleGenerator::generate_cube(std::vector<mParticle>& particles, size_t N, Real halfExtent, Eigen::Ref<RealVector3> origin, Eigen::Ref<RealVector3> v0, bool do_clear, bool hollow)
{
	if (do_clear)
		if (!particles.empty())
			particles.clear();

	Real step_size = 2.0 * halfExtent / N;
	for (Real i = -halfExtent; i < halfExtent; i+=step_size)
	{
		for (Real j = -halfExtent; j < halfExtent; j+=step_size)
		{
			for (Real k = -halfExtent; k < halfExtent; k+=step_size)
			{
				if (hollow)
					if ((i != -halfExtent && i != halfExtent-step_size ) && (j != -halfExtent && j != halfExtent-step_size ) && (k != -halfExtent && k != halfExtent-step_size ))
						continue;

				mParticle p;

				Real x = i + step_size/2.0 + origin[0];
				Real y = j + step_size/2.0 + origin[1];
				Real z = k + step_size/2.0 + origin[2];

				p.position = RealVector3(x,y,z);
				p.mass = step_size * step_size * step_size * 1000.0;

				p.velocity[0] = v0[0];
				p.velocity[1] = v0[1];
				p.velocity[2] = v0[2];

				particles.push_back(p);
			}
		}
	}
}

void ParticleGenerator::generate_two_freefall_cubes(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.0, 0.0, 0.0);
	RealVector3 v2_init(0.0, 0.0, 0.0);

	generate_cube(particles, N, 1.0, o1, v1_init, false, false);
	generate_cube(particles, N, 1.0, o2, v2_init, false, false);
}


void ParticleGenerator::generate_two_colliding_cubes(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.25, 0.0, 0.0);
    RealVector3 v2_init(0.00, 0.0, 0.0);

	generate_cube(particles, N, 1.0, o1, v1_init, false, false);
	generate_cube(particles, N, 1.0, o2, v2_init, false, false);
}

void ParticleGenerator::generate_rigid_box(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o(0.0, 0.0, 0.0);
	RealVector3 v_init(0.0, 0.0, 0.0);

	generate_cube(particles, N, 3.0, o, v_init, false, true);
}
