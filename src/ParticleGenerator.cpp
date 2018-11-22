#include "ParticleGenerator.hpp"

#include "math_types.hpp"
#include "Particle.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <random>
#include <cmath>

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

void ParticleGenerator::generate_cube(std::vector<mParticle>& particles, size_t N, Eigen::Ref<RealVector3> origin, Eigen::Ref<RealVector3> v0, Eigen::Ref<RealVector3> a0, Real halfExtent, bool do_clear, bool hollow)
{
	if (do_clear)
		if (!particles.empty())
			particles.clear();

	Real step_size = 2.0 * halfExtent / N;

	int idx_i = 0;

	for (Real i = -halfExtent; i < halfExtent; i+=step_size)
	{
		int idx_j = 0;

		for (Real j = -halfExtent; j < halfExtent; j+=step_size)
		{
			int idx_k = 0;

			for (Real k = -halfExtent; k < halfExtent; k+=step_size)
			{
				if (hollow)
				{
					int max_n;
					if (N%2 == 0)
						max_n = N-1;
					else
						max_n = N;

					if ((idx_i == 0 || idx_i == max_n) || (idx_j == 0 || idx_j == max_n) || (idx_k == 0 || idx_k == max_n))
						;
					else
						continue;
				}

				mParticle p;

				Real x = i + step_size/2.0 + origin[0];
				Real y = j + step_size/2.0 + origin[1];
				Real z = k + step_size/2.0 + origin[2];

				p.position = RealVector3(x,y,z);
				p.mass = step_size * step_size * step_size * 1000.0;

				p.velocity[0] = v0[0];
				p.velocity[1] = v0[1];
				p.velocity[2] = v0[2];

				p.acceleration[0] = a0[0];
				p.acceleration[1] = a0[1];
				p.acceleration[2] = a0[2];

				particles.push_back(p);

				++idx_k;
			}
			++idx_j;
		}
		++idx_i;
	}
}



void ParticleGenerator::generate_two_freefall_cubes(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.0, 0.0, 0.0);
	RealVector3 v2_init(0.0, 0.0, 0.0);

	RealVector3 g(0.0, 0.0, -0.98);

	generate_cube(particles, N, o1, v1_init, g, 1.0, false, false);
	generate_cube(particles, N, o2, v2_init, g, 1.0, false, false);
}


void ParticleGenerator::generate_two_colliding_cubes(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.25, 0.0, 0.0);
    RealVector3 v2_init(0.00, 0.0, 0.0);

    RealVector3 a(0.0, 0.0, 0.0);

	generate_cube(particles, N, o1, v1_init, a, 1.0, false, false);
	generate_cube(particles, N, o2, v2_init, a, 1.0, false, false);
}

void ParticleGenerator::generate_rigid_box(std::vector<mParticle>& particles, size_t N)
{
	RealVector3 o(0.0, 0.0, 0.0);
	RealVector3 v_init(0.0, 0.0, 0.0);
    RealVector3 a(0.0, 0.0, 0.0);

	generate_cube(particles, N, o, v_init, a, 3.0, true, true);
}
