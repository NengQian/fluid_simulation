#include "ParticleGenerator.hpp"

#include "math_types.hpp"
#include "Particle.hpp"

#include <merely3d/merely3d.hpp>
#include <CompactNSearch/CompactNSearch>

#include <random>
#include <cmath>
#include <iostream>
#include <cstdlib>


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
using namespace std;

void ParticleGenerator::generate_cuboid_box(std::vector<mParticle>& particles,
                         Eigen::Ref<RealVector3> v0,
                         mCuboid cuboid,   // each cuboid contains x,y,z number of particles, and origin;
                         Real radius,
                         bool do_clear)
{
    if (do_clear)
        if (!particles.empty())
            particles.clear();
    Real step_size = 2*radius;
    double x_half_extent = step_size*cuboid.x_n/2.0;
    double y_half_extent = step_size*cuboid.y_n/2.0;
    double z_half_extent = step_size*cuboid.z_n/2.0;
    bool hollow = cuboid.is_hollow;
    //int idx_i = 0;

    for (int k=0; k<cuboid.z_n; ++k)
    {
    	for (int j=0; j<cuboid.y_n; ++j)
    	{
    		for (int i=0; i<cuboid.x_n; ++i)
    		{
                if (hollow)
                {
                    int max_x_n = cuboid.x_n-1;
                    int max_y_n = cuboid.y_n-1;
                    if ((k == 0) || (j == 0 || j == max_y_n) || (i == 0 || i == max_x_n))
                        ;
                    else
                        continue;
                }

                mParticle p;

                Real x = (i+0.5)*step_size-x_half_extent + cuboid.origin[0];
                Real y = (j+0.5)*step_size-y_half_extent + cuboid.origin[1];
                Real z = (k+0.5)*step_size + cuboid.origin[2];

                p.position = RealVector3(x,y,z);
                p.mass = step_size * step_size * step_size * 1000.0;

                p.velocity[0] = v0[0];
                p.velocity[1] = v0[1];
                p.velocity[2] = v0[2];

                p.density = 0.0;

                particles.push_back(p);
    		}
    	}
    }
}

void ParticleGenerator::generate_cube(std::vector<mParticle>& particles, size_t N, Eigen::Ref<RealVector3> origin, Eigen::Ref<RealVector3> v0, Real halfExtent, bool do_clear, bool hollow)
{
	if (do_clear)
		if (!particles.empty())
			particles.clear();

	Real step_size = 2.0 * halfExtent / N;

    for (int k=0; k<N; ++k)
    {
    	for (int j=0; j<N; ++j)
    	{
    		for (int i=0; i<N; ++i)
    		{
                if (hollow)
                {
                    int max_n = N-1;
                    if ((k == 0) || (j == 0 || j == max_n) || (i == 0 || i == max_n))
                        ;
                    else
                        continue;
                }

                mParticle p;

                Real x = (i+0.5)*step_size-halfExtent + origin[0];
                Real y = (j+0.5)*step_size-halfExtent + origin[1];
                Real z = (k+0.5)*step_size + origin[2];

                p.position = RealVector3(x,y,z);
                p.mass = step_size * step_size * step_size * 1000.0;

                p.velocity[0] = v0[0];
                p.velocity[1] = v0[1];
                p.velocity[2] = v0[2];

                p.density = 0.0;

                particles.push_back(p);
    		}
    	}
    }
}

void ParticleGenerator::generate_two_freefall_cubes(std::vector<mParticle>& particles, size_t N, Real radius)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.0, 0.0, 0.0);
	RealVector3 v2_init(0.0, 0.0, 0.0);

    generate_cube(particles, N, o1, v1_init, radius, true, false);
    generate_cube(particles, N, o2, v2_init, radius, false, false);
}


void ParticleGenerator::generate_two_colliding_cubes(std::vector<mParticle>& particles, size_t N, Real radius)
{
	RealVector3 o1(0.0, 0.0, 0.0);
	RealVector3 o2(2.5, 0.0, 0.0);

	RealVector3 v1_init(0.25, 0.0, 0.0);
    RealVector3 v2_init(0.00, 0.0, 0.0);

    generate_cube(particles, N, o1, v1_init, radius, true);
    generate_cube(particles, N, o2, v2_init, radius, false);
}

void ParticleGenerator::generate_rigid_box(std::vector<mParticle>& particles, size_t N, Real radius)
{
	RealVector3 o(0.0, 0.0, 0.0);
	RealVector3 v_init(0.0, 0.0, 0.0);

    generate_cube(particles, N, o, v_init, N*radius, true, true);
}


    void ParticleGenerator::generate_sphere(std::vector<mParticle>& particles,
                         Eigen::Ref<RealVector3> v0,
                         mSphere sp,   // each cuboid contains number of particles per x,y,z. and also origin;
                         Real radius,
                         bool do_clear){
        if (do_clear)
            if (!particles.empty())
                particles.clear();
        Real step_size = 2*radius;
//        double x_half_extent = step_size*sp.radius_n;
//        double y_half_extent = step_size*sp.radius_n;
//        double z_half_extent = step_size*sp.radius_n;

        for (int z=-sp.radius_n; z<sp.radius_n; ++z)
        {
            for (int y=-sp.radius_n; y<sp.radius_n; ++y)
            {
                for (int x=-sp.radius_n; x<sp.radius_n; ++x)
                {
                    // we assume a x,y,z construct a voxel whose edge lengths are all 1;
                    double x_double = static_cast<double>(x)+0.5;  //take the center of this voxel!!
                    double y_double = static_cast<double>(y)+0.5;
                    double z_double = static_cast<double>(z)+0.5;
                    double dist_sqr = x_double*x_double + y_double*y_double + z_double*z_double;
                    double sphere_r_sqr = static_cast<double>(sp.radius_n*sp.radius_n);
                    cout<<"dist_sqr - sphere_r"<< dist_sqr-sphere_r_sqr<<endl;
                    if(abs(sqrt(dist_sqr) - sqrt(sphere_r_sqr))<=0.75+0.000001) // sqrt(3)/2 , the biggest error in this voxel
                    {
                        mParticle p;
                        Real px = sp.Center[0]+x*step_size+0.5*step_size;
                        Real py = sp.Center[1]+y*step_size+0.5*step_size;
                        Real pz = sp.Center[2]+z*step_size+0.5*step_size;
                        p.position = RealVector3(px,py,pz);
                        p.mass = step_size * step_size * step_size * 1000.0;
                        p.velocity[0] = v0[0];
                        p.velocity[1] = v0[1];
                        p.velocity[2] = v0[2];
                        p.density = 0.0;
                        particles.push_back(p);
                    }
                }
            }
        }
        return;
    }
