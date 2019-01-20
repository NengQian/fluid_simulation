#ifndef SPHSIMULATOR_DROP_ON_WATER_H
#define SPHSIMULATOR_DROP_ON_WATER_H
#include "SPHSimulator_rigid_body.hpp"

class SPHSimulator_drop_on_water : public SPHSimulator_rigid_body
{
public:
	SPHSimulator_drop_on_water(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type)
	 : SPHSimulator_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type)
	{
        generate_particles();
        set_boundary_attribute();
        sim_rec.boundary_particles = boundary_particles;
        //update_sim_record_state();
	}

	virtual void generate_particles() override
    {
            if (!particles.empty())
                particles.clear();

            if (!positions.empty())
                positions.clear();

            if (!boundary_particles.empty())
                boundary_particles.clear();

            if (!boundary_positions.empty())
                boundary_positions.clear();

            RealVector3 zero(0.0, 0.0, 0.0);

            //particleGenerator.generate_rigid_box(boundary_particles, 3*N, particle_radius);
            mCuboid test_cuboid;
            test_cuboid.origin = zero;

            test_cuboid.x_n = 3*N;
            test_cuboid.y_n = 3*N;
            test_cuboid.z_n = 3*N;

            if (N == 1)
            {
                test_cuboid.x_n = 9;
                test_cuboid.y_n = 9;
                test_cuboid.z_n = 1;
            }

            test_cuboid.is_hollow = true;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,test_cuboid,particle_radius,false);

            RealVector3 origin = RealVector3(0.0, 0.0, particle_radius);
            //particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);

            origin = RealVector3(0.0, 0.0, 3*particle_radius);
            
            mCuboid water_cuboid;
            water_cuboid.origin = origin;
            water_cuboid.x_n = 3*N-4;
            water_cuboid.y_n = 3*N-4;
            water_cuboid.z_n = N-5;
            water_cuboid.is_hollow = false;

            particleGenerator.generate_cuboid_box(particles,zero,water_cuboid,particle_radius,true);
            int ps = particles.size();
            std::vector<bool> set1(ps, true);

            origin = RealVector3(0.0, 0.0, 6*N*particle_radius);
            water_cuboid.origin = origin;
            water_cuboid.x_n = N;
            water_cuboid.y_n = N;
            water_cuboid.z_n = N;

            particleGenerator.generate_cuboid_box(particles,zero,water_cuboid,particle_radius,false);
            std::vector<bool> set2(ps, false);

            sim_rec.sets = set2;
            sim_rec.sets.insert(sim_rec.sets.end(), set1.begin(), set1.end());
            //

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};

#endif // SPHSIMULATOR_DROP_ON_WATER_H

