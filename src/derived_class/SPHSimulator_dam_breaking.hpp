#ifndef SPHSIMULATOR_DAM_BREAKING_H
#define SPHSIMULATOR_DAM_BREAKING_H
#include "SPHSimulator_rigid_body.hpp"

class SPHSimulator_dam_breaking : public SPHSimulator_rigid_body
{
public:
	SPHSimulator_dam_breaking(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH)
	 : SPHSimulator_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH)
	{
        generate_particles();
        SPHSimulator::set_boundary_attribute();
        SPHSimulator::sim_rec.boundary_particles = boundary_particles;
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

            test_cuboid.x_n = N+6;
            test_cuboid.y_n = static_cast<int>(N/4)*N+6;
            test_cuboid.z_n = 3*N;
            test_cuboid.is_hollow = true;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,zero,test_cuboid,particle_radius,false);

            RealVector3 origin = RealVector3(0.0, 0.0, particle_radius);
            //particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);

            origin = RealVector3(0.0, particle_radius*(test_cuboid.y_n-N-6), 3*particle_radius);
            particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};

#endif // SPHSIMULATOR_DAM_BREAKING_H
