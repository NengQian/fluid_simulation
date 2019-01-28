#ifndef SPHSIMULATOR_BULLET_H
#define SPHSIMULATOR_BULLET_H
#include "SPHSimulator_mobile_rigid_body.hpp"

class SPHSimulator_bullet : public SPHSimulator_mobile_rigid_body
{
public:
	SPHSimulator_bullet(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type, int mode)
	 : SPHSimulator_mobile_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type, mode)
	{
        generate_particles();
        set_boundary_attribute();
        sim_rec.boundary_particles = std::vector<mParticle>(boundary_particles.begin(), boundary_particles.begin()+moving_start_idx);
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

            RealVector3 zero = RealVector3(0.0, 0.0, 0.0);
            RealVector3 w_origin = RealVector3(0.0, 0.0, 10.0);
            
            mCuboid water_cuboid;
            water_cuboid.origin = w_origin;
            water_cuboid.x_n = N;
            water_cuboid.y_n = N;
            water_cuboid.z_n = 10*N;
            water_cuboid.is_hollow = false;

            particleGenerator.generate_cuboid_box(particles,zero,water_cuboid,particle_radius,true,false);

            //particleGenerator.generate_rigid_box(boundary_particles, 3*N, particle_radius);
            moving_start_idx = boundary_particles.size();
            
            mCuboid moving_cuboid;
            moving_cuboid.origin = RealVector3(0.0, 0.0, 0.0);

            moving_cuboid.x_n = 1;
            moving_cuboid.y_n = 1;
            moving_cuboid.z_n = 1;

            moving_cuboid.is_hollow = false;
            RealVector3 v = RealVector3(0.0, 0.0, 10.0);
            particleGenerator.generate_cuboid_box(boundary_particles,v,moving_cuboid,particle_radius,false);


            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};

#endif // SPHSIMULATOR_BULLET_H

