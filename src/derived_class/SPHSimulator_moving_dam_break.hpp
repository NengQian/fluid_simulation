#ifndef SPHSIMULATOR_MOVING_DAM_BREAK_H
#define SPHSIMULATOR_MOVING_DAM_BREAK_H
#include "SPHSimulator_mobile_rigid_body.hpp"

class SPHSimulator_moving_dam_break : public SPHSimulator_mobile_rigid_body
{
public:
	SPHSimulator_moving_dam_break(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type, int mode)
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

            RealVector3 origin(0.0, 0.0, 0.0);
            RealVector3 zero(0.0, 0.0, 0.0);

            //particleGenerator.generate_rigid_box(boundary_particles, 3*N, particle_radius);
            mCuboid upper_cuboid;
            upper_cuboid.origin = origin;

            upper_cuboid.x_n = N+6;
            upper_cuboid.y_n = 10*N+6;
            upper_cuboid.z_n = 10*N;

            upper_cuboid.is_hollow = true;
            upper_cuboid.is_closed = false;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,upper_cuboid,particle_radius,false, false);

            moving_start_idx = boundary_particles.size();
            
            mCuboid moving_cuboid;
            moving_cuboid.origin = origin + RealVector3(0.0, particle_radius*(upper_cuboid.y_n-2*N-12), 0.0);

            moving_cuboid.x_n = N+6;
            moving_cuboid.y_n = 1;
            moving_cuboid.z_n = 10*N;

            moving_cuboid.is_hollow = false;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,moving_cuboid,particle_radius,false);
            
            RealVector3 w_origin = RealVector3(0.0, particle_radius*(upper_cuboid.y_n-N-6), 3*particle_radius);
            
            mCuboid water_cuboid;
            water_cuboid.origin = w_origin+origin;
            water_cuboid.x_n = N;
            water_cuboid.y_n = N;
            water_cuboid.z_n = 6*N;
            water_cuboid.is_hollow = false;

            particleGenerator.generate_cuboid_box(particles,zero,water_cuboid,particle_radius,true,false);

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};

#endif // SPHSIMULATOR_MOVING_DAM_BREAK_H

