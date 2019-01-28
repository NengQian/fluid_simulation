#ifndef SPHSIMULATOR_WATERMILL_H
#define SPHSIMULATOR_WATERMILL_H
#include "SPHSimulator_mobile_rigid_body.hpp"

class SPHSimulator_watermill : public SPHSimulator_mobile_rigid_body
{
public:
	SPHSimulator_watermill(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type, int mode)
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

            RealVector3 zero(0.0, 0.0, 0.0);

            //particleGenerator.generate_rigid_box(boundary_particles, 3*N, particle_radius);
            mCuboid fix_cuboid;
            fix_cuboid.origin = zero;

            fix_cuboid.x_n = 3*N;
            fix_cuboid.y_n = 3*N;
            fix_cuboid.z_n = 3*N;

            fix_cuboid.is_hollow = true;
            fix_cuboid.is_closed = true;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,fix_cuboid,particle_radius,false);

            moving_start_idx = boundary_particles.size();
	        rotation_center = RealVector3(0.0, 0.0, 3*N*particle_radius);

            int total_box = 2;
            for (int i=0; i<total_box; ++i)
            {
                mCuboid moving_cuboid;
                moving_cuboid.origin = RealVector3(0.0, particle_radius*(0.6*fix_cuboid.y_n), fix_cuboid.z_n*particle_radius);

                moving_cuboid.x_n = N;
                moving_cuboid.y_n = N;
                moving_cuboid.z_n = 4;

                moving_cuboid.is_hollow = true;
                moving_cuboid.is_closed = false;
                particleGenerator.generate_cuboid_box(boundary_particles,zero,moving_cuboid,particle_radius,false,false,true, 2*i*M_PI/total_box, rotation_center[1], rotation_center[2]); 
            }

            RealVector3 origin = RealVector3(0.0, 0.0, particle_radius);
            //particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);

            origin = RealVector3(0.0, 0.0, 3*particle_radius);
            
            mCuboid water_cuboid;
            water_cuboid.origin = origin;
            water_cuboid.x_n = 3*N-6;
            water_cuboid.y_n = 3*N-6;
            water_cuboid.z_n = 0.5*N;
            water_cuboid.is_hollow = false;

            particleGenerator.generate_cuboid_box(particles,zero,water_cuboid,particle_radius,true);
            //

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};

#endif // SPHSIMULATOR_WATERMILL_H

