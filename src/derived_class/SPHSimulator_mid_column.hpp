#ifndef MID_COLUMN_H
#define MID_COLUMN_H
#include "SPHSimulator_rigid_body.hpp"
#include <cmath>

class SPHSimulator_mid_column : public SPHSimulator_rigid_body
{
public:    
    SPHSimulator_mid_column(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type)
     : SPHSimulator_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type)
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

            // generate the outside big cuboid boundary
            mCuboid out_box;
            out_box.origin = zero;
            out_box.x_n = N+20;
            out_box.y_n = N+20;
            out_box.z_n = 3*N;
            out_box.is_hollow = true;
            out_box.is_closed = false;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,out_box,particle_radius,false);

            // generate the mid thin column cuboid boundary
            mCuboid mid_col;
            mid_col.origin = RealVector3(0.0, 0.0, 14*particle_radius);
            mid_col.x_n = N+2;
            mid_col.y_n = N+2;
            mid_col.z_n = 20;
            mid_col.is_hollow = false;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,mid_col,particle_radius,false);

            //for toy, generate a sphere boundary
            mSphere sp;
            sp.Center = RealVector3(0.0, 0.0 , mid_col.origin[2]+(mid_col.z_n*2)*particle_radius);
            sp.radius_n = N/2+1;
            particleGenerator.generate_sphere(boundary_particles,zero,sp,particle_radius,false);

            // generate the fluid particle
            RealVector3 origin = RealVector3(0.0, 0.0, (sp.Center[2])+(sp.radius_n*2+20)*particle_radius);
            //particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);
            particleGenerator.generate_cube(particles, N, origin, zero, particle_radius*N, false, false);

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }
};



#endif // MID_COLUMN_H
