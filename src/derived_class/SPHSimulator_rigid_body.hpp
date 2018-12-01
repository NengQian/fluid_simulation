#ifndef SPHSIMULATOR_RIGID_BODY_H
#define SPHSIMULATOR_RIGID_BODY_H
#include "SPHSimulator.hpp"


class SPHSimulator_rigid_body : public SPHSimulator
{
public:
	bool viscosity_flag = true;
	bool XSPH_flag = true;


    SPHSimulator_rigid_body(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH):SPHSimulator(N, uParticle_len,dt, eta, B, alpha, rest_density){

        generate_particles();
        set_boundary_volumes();
        sim_rec.boundary_particles = boundary_particles;
        //update_sim_record_state();

        if (with_viscosity == 1)
        	viscosity_flag = true;
        else
        	viscosity_flag = false;

        if (with_XSPH == 1)
        	XSPH_flag = true;
        else
        	XSPH_flag = false;

    }

    virtual void update_simulation() override
    {
        //float fr = 2.4/N * 2 ;
        //set_neighbor_search_radius(fr);

        std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        Real r = neighbor_search_radius;
        std::vector<Real> densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, boundary_volumes, r);
        //std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

        //Real water_rest_density = 1000.0;
        //Real B = 100.0;

        std::vector<RealVector3> external_forces;
        for (size_t i=0; i<particles.size(); ++i)
            //external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) ); //Neng: we have the gravity in class private
            external_forces.push_back( gravity * particles[i].mass ); //Neng: we have the gravity in class private

        particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, boundary_volumes, external_forces, r, viscosity_flag);
        //particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);


        if (XSPH_flag == false)
        {
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt);
        } else {
            /* --------- using XSPH -------------------*/
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt, neighbors_set, r, densities);
        }

        update_positions();;
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

            /*
            test_cuboid.x_n = N+6;
            test_cuboid.y_n = static_cast<int>(N/4)*N+6;
            */

            test_cuboid.x_n = N*2;
            test_cuboid.y_n = N*2;
            test_cuboid.z_n = 3*N;
            test_cuboid.is_hollow = true;
            particleGenerator.generate_cuboid_box(boundary_particles,zero,zero,test_cuboid,particle_radius,false);

//            mCuboid test_cuboid2;
//            test_cuboid2.origin = RealVector3(0.0, particle_radius, 2*particle_radius);
//            test_cuboid2.x_n = 1;
//            test_cuboid2.y_n = 1;
//            test_cuboid2.z_n = 1;
//            test_cuboid2.is_hollow = false;
//            particleGenerator.generate_cuboid_box(particles,zero,zero,test_cuboid2,particle_radius,false);
            ///////only for test, we generate two cubic, consist of a cuboid
            RealVector3 origin = RealVector3(0.0, 0.0, particle_radius);
            //particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);

            origin = RealVector3(0.0, 0.0, 3*particle_radius);
            particleGenerator.generate_cube(particles, N, origin, zero, zero, particle_radius*N, false, false);


            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }


protected:
    // should I also move all boundary related function to here? Or maybe not...

};

#endif // SPHSIMULATOR_RIGID_BODY_H
