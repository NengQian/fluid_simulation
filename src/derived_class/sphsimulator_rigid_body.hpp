#ifndef SPHSIMULATOR_RIGID_BODY_H
#define SPHSIMULATOR_RIGID_BODY_H
#include "SPHSimulator.hpp"


class SPHSimulator_rigid_body : public SPHSimulator
{
public:
    SPHSimulator_rigid_body(Real dt, int N=5):SPHSimulator(dt,N){
        generate_particles();
        set_boundary_volumes();
        update_sim_record_state();
    }

    virtual void update_simulation() override{
        float r = 2.4/N * 2 ;
        set_neighbor_search_radius(r);

        std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        std::vector<Real> densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, boundary_volumes, r);
        //std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

        Real water_rest_density = 1000.0;
        Real B = 100.0;

        std::vector<RealVector3> external_forces;
        for (size_t i=0; i<particles.size(); ++i)
            //external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) ); //Neng: we have the gravity in class private
            external_forces.push_back( gravity * particles[i].mass ); //Neng: we have the gravity in class private

        particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, boundary_volumes, external_forces, water_rest_density, r, B);
        //particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);

        particleFunc.update_velocity(particles, dt);
        particleFunc.update_position(particles, dt);

        update_positions();;
    }



    virtual void generate_particles(){
            if (!particles.empty())
                particles.clear();

            if (!positions.empty())
                positions.clear();

            if (!boundary_particles.empty())
                boundary_particles.clear();

            if (!boundary_positions.empty())
                boundary_positions.clear();

            RealVector3 zero(0.0, 0.0, 0.0);

            particleGenerator.generate_rigid_box(boundary_particles, 3*N);
            particleGenerator.generate_cube(particles, N, zero, zero, zero);

            set_positions();
            set_boundary_positions();

            neighborSearcher.set_particles_ptr(positions);
            neighborSearcher.set_boundary_particles_ptr(boundary_positions);
    }


protected:
    // should I also move all boundary related function to here? Or maybe not...


};

#endif // SPHSIMULATOR_RIGID_BODY_H
