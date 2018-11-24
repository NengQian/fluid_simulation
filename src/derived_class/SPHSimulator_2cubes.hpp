#ifndef SPHSIMULATOR_2CUBES_H
#define SPHSIMULATOR_2CUBES_H
#include "SPHSimulator.hpp"
#include <stdio.h>
// 2 cubes collsion
class SPHSimulator_2cubes : public SPHSimulator
{
public:
    SPHSimulator_2cubes(Real dt, int N=5):SPHSimulator(dt,N){
        generate_particles();
        update_sim_record_state();
    }

    virtual void update_simulation() override
    {
            std::cout<<"hey now I am in subclass 2cubes"<<std::endl;
            set_neighbor_search_radius( 2.4/N * 2 );
            std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);

            Real r = static_cast<Real>(neighbor_search_radius);
            std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

            Real water_rest_density = 1000.0;
            Real B = 1000.0;

            std::vector<RealVector3> external_forces;
            for (size_t i=0; i<particles.size(); ++i)
                external_forces.push_back( RealVector3(0.0, 0.0, 0.0) );

            particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt);

            update_positions();
    }

    virtual void generate_particles() override
    {
        std::cout<<"hey now I am in subclass 2cubes generate"<<std::endl;

        if (!particles.empty())
            particles.clear();

        if (!positions.empty())
            positions.clear();

        particleGenerator.generate_two_colliding_cubes(particles, N);
        set_positions();
        neighborSearcher.set_particles_ptr(positions);
    }

};

#endif // SPHSIMULATOR_2CUBES_H
