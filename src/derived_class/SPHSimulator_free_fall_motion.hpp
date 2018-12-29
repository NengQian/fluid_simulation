#ifndef SPHSIMULATOR_FREE_FALL_MOTION_H
#define SPHSIMULATOR_FREE_FALL_MOTION_H
#include "SPHSimulator.hpp"
#include <stdio.h>

class SPHSimulator_free_fall_motion: public SPHSimulator
{
public:
    SPHSimulator_free_fall_motion(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density)
								 :SPHSimulator(N,  uParticle_len, dt, eta, B, alpha, rest_density)
	{
        generate_particles();
        //update_sim_record_state();
    }


    virtual void update_simulation() override
    {
        particleFunc.update_velocity(particles, dt, SPHSimulator::gravity);
        particleFunc.update_position(particles, dt);

        update_positions();
    }

    virtual void generate_particles() override
    {
            if (!particles.empty())
                particles.clear();

            if (!positions.empty())
                positions.clear();

            particleGenerator.generate_two_freefall_cubes(particles, N, particle_radius*N);
            set_positions();
            neighborSearcher.set_particles_ptr(positions);
    }

};

#endif // SPHSIMULATOR_FREE_FALL_MOTION_H
