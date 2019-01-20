#pragma once

#include <Particle.hpp>
#include "math_types.hpp"

using namespace Simulator;  //why here we use this namespace?

namespace Simulator
{
    struct SimulationState {
        std :: vector < mParticle > particles ;


        SimulationState()
        {
            particles.clear();
        }

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(particles);
        }
    };

    typedef struct SimulationState SimulationState;

    struct SimulationRecord {
        Real timestep;
        Real unit_particle_length;
        Real eta;
        Real rest_density;
        Real B;
        Real alpha;
        int solver_type;

        std::vector<SimulationState> states ;
        std::vector<mParticle> boundary_particles;
        std::vector<bool> sets; 

        SimulationRecord()
        {
            timestep = 0;
            unit_particle_length = 0;
            eta = 0;
            rest_density = 0;
            B = 0;
            alpha = 0;
            solver_type = -1;

            states.clear();
            boundary_particles.clear();
        }

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(timestep, unit_particle_length, eta, rest_density, B, alpha, solver_type, boundary_particles, states, sets);
        }
    };

    typedef struct SimulationRecord SiumlationRecord;
}

