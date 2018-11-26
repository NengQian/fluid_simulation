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
        Real timestep ;
        Real unit_particle_length;
        std :: vector < SimulationState > states ;
        std:: vector<mParticle> boundary_particles;

        SimulationRecord()
        {
            timestep=0;
            unit_particle_length=0;
            states.clear();
            boundary_particles.clear();
        }

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(timestep,unit_particle_length, boundary_particles,states);
        }
    };

    typedef struct SimulationRecord SiumlationRecord;
}

