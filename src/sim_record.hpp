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
        std :: vector < SimulationState > states ;

        SimulationRecord()
        {
            timestep=0;
            states.clear();
        }

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(timestep, states);
        }
    };

    typedef struct SimulationRecord SiumlationRecord;
}

