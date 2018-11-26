#pragma once

#include "math_types.hpp"

using namespace Simulator;  //why here we use this namespace?

namespace Simulator
{
	struct mParticle {
		RealVector3 position;
		RealVector3 velocity;
		RealVector3 acceleration;
        Real mass;   //maybe mass could move to sphsimulator class. because all our particles' mass is the same.

		mParticle() : position(RealVector3(0.0, 0.0, 0.0)), velocity(RealVector3(0.0, 0.0, 0.0)), acceleration(RealVector3(0.0, 0.0, 0.0)), mass(0.0) {}
        mParticle(Real p0,Real p1,Real p2,Real v0,Real v1, Real v2, Real a0, Real a1, Real a2, Real m):
            position(RealVector3(p0, p1, p2)), velocity(RealVector3(v0, v1, v2)), acceleration(RealVector3(a0, a1, a2)), mass(m) {}

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(position[0],position[1],position[2],
          velocity[0],velocity[1],velocity[2],
          acceleration[0],acceleration[1],acceleration[2],
          mass);
    }
    };

	typedef struct mParticle mParticle;


/*
 *  parameter used to describe a cubic.
 */
    class mCuboid{
    public:
        RealVector3 origin;
        int x_n;
        int y_n;
        int z_n;
        bool is_hollow;
    };
}




