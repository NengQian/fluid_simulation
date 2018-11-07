#pragma once

#include "math_types.hpp"

using namespace Simulator;

namespace Simulator
{
	struct mParticle {
		RealVector3 position;
		RealVector3 velocity;
		RealVector3 acceleration;
		Real mass;

		mParticle() : position(RealVector3(0.0, 0.0, 0.0)), velocity(RealVector3(0.0, 0.0, 0.0)), acceleration(RealVector3(0.0, 0.0, 0.0)), mass(0.0) {}
	};

	typedef struct mParticle mParticle;
}
