#ifndef SPHSIMULATOR_MOBILE_RIGID_BODY_H
#define SPHSIMULATOR_MOBILE_RIGID_BODY_H
#include "SPHSimulator_rigid_body.hpp"
#include "sim_record.hpp"
#include "Particle.hpp"

#include <cstdlib>

#define WCSPH 0
#define PBFSPH 1

class SPHSimulator_mobile_rigid_body : public SPHSimulator_rigid_body
{
public:
    SPHSimulator_mobile_rigid_body(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH, int solver_type, int mode)
	 : SPHSimulator_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH, solver_type), mode(mode)
	{

    }

    virtual void update_simulation() override
    {
		set_boundary_attribute();
		SPHSimulator_rigid_body::update_simulation();
		if (mode == 9) //wave generator
		{	
			particleFunc.update_boundary_position_shm(boundary_particles, moving_start_idx, mid_point, amp, dt, count);
		}
		else if (mode == 10) //moving dam break
		{
			particleFunc.update_boundary_position_moving_dam_break(boundary_particles, moving_start_idx, dt, count);
		} else if (mode == 11)
		{
			particleFunc.update_boundary_position_watermill(boundary_particles, moving_start_idx, rotation_center, dt, count);
		}
		set_boundary_positions();
		neighborSearcher.set_boundary_particles_ptr(boundary_positions);
		++count;
    }

	virtual void update_sim_record_state() override
	{
		SimulationState sim_state;
    	sim_state.particles = particles;
		sim_state.moving_boundary_particles = std::vector<mParticle>(boundary_particles.begin()+moving_start_idx, boundary_particles.end());
   		sim_rec.states.push_back(sim_state);
	}

protected:
	int count = 0;
	int moving_start_idx;
	double mid_point;
	double amp;
	RealVector3 rotation_center;

	int mode;
};

#endif // SPHSIMULATOR_MOBILE_RIGID_BODY_H
