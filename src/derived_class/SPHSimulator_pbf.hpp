#ifndef SPHSIMULATOR_PBF_H
#define SPHSIMULATOR_PBF_H
#include "SPHSimulator_rigid_body.hpp"


class SPHSimulator_PBF:public SPHSimulator_rigid_body
{
public:
    SPHSimulator_PBF(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH)
     : SPHSimulator_rigid_body(N, uParticle_len, dt, eta, B, alpha, rest_density, with_viscosity, with_XSPH)
    {
        generate_particles();
        SPHSimulator::set_boundary_volumes();
        SPHSimulator::sim_rec.boundary_particles = boundary_particles;
        //update_sim_record_state();
    }


    virtual void generate_particles() override
    {

    }

    virtual void update_simulation() override
    {

    }
};

#endif // SPHSIMULATOR_PBF_H
