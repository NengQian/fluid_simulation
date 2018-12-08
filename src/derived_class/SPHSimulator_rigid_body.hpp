#ifndef SPHSIMULATOR_RIGID_BODY_H
#define SPHSIMULATOR_RIGID_BODY_H
#include "SPHSimulator.hpp"


class SPHSimulator_rigid_body : public SPHSimulator
{
public:
	bool viscosity_flag = true;
	bool XSPH_flag = true;

    SPHSimulator_rigid_body(int N,  Real uParticle_len, Real dt, Real eta, Real B, Real alpha, Real rest_density, int with_viscosity, int with_XSPH)
	 : SPHSimulator(N, uParticle_len,dt, eta, B, alpha, rest_density)
	{
        if (with_viscosity == 1)
        	viscosity_flag = true;
        else
        	viscosity_flag = false;

        if (with_XSPH == 1)
        	XSPH_flag = true;
        else
        	XSPH_flag = false;

    }

    virtual void update_simulation() override
    {
        //float fr = 2.4/N * 2 ;
        //set_neighbor_search_radius(fr);

        std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);
        std::vector< std::vector<size_t> > neighbors_in_boundary = neighborSearcher.find_neighbors_in_boundary( );

        Real r = neighbor_search_radius;
        std::vector<Real> densities = particleFunc.update_density(neighbors_set, neighbors_in_boundary, particles, boundary_particles, r);
        //std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

        //Real water_rest_density = 1000.0;
        //Real B = 100.0;

        std::vector<RealVector3> external_forces;
        for (size_t i=0; i<particles.size(); ++i)
            //external_forces.push_back( RealVector3(0.0, 0.0, -0.981 * particles[i].mass) ); //Neng: we have the gravity in class private
            external_forces.push_back( gravity * particles[i].mass ); //Neng: we have the gravity in class private

        particleFunc.update_acceleration( particles, boundary_particles, neighbors_set, neighbors_in_boundary, densities, external_forces, r, viscosity_flag);
        //particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, water_rest_density, r, B);


        if (XSPH_flag == false)
        {
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt);
        } else {
            /* --------- using XSPH -------------------*/
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt, neighbors_set, r, densities);
        }

        update_positions();;
    }

    //virtual void generate_particles() = 0;

protected:
    // should I also move all boundary related function to here? Or maybe not...

};

#endif // SPHSIMULATOR_RIGID_BODY_H
