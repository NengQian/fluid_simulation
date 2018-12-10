#ifndef SPHSIMULATOR_2CUBES_H
#define SPHSIMULATOR_2CUBES_H
#include "SPHSimulator.hpp"
#include <stdio.h>

#include <cstdlib>

#define WCSPH 0
#define PBFSPH 1

// 2 cubes collsion
class SPHSimulator_2cubes : public SPHSimulator
{
public:
    SPHSimulator_2cubes(int N, Real uParticle_len , Real dt, Real eta, Real B, Real alpha, Real rest_density):SPHSimulator(N,uParticle_len,dt, eta, B, alpha, rest_density){
        generate_particles();
        //update_sim_record_state();
    }

    virtual void update_simulation() override
    {
    	switch(solver_type)
    	{
    		case WCSPH:
    			update_simulation_WCSPH();
    			break;
    		case PBFSPH:
    			update_simulation_PBFSPH();
    			break;
    		default:
    			std::cout << "Error: unknown solver type" << std::endl;
    	        std::exit(-1);
    	}
    }

    virtual void generate_particles() override
    {
        std::cout<<"hey now I am in subclass 2cubes generate"<<std::endl;

        if (!particles.empty())
            particles.clear();

        if (!positions.empty())
            positions.clear();

        particleGenerator.generate_two_colliding_cubes(particles, N, particle_radius*N);
        set_positions();
        neighborSearcher.set_particles_ptr(positions);
    }


protected:
	int solver_type = 1;
	int epoch = 5;

    virtual void update_simulation_WCSPH()
    {
            std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);

            Real r = static_cast<Real>(neighbor_search_radius);
            std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

            std::vector<RealVector3> external_forces;
            for (size_t i=0; i<particles.size(); ++i)
                external_forces.push_back( RealVector3(0.0, 0.0, 0.0) );

            particleFunc.update_acceleration( particles, neighbors_set, densities, external_forces, r);
            particleFunc.update_velocity(particles, dt);
            particleFunc.update_position(particles, dt);

            update_positions();
    }

	void update_simulation_PBFSPH()
	{
		// Step 0: save the current position information before any updates
		std::vector<RealVector3> old_positions;
		for (auto& pos : positions)
		{
			old_positions.push_back(RealVector3(pos));
		}

		// Step 1: preview of particles's status
    	for (size_t i=0; i<particles.size(); ++i)
		{
    		particles[i].velocity += RealVector3(0.0, 0.0, 0.0) * dt;
		}
        particleFunc.update_position(particles, dt);
        update_positions();

        // Step 2: search neighbors
        std::vector< std::vector<size_t> > neighbors_set = neighborSearcher.find_neighbors_within_radius(true);

        // Step 3: iteration of lambda and position computing
        for (int itr=0; itr<epoch; ++itr)
        {
        	// Step 3.0: compute density
            Real r = neighbor_search_radius;
            std::vector<Real> densities = particleFunc.update_density(neighbors_set, particles, r);

            /*
            std::cout << "in " << itr << "-th iteration" << std::endl;
            for (auto& d : densities)
            	std::cout << d << " ";

			std::cout << std::endl;
             */

        	// Step 3.1: compute lambda
        	std::vector<Real> lambda; // only contain lambda of fluid particles, cuz lambda
        	lambda.clear();

        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		mParticle P_i = particles[i];

        		size_t number_of_fluid_neighbors_of_i = neighbors_set[i].size();

        		Real C_i = densities[i] / rest_density - 1;
        		std::vector<RealVector3> grad_of_C_i; // vector with size #fluid_neighbors + #boundary_neighbors
        		grad_of_C_i.clear();

        		Real S_i = 0.0;

        		// compute grad_of_C_i and S_i
        		for (size_t k=0; k<number_of_fluid_neighbors_of_i; ++k)
        		{
        			size_t j;
        			mParticle NP_ij;

        			if (k == 0) // that is, j == i
        			{
        				// skip first, will later compute it by adding up all other gradients
        				grad_of_C_i.push_back(RealVector3(0.0, 0.0, 0.0));
        				continue;
        			}
        			else if (k < number_of_fluid_neighbors_of_i) // j != i and j is fluid neighbor
        			{
            			j = neighbors_set[i][k];
            			NP_ij = particles[j];
           			}

					RealVector3 grad_W = kernelHandler.gradient_of_kernel( P_i.position, NP_ij.position, 4 );
					RealVector3 grad_Cij = (-NP_ij.mass / rest_density) * grad_W;

					grad_of_C_i.push_back(grad_Cij);

					// subtract it with grad_Cii
					grad_of_C_i[0] -= grad_Cij;

					// update S_i, but here got to skip C_ii
					S_i += (grad_Cij[0]*grad_Cij[0] + grad_Cij[1]*grad_Cij[1] + grad_Cij[2]*grad_Cij[2]) / NP_ij.mass;
        		}

        		// add C_ii to complete S_i
        		RealVector3 grad_Cii = grad_of_C_i[0];
        		S_i += (grad_Cii[0]*grad_Cii[0] + grad_Cii[1]*grad_Cii[1] + grad_Cii[2]*grad_Cii[2]) / P_i.mass;

        		// compute lambda_i and push back
        		Real lambda_i = 0.0;

        		if (C_i > 0.0)
        		{
        			lambda_i = -C_i / (S_i + 0.0001);
        		}

        		lambda.push_back(lambda_i);
        	}

        	// Step 3.2: correct position
        	std::vector<RealVector3> dx;
        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		Real lambda_i = lambda[i];
        		mParticle P_i = particles[i];
        		RealVector3 dx_i = RealVector3(0.0, 0.0, 0.0);

        		size_t number_of_fluid_neighbors_of_i = neighbors_set[i].size();

        		// sum up the contribution of all fluid and boundary neighbors
        		for (size_t k=0; k<number_of_fluid_neighbors_of_i; ++k)
        		{
        			size_t j;
        			mParticle NP_ij;
        			Real lambda_j;

        			if (k == 0) // that is, j == i
        			{
        				NP_ij = P_i;
        				lambda_j = lambda_i;
        			}
        			else if (k < number_of_fluid_neighbors_of_i) // j != i and j is fluid neighbor
        			{
            			j = neighbors_set[i][k];
            			NP_ij = particles[j];
            			lambda_j = lambda[j];
           			}

					RealVector3 grad_W_ij = kernelHandler.gradient_of_kernel( P_i.position, NP_ij.position, 4 );

        			dx_i += (lambda_i * NP_ij.mass / P_i.mass + lambda_j) * grad_W_ij;
        		}

        		// computing of dx_i completed here
        		dx_i /= rest_density;

        		dx.push_back(dx_i);

        		// correct position x_i
        		//particles[i].position += dx_i;

        		// Step 4: update velocity
        		//particles[i].velocity = (particles[i].position - old_positions[i]) / dt;

        		//update_positions();
        	}

        	// Step 4: update position after every iteration <---- have to do it separately
        	for (size_t i=0; i<particles.size(); ++i)
        	{
        		particles[i].position += dx[i];
        		update_positions();
        	}
        }

        // Step 5: update velocity after the epochs
    	for (size_t i=0; i<particles.size(); ++i)
    	{
    		particles[i].velocity = (particles[i].position - old_positions[i]) / dt;
    	}
	}

};

#endif // SPHSIMULATOR_2CUBES_H
